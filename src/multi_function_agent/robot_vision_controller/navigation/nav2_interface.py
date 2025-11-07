"""
Nav2 Interface Module
Python interface for Nav2 navigation stack integration.
"""

import time
import logging
import threading
from enum import Enum
from typing import Optional, Tuple, Callable

logger = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool
    from tf_transformations import quaternion_from_euler
    
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
    logger.warning("Nav2 packages not available - using fallback mode")
    
    # Mock all classes for graceful degradation
    class Node:
        def __init__(self, name):
            self.name = name
        def get_clock(self):
            class Clock:
                def now(self):
                    class Time:
                        def to_msg(self): return None
                    return Time()
            return Clock()
        def create_subscription(self, *args, **kwargs):
            return None
    
    class ActionClient:
        def __init__(self, *args, **kwargs):
            pass
        def wait_for_server(self, timeout_sec=1.0):
            return False
    
    NavigateToPose = None
    Path = None
    PoseStamped = type('PoseStamped', (), {
        'header': type('Header', (), {'frame_id': '', 'stamp': None})(),
        'pose': type('Pose', (), {
            'position': type('Point', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
            'orientation': type('Quaternion', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})()
        })()
    })
    Pose = None
    Point = None
    Quaternion = None
    Bool = None
    
    def quaternion_from_euler(*args, **kwargs):
        return [0, 0, 0, 1]

# =============================================================================
# Navigation State
# =============================================================================

class NavigationState(Enum):
    """Nav2 navigation states."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


# =============================================================================
# Nav2 Interface
# =============================================================================

class Nav2Interface(Node):
    """
    ROS2 interface for Nav2 navigation stack.
    Handles goal sending, path monitoring, and navigation feedback.
    """
    
    def __init__(self):
        """Initialize Nav2 interface."""
        if not NAV2_AVAILABLE:
            logger.error("Nav2 not available - ROS2 packages not found")
            raise ImportError("Nav2 requires ROS2 navigation packages")
        
        super().__init__('nav2_interface_node')
        
        # Navigation state
        self.state = NavigationState.IDLE
        self.current_goal = None
        self.current_goal_handle = None
        self.navigation_result = None
        
        # Action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Path subscriber (optional - for visualization)
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self._path_callback,
            10
        )
        
        self.current_path = None
        
        # Callbacks
        self.goal_reached_callback: Optional[Callable] = None
        self.goal_failed_callback: Optional[Callable] = None
        
        logger.info("Nav2Interface initialized")
    
    def wait_for_nav2(self, timeout: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to be ready.
        """
        logger.info("Waiting for Nav2 action server...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                logger.info("✅ Nav2 action server ready")
                return True
            
            logger.debug("Nav2 not ready yet...")
        
        logger.error("❌ Nav2 action server timeout")
        return False
    
    def send_goal(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        frame_id: str = 'map'
    ) -> bool:
        """
        Send navigation goal to Nav2.
        """
        if self.state == NavigationState.NAVIGATING:
            logger.warning("Already navigating - cancelling current goal")
            self.cancel_navigation()
            time.sleep(0.2)
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(x, y, theta, frame_id)
        
        logger.info(f"📍 Sending goal: ({x:.2f}, {y:.2f}, {theta:.2f}rad)")
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            logger.error("❌ Goal rejected by Nav2")
            self.state = NavigationState.FAILED
            return False
        
        logger.info("✅ Goal accepted by Nav2")
        self.state = NavigationState.NAVIGATING
        self.current_goal = (x, y, theta)
        self.current_goal_handle = goal_handle
        
        # Register result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        
        return True
    
    def cancel_navigation(self) -> bool:
        """
        Cancel current navigation goal.
        """
        if self.state != NavigationState.NAVIGATING:
            logger.warning("No active navigation to cancel")
            return False
        
        if not self.current_goal_handle:
            logger.error("No goal handle available for cancellation")
            return False
        
        logger.warning("🛑 Cancelling navigation")
        
        # Cancel via action client
        cancel_future = self.nav_to_pose_client._cancel_goal_async()
        
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
        
        self.state = NavigationState.CANCELLED
        self.current_goal_handle = None
        logger.info("Navigation cancelled")
        
        return True
    
    def get_state(self) -> NavigationState:
        """Get current navigation state."""
        return self.state
    
    def is_navigating(self) -> bool:
        """Check if currently navigating."""
        return self.state == NavigationState.NAVIGATING
    
    def get_current_path(self) -> Optional[Path]: # type: ignore
        """Get current planned path from Nav2."""
        return self.current_path
    
    def _create_pose_stamped(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str
    ) -> PoseStamped: # type: ignore
        """Create PoseStamped message from coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        
        # Extract useful info
        distance_remaining = feedback.distance_remaining
        estimated_time = feedback.estimated_time_remaining.sec
        
        logger.debug(
            f"[NAV2 FEEDBACK] Distance: {distance_remaining:.2f}m, "
            f"ETA: {estimated_time}s"
        )
    
    def _result_callback(self, future):
        """Handle navigation result from Nav2."""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            logger.info("Navigation SUCCEEDED")
            self.state = NavigationState.SUCCEEDED
            
            if self.goal_reached_callback:
                self.goal_reached_callback()
        
        else:
            logger.warning(f"Navigation FAILED (status: {result.status})")
            self.state = NavigationState.FAILED
            
            if self.goal_failed_callback:
                self.goal_failed_callback(result.status)
        
        self.navigation_result = result
        self.current_goal_handle = None
    
    def _path_callback(self, msg: Path): # type: ignore
        """Handle planned path updates from Nav2."""
        self.current_path = msg
        logger.debug(f"Path updated: {len(msg.poses)} waypoints")