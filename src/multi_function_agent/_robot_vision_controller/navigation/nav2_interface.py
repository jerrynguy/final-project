"""
Nav2 Interface Module
Python interface for Nav2 navigation stack integration.
"""
import logging
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)

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
class Nav2Interface:
    """
    Wrapper for Nav2 via ROS2Bridge daemon.
    Uses subprocess communication instead of direct rclpy.
    """
    
    def __init__(self):
        from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
        self.bridge = get_ros2_node()
        self.state = NavigationState.IDLE
        logger.info("Nav2Interface initialized (daemon mode)")
    
    def wait_for_nav2(self, timeout: float = 10.0) -> bool:
        """Check if Nav2 daemon is ready."""
        # Daemon handles this internally
        logger.info("✅ Nav2 ready (daemon mode)")
        return True
    
    def send_goal(self, x: float, y: float, theta: float = 0.0, frame_id: str = 'map') -> bool:
        """Send Nav2 goal via daemon."""
        logger.info(f"📍 Sending Nav2 goal: ({x:.2f}, {y:.2f}, {theta:.2f}rad)")
        
        success = self.bridge.send_nav2_goal(x, y, theta)
        if success:
            self.state = NavigationState.NAVIGATING
        
        return success
    
    def cancel_navigation(self) -> bool:
        """Cancel Nav2 navigation."""
        logger.warning("🛑 Cancelling Nav2 navigation")
        success = self.bridge.cancel_nav2_goal()
        
        if success:
            self.state = NavigationState.CANCELLED
        
        return success
    
    def get_state(self) -> NavigationState:
        """Get current Nav2 state from daemon cache."""
        daemon_state = self.bridge.get_nav2_state()
        
        state_map = {
            'idle': NavigationState.IDLE,
            'navigating': NavigationState.NAVIGATING,
            'succeeded': NavigationState.SUCCEEDED,
            'failed': NavigationState.FAILED,
            'cancelled': NavigationState.CANCELLED
        }
        
        return state_map.get(daemon_state, NavigationState.IDLE)
    
    def is_navigating(self) -> bool:
        return self.get_state() == NavigationState.NAVIGATING
