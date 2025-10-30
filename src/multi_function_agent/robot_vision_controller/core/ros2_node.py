"""
ROS2 Node Module
Centralized ROS2 node for NAT Agent container.
Replaces HTTP bridge with native DDS communication.
"""

import logging
import threading
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

logger = logging.getLogger(__name__)


class NATAgentNode(Node):
    """
    Centralized ROS2 node for NAT Agent.
    Manages all publishers, subscribers, and provides data access.
    """
    
    def __init__(self):
        """Initialize NAT Agent ROS2 node."""
        super().__init__('nat_agent_node')
        
        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.qos_reliable
        )
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            self.qos_best_effort  # LIDAR typically uses best_effort
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            self.qos_reliable
        )
        
        # =====================================================================
        # Data Storage (thread-safe access)
        # =====================================================================
        self._data_lock = threading.Lock()
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        
        # =====================================================================
        # Callback hooks (optional)
        # =====================================================================
        self.scan_callback_hook: Optional[Callable] = None
        self.odom_callback_hook: Optional[Callable] = None
        
        logger.info("✅ NATAgentNode initialized")
        logger.info("   Publishers: /cmd_vel")
        logger.info("   Subscribers: /scan, /odom")
    
    # =========================================================================
    # Subscriber Callbacks
    # =========================================================================
    
    def _scan_callback(self, msg: LaserScan):
        """Store latest LIDAR scan."""
        with self._data_lock:
            self.latest_scan = msg
        
        # Call external hook if registered
        if self.scan_callback_hook:
            try:
                self.scan_callback_hook(msg)
            except Exception as e:
                logger.error(f"Scan callback hook error: {e}")
    
    def _odom_callback(self, msg: Odometry):
        """Store latest odometry."""
        with self._data_lock:
            self.latest_odom = msg
        
        # Call external hook if registered
        if self.odom_callback_hook:
            try:
                self.odom_callback_hook(msg)
            except Exception as e:
                logger.error(f"Odom callback hook error: {e}")
    
    # =========================================================================
    # Data Access Methods (thread-safe)
    # =========================================================================
    
    def get_scan(self) -> Optional[LaserScan]:
        """Get latest LIDAR scan (thread-safe)."""
        with self._data_lock:
            return self.latest_scan
    
    def get_odom(self) -> Optional[Odometry]:
        """Get latest odometry (thread-safe)."""
        with self._data_lock:
            return self.latest_odom
    
    def get_robot_pose(self) -> Optional[dict]:
        """
        Get robot pose from odometry.
        Returns: {'x': float, 'y': float, 'theta': float} or None
        """
        odom = self.get_odom()
        if odom is None:
            return None
        
        try:
            from tf_transformations import euler_from_quaternion
            
            q = odom.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            return {
                'x': odom.pose.pose.position.x,
                'y': odom.pose.pose.position.y,
                'theta': yaw
            }
        except Exception as e:
            logger.error(f"Failed to extract pose: {e}")
            return None
    
    # =========================================================================
    # Command Publishing
    # =========================================================================
    
    def publish_velocity(self, linear: float, angular: float):
        """
        Publish velocity command.
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
        """
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        
        self.cmd_vel_pub.publish(msg)
    
    def publish_stop(self):
        """Publish stop command."""
        self.publish_velocity(0.0, 0.0)


# =============================================================================
# ROS2 Manager (Singleton Pattern)
# =============================================================================

class ROS2Manager:
    """
    Singleton manager for ROS2 node lifecycle.
    Ensures only one node instance and one executor.
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self.node: Optional[NATAgentNode] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self._initialized = True
        
        logger.info("ROS2Manager created")
    
    def initialize(self) -> NATAgentNode:
        """
        Initialize ROS2 context and create node.
        """
        if self.node is not None:
            logger.warning("ROS2 already initialized")
            return self.node
        
        # Initialize rclpy if not already
        if not rclpy.ok():
            logger.info("Initializing rclpy...")
            rclpy.init()
        
        # Create node
        logger.info("Creating NATAgentNode...")
        self.node = NATAgentNode()
        
        # Create executor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spinning in background thread
        self.spin_thread = threading.Thread(
            target=self._spin_loop,
            daemon=True,
            name="ROS2SpinThread"
        )
        self.spin_thread.start()
        
        logger.info("✅ ROS2 node spinning in background")
        
        return self.node
    
    def _spin_loop(self):
        """Background thread for spinning executor."""
        try:
            logger.info("ROS2 spin thread started")
            self.executor.spin()
        except Exception as e:
            logger.error(f"ROS2 spin error: {e}")
    
    def get_node(self) -> Optional[NATAgentNode]:
        """Get existing node instance."""
        return self.node
    
    def shutdown(self):
        """Shutdown ROS2 node and executor."""
        if self.executor:
            logger.info("Shutting down ROS2 executor...")
            self.executor.shutdown()
        
        if self.node:
            logger.info("Destroying ROS2 node...")
            self.node.destroy_node()
        
        if rclpy.ok():
            logger.info("Shutting down rclpy...")
            rclpy.shutdown()
        
        self.node = None
        self.executor = None
        self.spin_thread = None
        
        logger.info("✅ ROS2 shutdown complete")


# =============================================================================
# Convenience Functions
# =============================================================================

def get_ros2_node() -> NATAgentNode:
    """
    Get or create ROS2 node instance.
    """
    manager = ROS2Manager()
    node = manager.get_node()
    
    if node is None:
        node = manager.initialize()
    
    return node


def shutdown_ros2():
    """Shutdown ROS2 manager."""
    manager = ROS2Manager()
    manager.shutdown()