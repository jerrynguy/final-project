"""
Robot Controller Interface Module
ROS2 interface for robot control with bridge mode fallback support.
"""

import time
import logging
import asyncio
import requests
import threading
from enum import Enum
from typing import Dict, Any, Optional
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import Twist, Vector3
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    
    ROS_AVAILABLE = True
    
except ImportError:
    class Node:
        def __init__(self, name):
            pass
    
    class QoSProfile:
        pass
    
    class ReliabilityPolicy:
        pass
    
    class HistoryPolicy:
        pass
    
    class Twist:
        pass
    
    class Vector3:
        pass
    
    class Odometry:
        pass
    
    class String:
        pass
    
    class Bool:
        pass
    
    class LaserScan:
        pass
    
    class OccupancyGrid:
        pass
    
    ROS_AVAILABLE = False

try:
    from multi_function_agent.robot_vision_controller.utils.movement_commands import convert_navigation_to_twist
    from multi_function_agent.robot_vision_controller.utils.safety_checks import validate_robot_command_safety
    from multi_function_agent.robot_vision_controller.utils.ros_interface import ROSManager
    
except ImportError:
    def convert_navigation_to_twist(action, parameters, config):
        return None
    
    def validate_robot_command_safety(twist, config):
        return True
    
    class ROSManager:
        def __init__(self, node):
            pass
        def start_spinning(self):
            pass

logger = logging.getLogger(__name__)

# Thêm vào đầu file, sau các import ROS2 hiện có (dòng ~30)

try:
    from multi_function_agent.robot_vision_controller.navigation.nav2_interface import (
        Nav2Interface,
        NavigationState
    )
    NAV2_AVAILABLE = True
except ImportError:
    Nav2Interface = None
    NavigationState = None
    NAV2_AVAILABLE = False
    logger.warning("Nav2Interface not available")


# =============================================================================
# Robot State Enumerations and Data Structures
# =============================================================================

class RobotState(Enum):
    """Robot operational states."""
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"
    DISCONNECTED = "disconnected"


@dataclass
class RobotStatus:
    """
    Robot status information.
    """
    state: RobotState = RobotState.DISCONNECTED
    position_x: float = 0.0
    position_y: float = 0.0
    is_ready: bool = False
    last_update: float = 0.0
    error_message: str = ""


# =============================================================================
# Robot Controller Interface
# =============================================================================

class RobotControllerInterface(Node):
    """
    ROS2 interface for robot control with HTTP bridge fallback.
    """
    
    def __init__(self, config_path: str = "config.yaml"):
        """
        Initialize robot controller interface.
        """
        self.use_bridge = self._should_use_bridge()
        
        if self.use_bridge:
            self.bridge_url = "http://localhost:8080"
            self._init_bridge_mode()
            return
        
        super().__init__("robot_vision_controller")
        
        self.config = self._load_config(config_path)

        self.use_nav2 = self.config.get('use_nav2', True)  # Enable Nav2 by default
        self.nav2_interface = None
        
        if self.use_nav2 and NAV2_AVAILABLE:
            try:
                self.nav2_interface = Nav2Interface()
                logger.info("Nav2Interface initialized")
            except Exception as e:
                logger.error(f"Failed to initialize Nav2: {e}")
                self.use_nav2 = False
        else:
            logger.warning("Nav2 disabled or not available")
        
        self.robot_status = RobotStatus()
        self.is_connected = False
        
        self._last_command_time = 0
        
        self.lidar_data = None
        self.use_lidar = self.config.get('use_lidar', True)
        
        self.map_data = None
        self.use_slam_map = self.config.get('use_slam_map', True)
        
        self._setup_ros_interface()
        
        self.ros_manager = ROSManager(self)
        self.ros_manager.start_spinning()
        
        logger.info("Simplified robot controller initialized")
    
    def _should_use_bridge(self) -> bool:
        """
        Determine if bridge mode should be used.
        """
        return not ROS_AVAILABLE
    
    def _init_bridge_mode(self):
        """
        Initialize HTTP bridge mode.
        """
        self.robot_status = RobotStatus()
        self.is_connected = self._check_bridge_connection()
        
        # Initialize attributes
        self.lidar_data = None
        self.map_data = None

        self.use_nav2 = False
        self.nav2_interface = None
        
        # Start LiDAR polling thread
        if self.is_connected:
            self._start_lidar_polling()
        
        logger.info("Using ROS bridge mode with lidar polling")
    
    def _start_lidar_polling(self):
        """
        Start background thread for polling LiDAR data from bridge.
        """
        def poll_loop():
            while True:
                try:
                    response = requests.get(
                        f"{self.bridge_url}/robot/lidar",
                        timeout=0.5
                    )
                    
                    if response.status_code == 200:
                        result = response.json()
                        data = result.get('data')
                        
                        if data:
                            self.lidar_data = self._convert_to_laserscan(data)
                            logger.debug(f"Lidar updated: {len(self.lidar_data.ranges)} rays")
                    else:
                        logger.debug("Waiting for lidar data from bridge...")
                        
                except Exception as e:
                    logger.debug(f"Lidar polling error: {e}")
                
                time.sleep(0.1)  # 10Hz polling rate
        
        thread = threading.Thread(target=poll_loop, daemon=True)
        thread.start()
        logger.info("Lidar polling thread started")
    
    def _convert_to_laserscan(self, data: dict):
        """
        Convert dictionary to LaserScan-like object.
        """
        class MockLaserScan:
            def __init__(self, data):
                self.ranges = data['ranges']
                self.angle_min = data['angle_min']
                self.angle_max = data['angle_max']
                self.angle_increment = data['angle_increment']
                self.range_min = data['range_min']
                self.range_max = data['range_max']
        
        return MockLaserScan(data)
    
    def _check_bridge_connection(self) -> bool:
        """
        Check if bridge server is accessible.
        """
        try:
            response = requests.get(
                f"{self.bridge_url}/robot/status",
                timeout=2
            )
            return response.status_code == 200
        except:
            return False
    
    async def _execute_via_bridge(self, navigation_decision: Dict[str, Any]) -> bool:
        """
        Execute command via HTTP bridge.
        """
        try:
            action = navigation_decision.get('action', 'stop')
            parameters = navigation_decision.get('parameters', {})
            
            payload = {
                "action": action,
                "parameters": parameters
            }
            
            response = requests.post(
                f"{self.bridge_url}/robot/command",
                json=payload,
                timeout=5
            )
            
            result = response.json()
            return result.get("status") == "success"
            
        except Exception as e:
            logger.error(f"Bridge command failed: {e}")
            return False
        
    def _load_config(self, config_path: str) -> dict:
        """Load configuration from YAML file."""
        import yaml
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
                func_config = config.get('functions', {}).get('robot_vision_controller', {})
                
                return {
                    # Robot controller
                    'min_command_interval': func_config.get('min_command_interval', 0.05),
                    'connection_timeout': func_config.get('connection_timeout', 3.0),
                    'max_linear_velocity': func_config.get('max_linear_velocity', 0.22),
                    'max_angular_velocity': func_config.get('max_angular_velocity', 2.84),
                    
                    # LIDAR/SLAM
                    'use_lidar': func_config.get('use_lidar', True),
                    'lidar_topic': func_config.get('lidar_topic', '/scan'),
                    'max_lidar_range': func_config.get('max_lidar_range', 3.5),
                    'use_slam_map': func_config.get('use_slam_map', True),
                    'map_topic': func_config.get('map_topic', '/map'),
                    
                    # Nav2 - THÊM MỚI
                    'use_nav2': func_config.get('use_nav2', True),
                    'nav2_timeout': func_config.get('nav2_timeout', 60.0),
                    'nav2_goal_tolerance': func_config.get('nav2_goal_tolerance', 0.2),
                    'nav2_angle_tolerance': func_config.get('nav2_angle_tolerance', 0.1),
                    'nav2_fallback_to_manual': func_config.get('nav2_fallback_to_manual', True),
                }
                # =============================================================================
                
        except FileNotFoundError:
            logger.warning(f"Config file not found: {config_path}, using defaults")
            return {
                'min_command_interval': 0.05,
                'connection_timeout': 3.0,
                'max_linear_velocity': 0.22,
                'max_angular_velocity': 2.84,
                'use_lidar': True,
                'lidar_topic': '/scan',
                'max_lidar_range': 3.5,
                'use_slam_map': True,
                'map_topic': '/map',
                'use_nav2': True,
                'nav2_timeout': 60.0,
                'nav2_goal_tolerance': 0.2,
                'nav2_angle_tolerance': 0.1,
                'nav2_fallback_to_manual': True,
            }
    
    def _setup_ros_interface(self):
        """
        Setup ROS2 publishers and subscribers.
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.status_pub = self.create_publisher(String, "/robot_vision_status", qos)
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odometry_callback,
            qos
        )
        
        # LiDAR subscriber
        if self.use_lidar:
            lidar_topic = self.config.get('lidar_topic', '/scan')
            self.scan_sub = self.create_subscription(
                LaserScan,
                lidar_topic,
                self._scan_callback,
                qos
            )
            logger.info(f"Lidar subscriber created on {lidar_topic}")
        
        # SLAM map subscriber
        if self.use_slam_map:
            map_topic = self.config.get('map_topic', '/map')
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                map_topic,
                self._map_callback,
                qos
            )
            logger.info(f"Map subscriber created on {map_topic}")
    
    async def connect(self) -> bool:
        """
        Connect to robot (via bridge or ROS).
        """
        if self.use_bridge:
            try:
                logger.info("Connecting to robot via bridge...")
                self.is_connected = self._check_bridge_connection()
                
                if self.is_connected:
                    self.robot_status.state = RobotState.IDLE
                    self.robot_status.is_ready = True
                    self._publish_status("Robot connected via bridge")
                    logger.info("Robot connected via bridge")
                    return True
                else:
                    logger.error("Bridge connection failed")
                    self.robot_status.state = RobotState.ERROR
                    return False
                    
            except Exception as e:
                logger.error(f"Bridge connection error: {e}")
                self.is_connected = False
                self.robot_status.state = RobotState.ERROR
                self.robot_status.error_message = str(e)
                return False
        
        try:
            logger.info("Connecting to robot...")
            
            # Test connection by publishing empty twist
            test_twist = Twist()
            self.cmd_vel_pub.publish(test_twist)
            
            await asyncio.sleep(0.5)
            
            self.is_connected = True
            self.robot_status.state = RobotState.IDLE
            self.robot_status.is_ready = True
            
            self._publish_status("Robot connected successfully")
            logger.info("Robot connection established")
            return True
            
        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            self.is_connected = False
            self.robot_status.state = RobotState.ERROR
            self.robot_status.error_message = str(e)
            return False
        
    async def wait_for_nav2_ready(self, timeout: float = 10.0) -> bool:
        """
        Wait for Nav2 to be ready.
        """
        if not self.use_nav2 or not self.nav2_interface:
            logger.warning("Nav2 not enabled")
            return False
        
        logger.info("Waiting for Nav2 to be ready...")
        
        # Wait for Nav2 action server
        ready = self.nav2_interface.wait_for_nav2(timeout=timeout)
        
        if ready:
            logger.info("Nav2 is ready")
            return True
        else:
            logger.error("Nav2 timeout - falling back to manual control")
            self.use_nav2 = False
            return False
        
    async def send_nav2_goal(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        blocking: bool = False
    ) -> bool:
        """
        Send navigation goal to Nav2.
        """
        if not self.use_nav2 or not self.nav2_interface:
            logger.error("Nav2 not available, cannot send goal")
            return False
        
        try:
            # Send goal to Nav2
            success = self.nav2_interface.send_goal(x, y, theta)
            
            if not success:
                logger.error(f"Nav2 rejected goal: ({x:.2f}, {y:.2f})")
                return False
            
            logger.info(f"Nav2 goal sent: ({x:.2f}, {y:.2f}, {theta:.2f}rad)")
            
            # If blocking, wait for completion
            if blocking:
                return await self._wait_for_nav2_completion()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send Nav2 goal: {e}")
            return False

    async def _wait_for_nav2_completion(self, timeout: float = 60.0) -> bool:
        """
        Wait for Nav2 navigation to complete.
        """
        import asyncio
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Spin ROS2 to process callbacks
            import rclpy
            rclpy.spin_once(self.nav2_interface, timeout_sec=0.1)
            
            state = self.nav2_interface.get_state()
            
            if state == NavigationState.SUCCEEDED:
                logger.info("✅ Nav2 navigation succeeded")
                return True
            
            elif state in [NavigationState.FAILED, NavigationState.CANCELLED]:
                logger.warning(f"❌ Nav2 navigation {state.value}")
                return False
            
            # Check LIDAR safety during navigation
            if self.lidar_data is not None:
                from multi_function_agent.robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
                safety_monitor = LidarSafetyMonitor()
                
                min_dist = safety_monitor.get_min_distance(self.lidar_data)
                
                # Emergency abort if critical
                if min_dist < safety_monitor.CRITICAL_DISTANCE:
                    logger.error(f"[SAFETY ABORT] Obstacle at {min_dist:.2f}m during Nav2")
                    self.cancel_nav2_navigation()
                    return False
            
            await asyncio.sleep(0.1)
        
        logger.error("Nav2 navigation timeout")
        self.cancel_nav2_navigation()
        return False
    
    def cancel_nav2_navigation(self) -> bool:
        """
        Cancel current Nav2 navigation.
        """
        if not self.use_nav2 or not self.nav2_interface:
            return False
        
        logger.warning("🛑 Cancelling Nav2 navigation")
        return self.nav2_interface.cancel_navigation()
    
    def get_nav2_state(self) -> Optional[str]:
        """
        Get current Nav2 navigation state.
        """
        if not self.use_nav2 or not self.nav2_interface:
            return None
        
        state = self.nav2_interface.get_state()
        return state.value if state else None
    
    async def execute_command(self, navigation_decision: Dict[str, Any]) -> bool:
        """
        Execute navigation command on robot.
        """
        if self.use_bridge:
            return await self._execute_via_bridge(navigation_decision)
        
        if not self.is_connected or not self.robot_status.is_ready:
            logger.error("Cannot execute command: robot not ready")
            return False
        
        current_time = time.time()
        
        try:
            action = navigation_decision.get('action', 'stop')
            parameters = navigation_decision.get('parameters', {})
            
            logger.info(f"execute_command called: action={action}")
            
            # Convert to ROS Twist message
            twist_msg = convert_navigation_to_twist(action, parameters, self.config)
            logger.info(
                f"Twist created: linear={twist_msg.linear.x}, "
                f"angular={twist_msg.angular.z}"
            )
            
            # Validate safety
            if not validate_robot_command_safety(twist_msg, self.config):
                logger.warning("Command failed safety validation")
                return await self._emergency_stop()
            
            # Send command
            success = await self._send_command(
                twist_msg,
                parameters.get('duration', 1.0)
            )
            logger.info(f"Send result: {success}")
            
            if success:
                self._last_command_time = current_time
                self._update_robot_state(action)
                self._publish_status(f"Executed: {action}")
            
            return success
            
        except Exception as e:
            logger.error(f"Command execution failed: {e}")
            self.robot_status.state = RobotState.ERROR
            return False
    
    async def _send_command(self, twist: Twist, duration: float) -> bool:
        """
        Send twist command with AGGRESSIVE safety monitoring.
        
        CRITICAL: Aborts immediately if obstacle detected during execution.
        """
        try:
            logger.info(
                f"Publishing cmd_vel: linear={twist.linear.x:.3f}, "
                f"angular={twist.angular.z:.3f}, duration={duration:.2f}s"
            )
            
            publish_rate = 20  # Increased from 10Hz to 20Hz for faster abort
            interval = 1.0 / publish_rate
            iterations = int(duration / interval)
            
            # Import safety monitor
            from multi_function_agent.robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
            safety_monitor = LidarSafetyMonitor()
            
            # Execute with continuous safety check
            for i in range(max(1, iterations)):
                # ============================================================
                # CRITICAL: Check safety BEFORE each publish
                # ============================================================
                if self.lidar_data is not None:
                    min_dist = safety_monitor.get_min_distance(self.lidar_data)
                    
                    # IMMEDIATE ABORT if critical distance breached
                    if min_dist < safety_monitor.CRITICAL_DISTANCE:
                        logger.error(
                            f"[EMERGENCY ABORT] Obstacle at {min_dist:.2f}m! "
                            f"Stopping immediately (iteration {i+1}/{iterations})"
                        )
                        # Send immediate stop
                        stop_twist = Twist()
                        for _ in range(5):  # Spam stop commands
                            self.cmd_vel_pub.publish(stop_twist)
                            await asyncio.sleep(0.01)
                        return False  # Abort execution
                    
                    # WARNING: Reduce speed significantly if approaching critical
                    elif min_dist < safety_monitor.WARNING_DISTANCE:
                        scale = (min_dist - safety_monitor.CRITICAL_DISTANCE) / \
                                (safety_monitor.WARNING_DISTANCE - safety_monitor.CRITICAL_DISTANCE)
                        scale = max(0.2, min(1.0, scale))  # Clamp [0.2, 1.0]
                        
                        logger.warning(
                            f"[SPEED REDUCTION] Obstacle at {min_dist:.2f}m, "
                            f"scaling to {scale*100:.0f}%"
                        )
                        
                        scaled_twist = Twist()
                        scaled_twist.linear.x = twist.linear.x * scale * 0.5
                        scaled_twist.angular.z = twist.angular.z * scale
                        self.cmd_vel_pub.publish(scaled_twist)
                    else:
                        # Safe: publish original command
                        self.cmd_vel_pub.publish(twist)
                else:
                    # No LIDAR data: publish with caution
                    self.cmd_vel_pub.publish(twist)
                
                await asyncio.sleep(interval)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            # Emergency stop on error
            try:
                stop_twist = Twist()
                for _ in range(5):
                    self.cmd_vel_pub.publish(stop_twist)
                    await asyncio.sleep(0.01)
            except:
                pass
            return False
    
    async def _emergency_stop(self) -> bool:
        """
        Execute emergency stop.
        """
        try:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            
            self.robot_status.state = RobotState.IDLE
            self._publish_status("EMERGENCY STOP")
            
            logger.warning("Emergency stop executed")
            return True
            
        except Exception as e:
            logger.error(f"Emergency stop failed: {e}")
            return False
    
    def _update_robot_state(self, action: str):
        """
        Update robot state based on action.
        """
        if action == 'stop':
            self.robot_status.state = RobotState.IDLE
        else:
            self.robot_status.state = RobotState.MOVING
        
        self.robot_status.last_update = time.time()
    
    def _odometry_callback(self, msg: Odometry):
        """
        Handle odometry message updates.
        """
        try:
            self.robot_status.position_x = msg.pose.pose.position.x
            self.robot_status.position_y = msg.pose.pose.position.y
            
            self.robot_status.last_update = time.time()
            
            # Check if connection is still alive
            self.robot_status.is_ready = (
                time.time() - self.robot_status.last_update
                < self.config['connection_timeout']
            )
            
        except Exception as e:
            logger.error(f"Odometry callback error: {e}")
    
    def _scan_callback(self, msg: LaserScan):
        """
        Handle LiDAR scan message updates.
        """
        try:
            self.lidar_data = msg
        except Exception as e:
            logger.error(f"Scan callback error: {e}")
    
    def _map_callback(self, msg: OccupancyGrid):
        """
        Handle SLAM map message updates.
        """
        try:
            self.map_data = msg
        except Exception as e:
            logger.error(f"Map callback error: {e}")
    
    def _publish_status(self, message: str):
        """
        Publish status message.
        """
        try:
            status_msg = String()
            status_msg.data = f"{time.time():.3f}: {message}"
            self.status_pub.publish(status_msg)
        except Exception as e:
            logger.debug(f"Status publish error: {e}")