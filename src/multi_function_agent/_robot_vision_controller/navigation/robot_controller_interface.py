"""
Robot Controller Interface Module
ROS2 interface for robot control with bridge mode fallback support.
"""

import time
import logging
import asyncio
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
    from tf_transformations import euler_from_quaternion
    
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
    from multi_function_agent._robot_vision_controller.utils.safety_checks import validate_robot_command_safety
    from multi_function_agent._robot_vision_controller.utils.ros_interface import ROSManager
    
except ImportError:    
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
    from multi_function_agent._robot_vision_controller.navigation.nav2_interface import (
        Nav2Interface,
        NavigationState
    )
    from multi_function_agent._robot_vision_controller.core.ros2_node import get_ros2_node
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
    theta: float = 0.0
    error_message: str = ""


# =============================================================================
# Robot Controller Interface
# =============================================================================

class RobotControllerInterface(Node):
    """
    ROS2 interface for robot control with HTTP bridge fallback.
    """
    
    def __init__(self, config_path: str = None):
        # Get centralized ROS2 node (singleton)
        self.ros_node = get_ros2_node()

        if config_path is None:
            import os
            from pathlib import Path

            possible_paths = [
                "/workspace/mounted_code/src/multi_function_agent/configs/config.yml",  # Container
                Path(__file__).parent.parent.parent / "configs/config.yml",  # Relative
                Path.home() / "nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/configs/config.yml",  # Host
                "config.yaml",  # Current dir fallback
            ]

            for path in possible_paths:
                if isinstance(path, Path):
                    path = str(path)
                if os.path.exists(path):
                    config_path = path
                    logger.info(f"Using config file: {config_path}")
                    break
            
            if config_path is None or not os.path.exists(config_path):
                logger.warning("No config file found, using defaults")
                config_path = "config.yaml"  # Will use defaults in loader
        
        self.config = self._load_config(config_path)
        
        self.use_nav2 = self.config.get('use_nav2', True)
        self.nav2_interface = None
        
        if self.use_nav2 and NAV2_AVAILABLE:
            try:
                self.nav2_interface = Nav2Interface()
                logger.info("Nav2Interface initialized")
            except Exception as e:
                logger.error(f"Failed to initialize Nav2: {e}")
                self.use_nav2 = False
        
        self._robot_status = RobotStatus()
        self.is_connected = False
        self._last_command_time = 0
        
        logger.info("Robot controller initialized with native ROS2")

    @property
    def lidar_data(self):
        """Get latest LIDAR data from ROS2 node."""
        return self.ros_node.get_scan()

    @property  
    def robot_status(self):
        """Get robot status from odometry."""
        if not hasattr(self, '_robot_status'):
            self._robot_status = RobotStatus()
        
        # Update position ONLY (preserve is_ready from connect())
        pose = self.ros_node.get_robot_pose()
        if pose:
            self._robot_status.position_x = pose['x']
            self._robot_status.position_y = pose['y']
            self._robot_status.theta = pose['theta']
        
        return self._robot_status

    @robot_status.setter
    def robot_status(self, value):
        """Set robot status."""
        self._robot_status = value
        
    def _load_config(self, config_path: str) -> dict:
        """Load configuration from YAML file."""
        import yaml
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
                func_config = config.get('functions', {}).get('_robot_vision_controller', {})
                
                return {
                    # Robot controller
                    'min_command_interval': func_config.get('min_command_interval', 0.05),
                    'connection_timeout': func_config.get('connection_timeout', 3.0),
                    'max_linear_velocity': func_config.get('max_linear_velocity', 0.22),
                    'max_angular_velocity': func_config.get('max_angular_velocity', 2.84),
                    'exploration_speed_boost': func_config.get('exploration_speed_boost', 1.0),
                    
                    # LIDAR/SLAM
                    'use_lidar': func_config.get('use_lidar', True),
                    'lidar_topic': func_config.get('lidar_topic', '/scan'),
                    'max_lidar_range': func_config.get('max_lidar_range', 3.5),
                    'use_slam_map': func_config.get('use_slam_map', True),
                    'map_topic': func_config.get('map_topic', '/map'),
                    
                    # Nav2 
                    'use_nav2': func_config.get('use_nav2', True),
                    'nav2_timeout': func_config.get('nav2_timeout', 60.0),
                    'nav2_goal_tolerance': func_config.get('nav2_goal_tolerance', 0.2),
                    'nav2_angle_tolerance': func_config.get('nav2_angle_tolerance', 0.1),
                    'nav2_fallback_to_manual': func_config.get('nav2_fallback_to_manual', True),
                }
                # =============================================================================
                
        except FileNotFoundError:
            logger.info(f"Config file not found at {config_path}, using built-in defaults")
            return {
                'min_command_interval': 0.05,
                'connection_timeout': 3.0,
                'max_linear_velocity': 0.22,
                'max_angular_velocity': 2.84,
                'exploration_speed_boost': 1.0,
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
        
    async def connect(self) -> bool:
        """Connect to robot via ROS2."""
        try:
            logger.info("Checking ROS2 connection...")
            
            # Test by publishing empty twist
            self.ros_node.publish_stop()
            
            await asyncio.sleep(0.5)
            
            # Check if receiving data
            if self.ros_node.get_scan() is None:
                logger.warning("No LIDAR data yet")
            
            if self.ros_node.get_odom() is None:
                logger.warning("No odometry data yet")
            
            self.is_connected = True
            self.robot_status.state = RobotState.IDLE
            self.robot_status.is_ready = True
            
            logger.info("✅ Robot connected via ROS2 DDS")
            return True
            
        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            self.is_connected = False
            self.robot_status.state = RobotState.ERROR
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
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send Nav2 goal: {e}")
            return False
        
    def check_nav2_safety(self, lidar_override = None) -> bool:
        """Check Nav2 safety NON-BLOCKING."""
        if not self.use_nav2 or not self.nav2_interface:
            return True
        
        if not self.nav2_interface.is_navigating():
            return True
        
        # Use override if provided (for atomic reads)
        lidar_to_check = lidar_override if lidar_override is not None else self.lidar_data
        if lidar_to_check is None:
            return True
        
        from multi_function_agent._robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
        safety_monitor = LidarSafetyMonitor()
        
        min_dist = safety_monitor.get_min_distance(lidar_to_check)
        
        if min_dist < safety_monitor.CRITICAL_DISTANCE:
            logger.error(f"[ABORT NAV2] Obstacle at {min_dist:.2f}m")
            self.cancel_nav2_navigation()
            return False
        
        return True
    
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
        """Execute navigation command on robot."""
        
        if not self.is_connected or not self.robot_status.is_ready:
            logger.error("Cannot execute command: robot not ready")
            return False
        
        current_time = time.time()
        
        try:
            action = navigation_decision.get('action', 'stop')
            parameters = navigation_decision.get('parameters', {})
            
            # Create Twist directly
            class TwistMsg:
                def __init__(self):
                    class Vec3:
                        def __init__(self):
                            self.x = 0.0
                            self.y = 0.0
                            self.z = 0.0
                    
                    self.linear = Vec3()
                    self.angular = Vec3()
            
            twist_msg = TwistMsg()
            twist_msg.linear.x = parameters.get('linear_velocity', 0.0)
            twist_msg.angular.z = parameters.get('angular_velocity', 0.0)
            
            # Clamp to safety limits
            max_linear = self.config.get('max_linear_velocity', 0.22)
            max_angular = self.config.get('max_angular_velocity', 2.84)
            
            twist_msg.linear.x = max(-max_linear, min(max_linear, twist_msg.linear.x))
            twist_msg.angular.z = max(-max_angular, min(max_angular, twist_msg.angular.z))
            
            # Validate safety
            if not validate_robot_command_safety(twist_msg, self.config):
                logger.warning("Command failed safety validation")
                return await self._emergency_stop()
            
            # Send command
            success = await self._send_command(
                twist_msg,
                parameters.get('duration', 1.0)
            )
            
            if success:
                self._last_command_time = current_time
                self._update_robot_state(action)
            
            return success
            
        except Exception as e:
            logger.error(f"Command execution failed: {e}")
            self.robot_status.state = RobotState.ERROR
            return False
    
    async def _send_command(self, twist: Twist, duration: float) -> bool:
        try:
            logger.info(
                f"Publishing cmd_vel: linear={twist.linear.x:.3f}, "
                f"angular={twist.angular.z:.3f}, duration={duration:.2f}s"
            )
            
            publish_rate = 20
            interval = 1.0 / publish_rate
            iterations = int(duration / interval)
            
            from multi_function_agent._robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
            safety_monitor = LidarSafetyMonitor()
            
            for i in range(max(1, iterations)):
                if self.lidar_data is not None:
                    min_dist = safety_monitor.get_min_distance(self.lidar_data)
                    
                    if min_dist < safety_monitor.CRITICAL_DISTANCE:
                        logger.error(
                            f"[EMERGENCY ABORT] Obstacle at {min_dist:.2f}m! "
                            f"Stopping immediately (iteration {i+1}/{iterations})"
                        )
                        # ✅ THAY ĐỔI: Dùng ros_node
                        for _ in range(5):
                            self.ros_node.publish_stop()
                            await asyncio.sleep(0.01)
                        return False
                    
                    elif min_dist < safety_monitor.WARNING_DISTANCE:
                        scale = (min_dist - safety_monitor.CRITICAL_DISTANCE) / \
                                (safety_monitor.WARNING_DISTANCE - safety_monitor.CRITICAL_DISTANCE)
                        scale = max(0.2, min(1.0, scale))
                        
                        logger.warning(
                            f"[SPEED REDUCTION] Obstacle at {min_dist:.2f}m, "
                            f"scaling to {scale*100:.0f}%"
                        )
                        
                        # ✅ THAY ĐỔI: Dùng ros_node
                        scaled_linear = twist.linear.x * scale * 0.5
                        scaled_angular = twist.angular.z * scale
                        self.ros_node.publish_velocity(scaled_linear, scaled_angular)
                    else:
                        # ✅ THAY ĐỔI: Dùng ros_node
                        self.ros_node.publish_velocity(twist.linear.x, twist.angular.z)
                else:
                    # ✅ THAY ĐỔI: Dùng ros_node
                    self.ros_node.publish_velocity(twist.linear.x, twist.angular.z)
                
                await asyncio.sleep(interval)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            try:
                # ✅ THAY ĐỔI: Dùng ros_node
                for _ in range(5):
                    self.ros_node.publish_stop()
                    await asyncio.sleep(0.01)
            except:
                pass
            return False
    
    async def _emergency_stop(self) -> bool:
        try:
            self.ros_node.publish_stop()
            
            # Update status using private attribute
            if not hasattr(self, '_robot_status'):
                self._robot_status = RobotStatus()
            self._robot_status.state = RobotState.IDLE
            
            logger.warning("Emergency stop executed")
            return True
            
        except Exception as e:
            logger.error(f"Emergency stop failed: {e}")
            return False
    
    def _update_robot_state(self, action: str):
        if not hasattr(self, '_robot_status'):
            self._robot_status = RobotStatus()
        
        if action == 'stop':
            self._robot_status.state = RobotState.IDLE
        else:
            self._robot_status.state = RobotState.MOVING
        
        self._robot_status.last_update = time.time()
    
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