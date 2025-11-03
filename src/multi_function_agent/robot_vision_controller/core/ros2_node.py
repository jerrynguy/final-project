"""
ROS2 Node Module - Subprocess Wrapper
Uses system Python 3.10 with rclpy via subprocess.
Works with Python 3.12/3.11 venv without building from source.
"""

import subprocess
import json
import logging
import threading
import time
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class ROS2Bridge:
    """
    Bridge between Python 3.11+ (NAT) and Python 3.10 (ROS2).
    Uses subprocess to call system Python with rclpy.
    """
    
    def __init__(self):
        self.system_python = '/usr/bin/python3'  # System Python 3.10
        self._lidar_cache = None
        self._odom_cache = None
        self._monitor_thread = None
        self._running = False
        self._lock = threading.Lock()
        
        # Verify ROS2 available
        self._verify_ros2()
        
        # Start background monitor for sensor data
        self._start_monitor()
        
        logger.info("✅ ROS2Bridge initialized (subprocess mode)")
    
    def get_name(self):
        """Get node name (compatibility)."""
        return "ros2_bridge_subprocess"
    
    def _verify_ros2(self):
        """Verify system Python has rclpy."""
        try:
            result = subprocess.run(
                [self.system_python, '-c', 'import rclpy; print("OK")'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode != 0:
                raise RuntimeError(f"rclpy not available: {result.stderr}")
            logger.info("✅ System Python has rclpy")
        except Exception as e:
            logger.error(f"❌ ROS2 verification failed: {e}")
            raise
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command via subprocess."""
        script = f"""
import rclpy
from geometry_msgs.msg import Twist

try:
    rclpy.init()
    node = rclpy.create_node('cmd_vel_publisher')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # Wait for publisher to be ready
    import time
    time.sleep(0.01)
    
    msg = Twist()
    msg.linear.x = {linear}
    msg.angular.z = {angular}
    
    pub.publish(msg)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
except Exception as e:
    import sys
    print(f"ERROR: {{e}}", file=sys.stderr)
    sys.exit(1)
"""
        try:
            result = subprocess.run(
                [self.system_python, '-c', script],
                capture_output=True,
                text=True,
                timeout=0.5
            )
            if result.returncode != 0:
                logger.error(f"Publish failed: {result.stderr}")
                return
        except subprocess.TimeoutExpired:
            logger.warning("Publish timeout (non-critical)")
        except Exception as e:
            logger.error(f"Publish error: {e}")
    
    def publish_stop(self):
        """Publish stop command."""
        self.publish_velocity(0.0, 0.0)
    
    def get_scan(self) -> Optional[Any]:
        """Get cached LIDAR data (returns dict-like object)."""
        with self._lock:
            return self._lidar_cache
    
    def get_odom(self) -> Optional[Any]:
        """Get cached odometry data (returns dict-like object)."""
        with self._lock:
            return self._odom_cache
    
    def get_robot_pose(self) -> Optional[Dict]:
        """Get robot pose from cached odometry."""
        with self._lock:
            if not self._odom_cache:
                return None
            
            try:
                return {
                    'x': self._odom_cache['position']['x'],
                    'y': self._odom_cache['position']['y'],
                    'theta': self._odom_cache['orientation']['yaw']
                }
            except (KeyError, TypeError):
                return None
    
    def _start_monitor(self):
        """Start background thread to monitor sensor data."""
        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="ROS2Monitor"
        )
        self._monitor_thread.start()
        logger.info("✅ Sensor monitor started")
    
    def _monitor_loop(self):
        """Background loop to fetch sensor data."""
        while self._running:
            try:
                # Fetch LIDAR data
                self._fetch_lidar()
                
                # Fetch odometry data
                self._fetch_odom()
                
                time.sleep(0.1)  # 10Hz update
                
            except Exception as e:
                logger.debug(f"Monitor error: {e}")
                time.sleep(0.5)
    
    def _fetch_lidar(self):
        """Fetch LIDAR data via subprocess."""
        script = """
import rclpy
from sensor_msgs.msg import LaserScan
import json
import sys

data_received = False

def callback(msg):
    global data_received
    data = {
        'ranges': list(msg.ranges),
        'angle_min': msg.angle_min,
        'angle_max': msg.angle_max,
        'angle_increment': msg.angle_increment,
        'range_min': msg.range_min,
        'range_max': msg.range_max
    }
    print(json.dumps(data))
    data_received = True

try:
    rclpy.init()
    node = rclpy.create_node('lidar_monitor')
    sub = node.create_subscription(LaserScan, '/scan', callback, 10)
    
    # Wait for one message
    start = rclpy.clock.Clock().now()
    while not data_received and (rclpy.clock.Clock().now() - start).nanoseconds < 1e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()
except:
    pass
"""
        try:
            result = subprocess.run(
                [self.system_python, '-c', script],
                capture_output=True,
                text=True,
                timeout=1.5
            )
            if result.returncode == 0 and result.stdout.strip():
                data = json.loads(result.stdout)
                # Convert to object-like dict
                class LaserScanData:
                    def __init__(self, d):
                        self.ranges = d['ranges']
                        self.angle_min = d['angle_min']
                        self.angle_max = d['angle_max']
                        self.angle_increment = d['angle_increment']
                        self.range_min = d['range_min']
                        self.range_max = d['range_max']
                
                with self._lock:
                    self._lidar_cache = LaserScanData(data)
        except:
            pass
    
    def _fetch_odom(self):
        """Fetch odometry data via subprocess."""
        script = """
import rclpy
from nav_msgs.msg import Odometry
import json
import sys
import math

data_received = False

def callback(msg):
    global data_received
    q = msg.pose.pose.orientation
    
    # Convert quaternion to yaw
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    data = {
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'yaw': yaw
        },
        'velocity': {
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z
        }
    }
    print(json.dumps(data))
    data_received = True

try:
    rclpy.init()
    node = rclpy.create_node('odom_monitor')
    sub = node.create_subscription(Odometry, '/odom', callback, 10)
    
    # Wait for one message
    start = rclpy.clock.Clock().now()
    while not data_received and (rclpy.clock.Clock().now() - start).nanoseconds < 1e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()
except:
    pass
"""
        try:
            result = subprocess.run(
                [self.system_python, '-c', script],
                capture_output=True,
                text=True,
                timeout=1.5
            )
            if result.returncode == 0 and result.stdout.strip():
                data = json.loads(result.stdout)
                with self._lock:
                    self._odom_cache = data
        except:
            pass
    
    def shutdown(self):
        """Shutdown bridge."""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        logger.info("✅ ROS2Bridge shutdown")


# =============================================================================
# Singleton Management
# =============================================================================

_bridge_instance = None
_bridge_lock = threading.Lock()

def get_ros2_node():
    """Get or create ROS2 bridge instance."""
    global _bridge_instance
    
    with _bridge_lock:
        if _bridge_instance is None:
            _bridge_instance = ROS2Bridge()
    
    return _bridge_instance

def shutdown_ros2():
    """Shutdown ROS2 bridge."""
    global _bridge_instance
    
    with _bridge_lock:
        if _bridge_instance:
            _bridge_instance.shutdown()
            _bridge_instance = None