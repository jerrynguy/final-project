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
        """Start persistent subprocess daemon."""
        self._running = True
        self._daemon_process = subprocess.Popen(
            [self.system_python, '-c', self._get_daemon_script()],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        # Thread to read daemon output
        self._monitor_thread = threading.Thread(
            target=self._read_daemon_output,
            daemon=True
        )
        self._monitor_thread.start()
        logger.info("✅ ROS2 daemon started")

    def _get_daemon_script(self) -> str:
        """Persistent ROS2 node script."""
        return """
    import rclpy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    import json
    import sys
    import math

    class SensorNode:
        def __init__(self):
            rclpy.init()
            self.node = rclpy.create_node('sensor_daemon')
            
            self.lidar_sub = self.node.create_subscription(
                LaserScan, '/scan', self.lidar_callback, 10)
            self.odom_sub = self.node.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)
            
            self.lidar_data = None
            self.odom_data = None
        
        def lidar_callback(self, msg):
            self.lidar_data = {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            }
            print(json.dumps({'type': 'lidar', 'data': self.lidar_data}), flush=True)
        
        def odom_callback(self, msg):
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.odom_data = {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {'yaw': yaw},
                'velocity': {
                    'linear': msg.twist.twist.linear.x,
                    'angular': msg.twist.twist.angular.z
                }
            }
            print(json.dumps({'type': 'odom', 'data': self.odom_data}), flush=True)
        
        def spin(self):
            try:
                rclpy.spin(self.node)
            except KeyboardInterrupt:
                pass
            finally:
                self.node.destroy_node()
                rclpy.shutdown()

    if __name__ == '__main__':
        node = SensorNode()
        node.spin()
    """

    def _read_daemon_output(self):
        """Read sensor data from daemon stdout."""
        while self._running:
            try:
                line = self._daemon_process.stdout.readline()
                if not line:
                    break
                
                msg = json.loads(line.strip())
                msg_type = msg['type']
                data = msg['data']
                
                with self._lock:
                    if msg_type == 'lidar':
                        class LaserScanData:
                            def __init__(self, d):
                                self.ranges = d['ranges']
                                self.angle_min = d['angle_min']
                                self.angle_max = d['angle_max']
                                self.angle_increment = d['angle_increment']
                                self.range_min = d['range_min']
                                self.range_max = d['range_max']
                        self._lidar_cache = LaserScanData(data)
                    
                    elif msg_type == 'odom':
                        self._odom_cache = data
            
            except json.JSONDecodeError:
                continue
            except Exception as e:
                logger.error(f"Daemon read error: {e}")
                break

    def shutdown(self):
        """Shutdown bridge."""
        self._running = False
        
        if hasattr(self, '_daemon_process'):
            self._daemon_process.terminate()
            self._daemon_process.wait(timeout=2)
        
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