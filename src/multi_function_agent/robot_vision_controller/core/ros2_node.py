"""
ROS2 Node Module - Persistent Daemon Subprocess
Uses system Python 3.10 with rclpy via long-running subprocess.
"""

import subprocess
import json
import logging
import threading
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class ROS2Bridge:
    """
    Bridge between Python 3.11+ (NAT) and Python 3.10 (ROS2).
    Uses persistent subprocess daemon for real-time sensor data.
    """
    
    def __init__(self):
        self.system_python = '/usr/bin/python3'
        self._lidar_cache = None
        self._odom_cache = None
        self._monitor_thread = None
        self._daemon_process = None
        self._running = False
        self._lock = threading.Lock()
        
        # Start persistent daemon
        self._start_monitor()
        
        logger.info("✅ ROS2Bridge initialized (daemon mode)")
    
    def get_name(self):
        """Get node name (compatibility)."""
        return "ros2_bridge_daemon"
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command via subprocess."""
        script = """
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('cmd_vel_pub')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

import time
time.sleep(0.02)

msg = Twist()
msg.linear.x = {linear}
msg.angular.z = {angular}
pub.publish(msg)

node.destroy_node()
rclpy.shutdown()
"""
        try:
            subprocess.run(
                [self.system_python, '-c', script],
                capture_output=True,
                timeout=0.5
            )
        except Exception as e:
            logger.debug(f"Publish error: {e}")
    
    def publish_stop(self):
        """Publish stop command."""
        self.publish_velocity(0.0, 0.0)
    
    def get_scan(self) -> Optional[Any]:
        """Get cached LIDAR data."""
        with self._lock:
            return self._lidar_cache
    
    def get_odom(self) -> Optional[Any]:
        """Get cached odometry data."""
        with self._lock:
            return self._odom_cache
    
    def get_robot_pose(self) -> Optional[Dict]:
        """Get robot pose from odometry."""
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
        """Start persistent daemon subprocess."""
        self._running = True
        
        script = """
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math

# RELIABLE QoS to match Gazebo publisher
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class SensorDaemon:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('sensor_daemon')
        
        self.lidar_sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_cb, qos)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_cb, qos)
    
    def lidar_cb(self, msg):
        data = {
            'type': 'lidar',
            'data': {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            }
        }
        print(json.dumps(data), flush=True)
    
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny, cosy)
        
        data = {
            'type': 'odom',
            'data': {
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
        }
        print(json.dumps(data), flush=True)
    
    def spin(self):
        try:
            rclpy.spin(self.node)
        except:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

daemon = SensorDaemon()
daemon.spin()
"""
        
        try:
            self._daemon_process = subprocess.Popen(
                [self.system_python, '-c', script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            self._monitor_thread = threading.Thread(
                target=self._read_daemon,
                daemon=True
            )
            self._monitor_thread.start()
            
            logger.info("✅ ROS2 daemon subprocess started")
        except Exception as e:
            logger.error(f"❌ Failed to start daemon: {e}")
            raise
    
    def _read_daemon(self):
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
        
        logger.warning("Daemon reader stopped")
    
    def shutdown(self):
        """Shutdown bridge and daemon."""
        self._running = False
        
        if self._daemon_process:
            self._daemon_process.terminate()
            try:
                self._daemon_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._daemon_process.kill()
        
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        
        logger.info("✅ ROS2Bridge shutdown")


# Singleton
_bridge_instance = None
_bridge_lock = threading.Lock()

def get_ros2_node():
    """Get or create ROS2 bridge."""
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