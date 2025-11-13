"""
ROS2 Node Module - Persistent Daemon Subprocess
Uses system Python 3.10 with rclpy via long-running subprocess.
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
    Uses persistent subprocess daemon for real-time sensor data AND commands.
    """
    
    def __init__(self):
        self.system_python = '/usr/bin/python3'
        self._lidar_cache = None
        self._odom_cache = None
        self._monitor_thread = None
        self._daemon_process = None
        self._running = False
        self._lock = threading.Lock()
        
        # Command queue for publisher daemon
        self._cmd_queue_file = '/tmp/ros2_cmd_queue.txt'
        
        # Start persistent daemon
        self._start_monitor()

        self._nav2_state_cache = 'idle'  # idle, navigating, succeeded, failed
        self._slam_map_cache = None
        
        logger.info("✅ ROS2Bridge initialized (daemon mode)")

    def send_nav2_goal(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Send Nav2 goal via daemon."""
        try:
            cmd = {
                'type': 'nav2_goal',
                'x': x,
                'y': y,
                'theta': theta
            }
            self._daemon_process.stdin.write(json.dumps(cmd) + '\n')
            self._daemon_process.stdin.flush()
            return True
        except Exception as e:
            logger.error(f"Nav2 goal send failed: {e}")
            return False

    def cancel_nav2_goal(self) -> bool:
        """Cancel Nav2 goal."""
        try:
            cmd = {'type': 'nav2_cancel'}
            self._daemon_process.stdin.write(json.dumps(cmd) + '\n')
            self._daemon_process.stdin.flush()
            return True
        except Exception as e:
            logger.error(f"Nav2 cancel failed: {e}")
            return False

    def get_nav2_state(self) -> Optional[str]:
        """Get Nav2 navigation state."""
        with self._lock:
            return self._nav2_state_cache

    def get_slam_map(self) -> Optional[Dict]:
        """Get SLAM map data."""
        with self._lock:
            return self._slam_map_cache
    
    def get_name(self):
        """Get node name (compatibility)."""
        return "ros2_bridge_daemon"
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command via file queue (non-blocking)."""
        try:
            with open(self._cmd_queue_file, 'w') as f:
                f.write(f"{linear},{angular}\n")
        except Exception as e:
            logger.debug(f"Queue write error: {e}")
    
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
        
        # Create empty queue file
        try:
            open(self._cmd_queue_file, 'w').close()
        except:
            pass
        
        script = """
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
from rclpy.action import ActionClient
try:
    from sensor_msgs.msg import LaserScan
except ImportError:
    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import LaserScan
try:
    from nav_msgs.msg import Odometry, OccupancyGrid
except ImportError:
    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import Odometry, OccupancyGrid
try:
    from geometry_msgs.msg import Twist, PoseStamped, Quaternion
except ImportError:
    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import Twist, PoseStamped, Quaternion
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math
import time
import sys
import select

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class ROS2Daemon:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('ros2_daemon')
        
        # Sensor subscribers
        self.lidar_sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_cb, qos)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_cb, qos)
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_cb, qos)
        
        # Command publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Nav2 action client
        self.nav2_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # State
        self.cmd_queue_file = '/tmp/ros2_cmd_queue.txt'
        self.last_cmd = (0.0, 0.0)
        self.nav2_state = 'idle'
        self.nav2_goal_handle = None
        
        # Wait for Nav2 server
        self.nav2_client.wait_for_server()
    
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
    
    def map_cb(self, msg):
        # Only send map updates periodically (avoid flooding)
        data = {
            'type': 'slam_map',
            'data': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y
                },
                'data_compressed': True  # Signal that full map is available
            }
        }
        print(json.dumps(data), flush=True)
    
    def read_cmd_queue(self):
        try:
            with open(self.cmd_queue_file, 'r') as f:
                line = f.read().strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        self.last_cmd = (float(parts[0]), float(parts[1]))
        except:
            pass
        return self.last_cmd
    
    def publish_cmd_vel(self):
        linear, angular = self.read_cmd_queue()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def send_nav2_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        send_future = self.nav2_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=2.0)
        
        goal_handle = send_future.result()
        if goal_handle.accepted:
            self.nav2_state = 'navigating'
            self.nav2_goal_handle = goal_handle
            
            # Monitor result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav2_result_cb)
            
            return True
        return False
    
    def nav2_result_cb(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.nav2_state = 'succeeded'
        else:
            self.nav2_state = 'failed'
        
        # Emit state update
        data = {'type': 'nav2_state', 'state': self.nav2_state}
        print(json.dumps(data), flush=True)
    
    def cancel_nav2_goal(self):
        if self.nav2_goal_handle:
            cancel_future = self.nav2_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)
            self.nav2_state = 'cancelled'
            return True
        return False
    
    def process_command(self, cmd):
        cmd_type = cmd.get('type')
        
        if cmd_type == 'nav2_goal':
            success = self.send_nav2_goal(cmd['x'], cmd['y'], cmd.get('theta', 0.0))
            response = {'type': 'nav2_response', 'success': success}
            print(json.dumps(response), flush=True)
        
        elif cmd_type == 'nav2_cancel':
            success = self.cancel_nav2_goal()
            response = {'type': 'nav2_response', 'success': success}
            print(json.dumps(response), flush=True)
    
    def spin(self):
        timer = self.node.create_timer(0.05, self.publish_cmd_vel)
        
        try:
            while rclpy.ok():
                # Check for commands from stdin (non-blocking)
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip()
                    if line:
                        try:
                            cmd = json.loads(line)
                            self.process_command(cmd)
                        except:
                            pass
                
                # Spin ROS2 node
                rclpy.spin_once(self.node, timeout_sec=0.01)
        
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

daemon = ROS2Daemon()
daemon.spin()
"""
        
        try:
            self._daemon_process = subprocess.Popen(
                [self.system_python, '-c', script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
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
        """Read sensor data + Nav2 state from daemon stdout."""
        while self._running:
            try:
                line = self._daemon_process.stdout.readline()
                if not line:
                    break
                
                msg = json.loads(line.strip())
                msg_type = msg['type']
                
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
                        self._lidar_cache = LaserScanData(msg['data'])
                    
                    elif msg_type == 'odom':
                        self._odom_cache = msg['data']
                    
                    elif msg_type == 'nav2_state':
                        self._nav2_state_cache = msg['state']
                    
                    elif msg_type == 'slam_map':
                        self._slam_map_cache = msg['data']
            
            except json.JSONDecodeError:
                continue
            except Exception as e:
                logger.error(f"Daemon read error: {e}")
                break
        
        logger.warning("Daemon reader stopped")
    
    def shutdown(self):
        """Shutdown bridge and daemon."""
        self._running = False
        
        # Send stop command
        self.publish_stop()
        time.sleep(0.1)
        
        if self._daemon_process:
            self._daemon_process.terminate()
            try:
                self._daemon_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._daemon_process.kill()
        
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        
        # Cleanup queue file
        try:
            import os
            os.remove(self._cmd_queue_file)
        except:
            pass
        
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