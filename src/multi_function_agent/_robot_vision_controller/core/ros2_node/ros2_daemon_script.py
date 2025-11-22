#!/usr/bin/env python3
"""
ROS2 Daemon Script
Persistent subprocess for ROS2 communication (Python 3.10).

This script runs in system Python 3.10 and bridges ROS2 to Python 3.11 NAT container.
"""

import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
import logging
from rclpy.action import ActionClient
try:
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import Twist, PoseStamped, Quaternion
    from nav2_msgs.action import NavigateToPose
except ImportError:
    # Fallback to stubs if ROS2 not available
    from multi_function_agent._robot_vision_controller.utils.ros2_stubs import (
        LaserScan, Odometry, OccupancyGrid, Twist, PoseStamped, Quaternion, NavigateToPose
    )

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math
import time
import sys
import select

logger = logging.getLogger(__name__)

# =============================================================================
# QoS Configuration
# =============================================================================

def create_sensor_qos():
    """Create QoS profile for sensor data."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )


# =============================================================================
# ROS2 Daemon Class
# =============================================================================

class ROS2Daemon:
    """
    Persistent ROS2 daemon for sensor streaming and command execution.
    """
    
    CMD_QUEUE_FILE = '/tmp/ros2_cmd_queue.txt'
    
    def __init__(self):
        """Initialize daemon."""
        rclpy.init()
        self.node = rclpy.create_node('ros2_daemon')
        
        qos = create_sensor_qos()
        
        # Sensor subscribers
        self.lidar_sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos)
        
        # Command publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Nav2 action client
        self.nav2_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # State
        self.last_cmd = (0.0, 0.0)
        self.nav2_state = 'idle'
        self.nav2_goal_handle = None
        
        # Wait for Nav2 server
        self._wait_for_nav2_server()
    
    def _wait_for_nav2_server(self, timeout=5.0):
        """Wait for Nav2 action server."""
        server_available = self.nav2_client.wait_for_server(timeout_sec=timeout)
        self._publish_message({
            'type': 'nav2_server',
            'available': server_available
        })
    
    # =========================================================================
    # Sensor Callbacks
    # =========================================================================
    
    def lidar_callback(self, msg):
        """Process LiDAR scan data."""
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
        self._publish_message(data)
    
    def odom_callback(self, msg):
        """Process odometry data."""
        # Convert quaternion to yaw
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
        self._publish_message(data)
    
    def map_callback(self, msg):
        """Process SLAM map data."""
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
                'data_compressed': True  # Full map data too large for JSON
            }
        }
        self._publish_message(data)
    
    # =========================================================================
    # Command Processing
    # =========================================================================
    
    def read_cmd_queue(self):
        """Read velocity command from queue file."""
        try:
            with open(self.CMD_QUEUE_FILE, 'r') as f:
                line = f.read().strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        self.last_cmd = (float(parts[0]), float(parts[1]))
        except:
            pass
        return self.last_cmd
    
    def publish_cmd_vel(self):
        """Publish velocity command at fixed rate."""
        linear, angular = self.read_cmd_queue()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    # =========================================================================
    # Nav2 Integration
    # =========================================================================
    
    def send_nav2_goal(self, x, y, theta):
        """Send navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        # Send goal
        send_future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav2_feedback_callback
        )
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=2.0)
        
        goal_handle = send_future.result()

        # Handle timeout/ None case
        if goal_handle is None:
            logger.error("[NAV2 DAEMON] Goal send timeout - action server not ready")
            self.nav2_state = 'failed'
            self._publish_message({'type': 'nav2_state', 'state': 'failed'})
            return False
        
        if goal_handle.accepted:
            self.nav2_state = 'navigating'
            self.nav2_goal_handle = goal_handle
            self._publish_message({'type': 'nav2_state', 'state': 'navigating'})
            
            # Setup result callback
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav2_result_callback)
            return True
        else:
            self.nav2_state = 'failed'
            self._publish_message({'type': 'nav2_state', 'state': 'failed'})
            return False
    
    def nav2_result_callback(self, future):
        """Handle Nav2 navigation result."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.nav2_state = 'succeeded'
        else:
            self.nav2_state = 'failed'
        
        self._publish_message({'type': 'nav2_state', 'state': self.nav2_state})
    
    def nav2_feedback_callback(self, feedback_msg):
        """Handle Nav2 navigation feedback."""
        feedback = feedback_msg.feedback
        self._publish_message({
            'type': 'nav2_feedback',
            'distance_remaining': feedback.distance_remaining,
            'navigation_time': feedback.navigation_time.sec
        })
    
    def cancel_nav2_goal(self):
        """Cancel current Nav2 navigation."""
        if self.nav2_goal_handle:
            cancel_future = self.nav2_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)
            self.nav2_state = 'cancelled'
            return True
        return False
    
    # =========================================================================
    # Command Processing
    # =========================================================================
    
    def process_command(self, cmd):
        """Process command from stdin."""
        cmd_type = cmd.get('type')
        
        if cmd_type == 'nav2_goal':
            success = self.send_nav2_goal(cmd['x'], cmd['y'], cmd.get('theta', 0.0))
            self._publish_message({'type': 'nav2_response', 'success': success})
        
        elif cmd_type == 'nav2_cancel':
            success = self.cancel_nav2_goal()
            self._publish_message({'type': 'nav2_response', 'success': success})
    
    # =========================================================================
    # Main Loop
    # =========================================================================
    
    def spin(self):
        """Main daemon loop."""
        # Create timer for cmd_vel publishing (20Hz)
        timer = self.node.create_timer(0.05, self.publish_cmd_vel)
        
        try:
            while rclpy.ok():
                # Check for stdin commands (non-blocking)
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip()
                    if line:
                        try:
                            cmd = json.loads(line)
                            self.process_command(cmd)
                        except json.JSONDecodeError:
                            pass
                
                # Spin once
                rclpy.spin_once(self.node, timeout_sec=0.01)
        
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
    
    # =========================================================================
    # Helpers
    # =========================================================================
    
    def _publish_message(self, data):
        """Publish message to stdout (for parent process)."""
        print(json.dumps(data), flush=True)


# =============================================================================
# Entry Point
# =============================================================================

if __name__ == '__main__':
    daemon = ROS2Daemon()
    daemon.spin()