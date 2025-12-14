"""
ROS2 Telemetry Bridge for GUI Backend
Subscribes to /odom, /scan topics and streams data via callback
"""

import subprocess
import json
import asyncio
from typing import Callable, Optional
from dataclasses import dataclass
from datetime import datetime


@dataclass
class OdomData:
    """Odometry data structure"""
    x: float
    y: float
    theta: float
    linear_vel: float
    angular_vel: float
    timestamp: str


@dataclass
class ScanData:
    """LIDAR scan data structure"""
    ranges: list[float]  # List of distances
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    timestamp: str


class ROS2Bridge:
    """
    Manages ROS2 topic subscriptions using subprocess bridge
    Similar to robot_vision_controller's ros2_node.py pattern
    """
    
    def __init__(self):
        self.odom_process: Optional[subprocess.Popen] = None
        self.scan_process: Optional[subprocess.Popen] = None
        self.running = False
        
    async def start_odom_stream(self, callback: Callable):
        """
        Subscribe to /odom and call callback with parsed data
        Uses system Python 3.10 with rclpy
        """
        self.running = True
        
        # Python script that runs in subprocess
        script = """
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import sys
import math

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('gui_odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback,
            10
        )
        
    def callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to theta (yaw angle)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        
        # Extract velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Output JSON to stdout
        data = {
            'x': round(x, 3),
            'y': round(y, 3),
            'theta': round(theta, 3),
            'linear_vel': round(linear_vel, 3),
            'angular_vel': round(angular_vel, 3),
            'timestamp': str(msg.header.stamp.sec) + '.' + str(msg.header.stamp.nanosec)
        }
        print(json.dumps(data), flush=True)

def main():
    rclpy.init()
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        
        # Start subprocess with system Python 3.10
        self.odom_process = subprocess.Popen(
            ["python3", "-c", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        # Read stdout line by line asynchronously
        while self.running and self.odom_process.poll() is None:
            try:
                line = self.odom_process.stdout.readline()
                if line:
                    data = json.loads(line.strip())
                    odom = OdomData(
                        x=data['x'],
                        y=data['y'],
                        theta=data['theta'],
                        linear_vel=data['linear_vel'],
                        angular_vel=data['angular_vel'],
                        timestamp=datetime.now().isoformat()
                    )
                    await callback(odom)
                await asyncio.sleep(0.01)  # Prevent CPU spinning
            except Exception as e:
                print(f"Odom stream error: {e}")
                await asyncio.sleep(0.1)
    
    async def start_scan_stream(self, callback: Callable):
        """
        Subscribe to /scan and call callback with parsed data
        OPTIMIZED: Downsample to 36 points (every 10th point from 360)
        """
        self.running = True
        
        script = """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import sys

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('gui_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )
        
    def callback(self, msg):
        # DOWNSAMPLE: Take every 10th point (360 -> 36 points)
        ranges = list(msg.ranges[::10])
        
        data = {
            'ranges': [round(r, 2) if not (r < msg.range_min or r > msg.range_max) else -1 
                      for r in ranges],
            'angle_min': round(msg.angle_min, 3),
            'angle_max': round(msg.angle_max, 3),
            'angle_increment': round(msg.angle_increment * 10, 3),  # Adjusted for downsample
            'range_min': round(msg.range_min, 2),
            'range_max': round(msg.range_max, 2),
            'timestamp': str(msg.header.stamp.sec) + '.' + str(msg.header.stamp.nanosec)
        }
        print(json.dumps(data), flush=True)

def main():
    rclpy.init()
    node = ScanSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        
        self.scan_process = subprocess.Popen(
            ["python3", "-c", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        while self.running and self.scan_process.poll() is None:
            try:
                line = self.scan_process.stdout.readline()
                if line:
                    data = json.loads(line.strip())
                    scan = ScanData(
                        ranges=data['ranges'],
                        angle_min=data['angle_min'],
                        angle_max=data['angle_max'],
                        angle_increment=data['angle_increment'],
                        range_min=data['range_min'],
                        range_max=data['range_max'],
                        timestamp=datetime.now().isoformat()
                    )
                    await callback(scan)
                await asyncio.sleep(0.05)  # 20Hz max
            except Exception as e:
                print(f"Scan stream error: {e}")
                await asyncio.sleep(0.1)
    
    def stop(self):
        """Stop all ROS2 subscriptions"""
        self.running = False
        if self.odom_process:
            self.odom_process.terminate()
            self.odom_process.wait(timeout=2)
        if self.scan_process:
            self.scan_process.terminate()
            self.scan_process.wait(timeout=2)