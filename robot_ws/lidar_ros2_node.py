#!/usr/bin/env python3
"""LiDAR bridge: nhận JSON scan từ Pi (port 9001) → publish /scan (LaserScan)"""
import socket, json, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PI_IP     = '192.168.2.185'
PI_PORT   = 9001
FRAME_ID  = 'laser'
NUM_BINS  = 360  # 1 độ / bin


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(f'Kết nối Pi LiDAR {PI_IP}:{PI_PORT}...')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((PI_IP, PI_PORT))
        self.get_logger().info('✅ Đã kết nối LiDAR bridge')

        self.buf = ''
        self.timer = self.create_timer(0.02, self.poll)  # 50Hz poll

    def poll(self):
        try:
            self.sock.settimeout(0.01)
            data = self.sock.recv(65536).decode('utf-8', errors='ignore')
        except socket.timeout:
            return
        except Exception as e:
            self.get_logger().error(f'Socket err: {e}')
            return

        if not data:
            return

        self.buf += data
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            line = line.strip()
            if line:
                self.publish_scan(line)

    def publish_scan(self, line):
        try:
            pts = json.loads(line)  # [[angle_deg, dist_mm], ...]
        except json.JSONDecodeError:
            return
        if not pts:
            return

        ranges = [float('inf')] * NUM_BINS
        for angle_deg, dist_mm in pts:
            bin_idx = int(round(angle_deg)) % NUM_BINS
            r = dist_mm / 1000.0  # mm → m
            if 0.0 < r < ranges[bin_idx] or ranges[bin_idx] == float('inf'):
                ranges[bin_idx] = r

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / NUM_BINS
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()