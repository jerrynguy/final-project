#!/usr/bin/env python3
"""
ROS2 node trên laptop:
- Kết nối Pi lấy encoder → publish /odom
- Subscribe /cmd_vel → gửi Pi điều khiển motor
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import socket, threading, math, re

# === THAY IP PI ===
PI_IP = '192.168.2.185'   # hostname -I trên Pi

# === THAY THÔNG SỐ ROBOT ===
WHEEL_RADIUS = 0.033   # m - bán kính bánh
WHEEL_BASE   = 0.160   # m - khoảng cách 2 bánh
ENCODER_PPR  = 1440    # pulse/vòng (encoder lines * gear ratio)
MAX_PWM      = 255
MAX_SPEED    = 0.3            # m/s ứng với PWM=255 (hiệu chỉnh vật lý, không đổi)
MAX_LINEAR_SPEED = 0.1        # m/s - tốc độ tối đa cho phép chạy
MAX_ANGULAR_SPEED = 0.5        # rad/s
TRIM_LEFT  = 0.85   # thử trước, bánh trái đang mạnh hơn nên giảm
TRIM_RIGHT = 1.0

class PiRobotNode(Node):
    def __init__(self):
        super().__init__('pi_robot_node')

        # TCP kết nối Pi
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cmd_sock.connect((PI_IP, 9002))
        self.get_logger().info(f'✅ CMD connected to Pi {PI_IP}:9002')

        self.enc_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.enc_sock.connect((PI_IP, 9003))
        self.get_logger().info(f'✅ ENC connected to Pi {PI_IP}:9003')

        # ROS
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br    = TransformBroadcaster(self)

        # Odometry state
        self.x = self.y = self.theta = 0.0
        self.prev_enc1 = self.prev_enc2 = None

        # Đọc encoder từ Pi trong background thread
        threading.Thread(target=self._enc_loop, daemon=True).start()
        self.get_logger().info('Pi robot node started')

    def cmd_cb(self, msg: Twist):
        """Twist → PWM → gửi Pi"""
        v = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, msg.linear.x))
        w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, msg.angular.z))

        v_left  = v + w * WHEEL_BASE / 2.0
        v_right = v - w * WHEEL_BASE / 2.0

        pwm1 = int(v_left  / MAX_SPEED * MAX_PWM)
        pwm2 = -int(v_right / MAX_SPEED * MAX_PWM)
        pwm1 = int(pwm1 * TRIM_LEFT)
        pwm2 = int(pwm2 * TRIM_RIGHT)
        pwm1 = max(-255, min(255, pwm1))
        pwm2 = max(-255, min(255, pwm2))

        cmd = f"M {pwm1} {pwm2}\n"
        try:
            self.cmd_sock.sendall(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')

    def _enc_loop(self):
        """Parse encoder từ STM32: 'ENC1: 1234  ENC2: 5678'"""
        buf = ""
        pattern = re.compile(r'ENC1:\s*(-?\d+)\s+ENC2:\s*(-?\d+)')

        while True:
            try:
                data = self.enc_sock.recv(256).decode('utf-8', errors='ignore')
                if not data:
                    break
                buf += data
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    m = pattern.search(line)
                    if m:
                        enc1 = int(m.group(1))
                        enc2 = int(m.group(2))
                        self._update_odom(enc1, enc2)
            except Exception as e:
                self.get_logger().error(f'ENC read error: {e}')
                break

    def _update_odom(self, enc1: int, enc2: int):
        if self.prev_enc1 is None:
            self.prev_enc1, self.prev_enc2 = enc1, enc2
            return

        # Delta ticks → distance (m)
        d1 = (enc1 - self.prev_enc1) / ENCODER_PPR * 2 * math.pi * WHEEL_RADIUS
        d2 = (enc2 - self.prev_enc2) / ENCODER_PPR * 2 * math.pi * WHEEL_RADIUS
        self.prev_enc1, self.prev_enc2 = enc1, enc2

        d_center = (d1 + d2) / 2.0
        d_theta  = (d2 - d1) / WHEEL_BASE

        self.x     += d_center * math.cos(self.theta + d_theta / 2)
        self.y     += d_center * math.sin(self.theta + d_theta / 2)
        self.theta += d_theta

        now = self.get_clock().now()
        self._publish_odom(now)

    def _publish_odom(self, stamp):
        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)

        # TF
        tf = TransformStamped()
        tf.header.stamp    = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

        # Odom
        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.cmd_sock.sendall(b'M 0 0\n')
        super().destroy_node()

def main():
    rclpy.init()
    node = PiRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
