#!/bin/bash
# Khởi động toàn bộ robot stack: Pi (lidar + motor bridge) + Laptop (odom, scan, slam, teleop, rviz)
# Yêu cầu: SSH key đã setup laptop -> Pi (không cần gõ password)

PI_USER="pi"
PI_IP="192.168.2.185"
WS_DIR="$HOME/robot_ws"

run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

echo "Starting robot stack..."

# Pi: LiDAR bridge
run_in_terminal "ssh -t ${PI_USER}@${PI_IP} 'cd ~/robot_bridge && python3 lidar_bridge.py'"
sleep 10

# Pi: Motor/encoder bridge
run_in_terminal "ssh -t ${PI_USER}@${PI_IP} 'cd ~/robot_bridge && python3 robot_bridge.py'"
sleep 10

# Laptop: odom + cmd_vel bridge
run_in_terminal "cd ${WS_DIR} && python3 pi_robot_node.py"
sleep 2

# Laptop: lidar -> /scan
run_in_terminal "cd ${WS_DIR} && python3 lidar_ros2_node.py"
sleep 2

# Laptop: static TF + SLAM
run_in_terminal "cd ${WS_DIR} && ros2 launch slam_launch.py"
sleep 3

# Laptop: RViz
run_in_terminal "rviz2"
sleep 2

# Laptop: teleop (chạy cuối để bạn gõ ngay khi terminal mở)
run_in_terminal "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p publish_rate:=10.0"

echo "All services started!"
