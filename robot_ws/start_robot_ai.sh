#!/bin/bash
# Khởi động robot stack cho AI agent.
# Đặt tại: ~/robot_ws/start_robot_ai.sh
#
# Khác start_robot.sh (bản teleop):
#   + MediaMTX (server RTSP)
#   + camera trên Pi
#   + ffplay để xem stream
#   - KHÔNG mở teleop: daemon của agent publish /cmd_vel ở 20Hz,
#     teleop chạy song song sẽ tranh quyền điều khiển.

PI_USER="pi"
PI_IP="192.168.2.185"
WS_DIR="$HOME/robot_ws"

# --- kiểm tra trước khi mở 8 cái terminal ---
if [ ! -x "$HOME/mediamtx" ]; then
    echo "❌ Không thấy $HOME/mediamtx"
    exit 1
fi

LAPTOP_IP=$(ip -4 route get "${PI_IP}" 2>/dev/null | grep -oP 'src \K\S+')
if [ -z "$LAPTOP_IP" ]; then
    echo "❌ Không định tuyến được tới ${PI_IP}. Kiểm tra WiFi."
    exit 1
fi

if ! ping -c1 -W2 "${PI_IP}" >/dev/null 2>&1; then
    echo "❌ Pi ${PI_IP} không phản hồi."
    exit 1
fi

echo "Laptop IP: ${LAPTOP_IP}  →  Pi: ${PI_IP}"
echo "Sẽ hỏi mật khẩu SSH 3 lần (lidar / motor / camera)."
echo ""

run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

# MediaMTX phải chạy TRƯỚC khi Pi push stream
run_in_terminal "cd \$HOME && ./mediamtx"
sleep 3

run_in_terminal "ssh -t ${PI_USER}@${PI_IP} 'cd ~/robot_bridge && python3 lidar_bridge.py'"
sleep 10

run_in_terminal "ssh -t ${PI_USER}@${PI_IP} 'cd ~/robot_bridge && python3 robot_bridge.py'"
sleep 10

run_in_terminal "ssh -t ${PI_USER}@${PI_IP} 'cd ~/robot_bridge && LAPTOP_IP=${LAPTOP_IP} ./pi_camera_stream.sh'"
sleep 5

# pi_robot_node khởi động lại = odom reset về 0. Luôn chạy mới trước mỗi phiên.
run_in_terminal "cd ${WS_DIR} && python3 pi_robot_node.py"
sleep 2

run_in_terminal "cd ${WS_DIR} && python3 lidar_ros2_node.py"
sleep 2

run_in_terminal "cd ${WS_DIR} && ros2 launch slam_launch.py"
sleep 3

run_in_terminal "rviz2"
sleep 2

run_in_terminal "ffplay -fflags nobuffer -rtsp_transport tcp rtsp://127.0.0.1:8554/robotcam"

cat << 'EOF'

─────────────────────────────────────────────
Kiểm tra:
  ros2 topic list | grep -E 'scan|odom|map'     (phải đủ 3)
  Cửa sổ ffplay có hình

Verify map bằng tay (ĐÓNG tab này trước khi chạy AI):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p publish_rate:=10.0

Chạy AI:
cd ~/nemo-agent-toolkit/docker
./run_hybrid_container.sh

# trong container:
. /workspace/.venv/bin/activate
cd /workspace/mounted_code
uv pip install -e .

nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml \
  --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore 120 seconds, if you see a person follow them, otherwise stop'
─────────────────────────────────────────────
EOF
