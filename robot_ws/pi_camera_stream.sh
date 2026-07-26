#!/bin/bash
# Đặt tại: ~/robot_bridge/pi_camera_stream.sh (trên Pi)
# CSI camera (OV5647) -> H264 -> RTSP push lên MediaMTX của laptop
#
# Yêu cầu: sudo apt install -y rpicam-apps ffmpeg
# Được start_robot_ai.sh gọi tự động, hoặc chạy tay:
#   LAPTOP_IP=192.168.2.81 ./pi_camera_stream.sh

LAPTOP_IP="${LAPTOP_IP:-192.168.2.1}"

echo "Streaming camera -> rtsp://${LAPTOP_IP}:8554/robotcam"

# rpicam-vid chỉ xuất YUV thô, ffmpeg lo encode + RTSP.
# Không dùng --codec h264 / --libav-format rtsp: cả hai đều không
# ép được RTSP-over-TCP, MediaMTX trả về 400 Bad Request.
rpicam-vid -t 0 \
    --width 640 \
    --height 480 \
    --framerate 10 \
    --codec yuv420 \
    --rotation 180 \
    --nopreview \
    -o - \
| ffmpeg -hide_banner \
    -f rawvideo -pix_fmt yuv420p -s 640x480 -r 10 -i - \
    -c:v libx264 -preset ultrafast -tune zerolatency -g 10 \
    -f rtsp -rtsp_transport tcp \
    "rtsp://${LAPTOP_IP}:8554/robotcam"

# Ghi chú:
#   --rotation 180  : camera lắp ngược trên khung robot
#   -tune zerolatency: tắt B-frame, giảm trễ vòng điều khiển
#   -g 10           : keyframe mỗi 1 giây, OpenCV bắt stream nhanh hơn
#   Pi 5 KHÔNG có encoder H264 phần cứng -> libx264 chạy software.
#   CPU cao thì hạ -r 8 hoặc 480x360.
