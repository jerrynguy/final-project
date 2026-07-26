# Context: Robot thực + ROS2 + SLAM + AI (chuyển từ Gazebo sang robot thật)

## Phần cứng
- Raspberry Pi (Debian 13 Trixie, aarch64) — không có ROS2, chỉ chạy bridge script
- STM32 (Arduino framework, BTS7960 driver, encoder TIM3/TIM4)
- RPLIDAR A1M8
- Laptop (Ubuntu, ROS2 Humble) — xử lý chính (odom, SLAM, RViz, AI agent)

## Cổng USB trên Pi
- `/dev/ttyUSB0` → STM32 (CH340K)
- `/dev/ttyUSB1` → RPLIDAR A1M8 (CP2102)

## Kiến trúc
```
Laptop (ROS2) ←→ WiFi ←→ Pi (192.168.2.185)
                          ├── ttyUSB0 (STM32)  → TCP 9002 (cmd) / 9003 (encoder)
                          └── ttyUSB1 (LiDAR)  → TCP 9001 (raw scan JSON)
```

## File hiện có

**Pi `~/robot_bridge/`:**
- `lidar_bridge.py` — dùng thư viện `rplidar-roboticia`, parse scan thật (không phải raw serial nữa), gửi JSON `[[angle_deg, dist_mm], ...]` qua TCP 9001. Có gọi `lidar.clean_input()` trước `get_health()` để tránh lỗi "Incorrect descriptor starting bytes" do buffer rác từ lần chạy trước.
- `robot_bridge.py` — không đổi gì, hoạt động tốt. TCP 9002 nhận `M <pwm1> <pwm2>\n`, TCP 9003 gửi `ENC1: <val> ENC2: <val>\n`.

**Laptop `~/robot_ws/`:**
- `pi_robot_node.py` — publish `/odom` + TF `odom→base_link`, subscribe `/cmd_vel`. **Đã sửa gần đây:**
  - Thêm `TRIM_LEFT = 0.85` / `TRIM_RIGHT = 1` (hệ số nhân PWM từng bánh) để fix lỗi robot lệch trái khi đi thẳng — **đã fix xong, robot đi thẳng OK**.
  - Thêm `MAX_LINEAR_SPEED = 0.1` (m/s) — giới hạn tốc độ thực tế cho êm hơn, **tách riêng** khỏi `MAX_SPEED = 0.3` (hằng số hiệu chỉnh vật lý PWM=255 ↔ m/s, không được đổi giá trị này).
- `lidar_ros2_node.py` — nhận JSON từ Pi port 9001, publish `/scan` (LaserScan, 360 bin, frame_id `laser`). Hoạt động OK, ~7Hz.
- `slam_launch.py` — launch static TF `base_link→laser` (giả định lidar ở tâm robot, cao 5cm, không xoay — **chưa verify vị trí lidar thực tế**) + `slam_toolbox` (async). **Chạy bằng `ros2 launch slam_launch.py`, KHÔNG PHẢI `python3`**.
- `start_robot.sh` — script mở 7 tab gnome-terminal tự động: SSH Pi (lidar_bridge, robot_bridge) → pi_robot_node → lidar_ros2_node → slam launch → rviz2 → teleop. Cần SSH key để không bị hỏi password (**user vẫn đang dùng password thủ công**).

## Trạng thái SLAM/Mapping
- Pipeline kỹ thuật đã chạy được end-to-end (`/scan`, `/odom`, `/tf`, `/map` đều xuất hiện).
- **Vấn đề đã fix:** robot lệch trái khi đi thẳng → do mất cân bằng PWM 2 bánh → đã thêm TRIM để sửa.
- **Vấn đề còn tồn tại:** bánh xe trượt trên mặt gạch (ma sát kém) → encoder đếm quay nhưng xe không di chuyển tương ứng → odom vẫn có thể sai → map từng bị vỡ (hình "nan quạt" tỏa ra nhiều điểm, dấu hiệu điển hình của odom lỗi nặng).
- Đã thử giải pháp tạm: dán băng dính điện đen lên bánh xe để tăng ma sát (rẻ, nhanh, nhưng không bền — sau này nên đổi bánh cao su gân).
- **User quyết định: tạm gác SLAM mapping lại**, chưa verify map có đẹp hơn sau khi fix trim + dán băng dính hay chưa. Đây là việc cần quay lại sau.

## Thông số robot (trong `pi_robot_node.py`, cần đo lại cho chính xác)
```python
WHEEL_RADIUS = 0.033   # m
WHEEL_BASE   = 0.160   # m
ENCODER_PPR  = 1440
MAX_SPEED    = 0.3     # m/s ứng PWM=255 (hằng số hiệu chỉnh vật lý, đừng đổi)
MAX_LINEAR_SPEED = 0.1 # m/s giới hạn tốc độ thực tế (mới thêm)
TRIM_LEFT / TRIM_RIGHT # hệ số cân bằng 2 bánh (mới thêm, giá trị cuối chưa chốt)
```

## Đã hoạt động
- ✅ Motor tiến/lùi đúng chiều
- ✅ Quay trái/phải đúng chiều (z dương = trái)
- ✅ Đi thẳng không lệch (sau khi thêm TRIM)
- ✅ Keyboard teleop, tốc độ đã giảm êm hơn (0.1 m/s)
- ✅ Encoder đọc được
- ✅ LiDAR → `/scan` publish OK
- ✅ SLAM pipeline chạy được kỹ thuật (không lỗi), nhưng map chất lượng còn nghi vấn do trượt bánh

## Chưa làm / đang dang dở
- ❌ Verify lại map sau khi fix trim + băng dính (tạm gác)
- ❌ Đo chính xác `WHEEL_RADIUS`, `WHEEL_BASE`, `ENCODER_PPR`
- ❌ Verify vị trí lidar thực tế trên robot (đang giả định ở tâm, cao 5cm)
- ❌ **AI điều khiển robot thật** — ưu tiên tiếp theo (xem phần dưới)

---

## Phần AI (từng làm trên Gazebo, giờ chuyển sang robot thật)

### Setup cũ trên Gazebo (tham khảo, KHÔNG dùng nữa cho robot thật)
Dùng script mở nhiều terminal: Gazebo, slam_toolbox (`use_sim_time:=True`), RViz, MediaMTX (RTSP server), `rtsp_publisher.py` (custom, publish camera ảo ra RTSP), `ffplay` xem thử stream.

Sau đó chạy AI agent trong Docker:
```bash
cd ~/nemo-agent-toolkit/docker
./run_hybrid_container.sh          # start container interactive

# Trong container, lần đầu:
. /workspace/.venv/bin/activate
cd /workspace/mounted_code
uv pip install -e .

# Chạy AI agent:
nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml \
  --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore 120 seconds, if you see a person follow them, otherwise stop'
```

### Kế hoạch cho robot thật
User dự định khi chuyển sang robot thật, phần chạy AI sẽ **chỉ còn**:
```bash
cd /workspace/mounted_code
uv pip install -e .
nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml \
  --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore 120 seconds, if you see a person follow them, otherwise stop'
```
(bỏ hết phần Gazebo/mediamtx/rtsp_publisher/ffplay thủ công — có thể cần script mới tương đương `start_robot.sh` để tự động hóa phần chuẩn bị RTSP stream + ROS2 bridge trước khi chạy NAT)

### Việc cần làm tiếp (chưa bắt đầu, ưu tiên hàng đầu ở chat mới)
1. **Review code trước khi chạy trên robot thật**, đặc biệt:
   - File `ros2_daemon_script.py` — **user đã tự sửa để thích ứng với robot thật**, nhưng CHƯA được review/kiểm tra kỹ. Cần user upload nội dung file này ở chat mới để xem lại.
   - Cấu hình `config.yml` của `multi_function_agent` — cần xem có phần nào hardcode theo Gazebo (topic name, service, sim_time...) không.
   - Cần xác nhận robot thật cần camera thật (không phải camera ảo Gazebo) → RTSP pipeline có thể cần điều chỉnh source stream (camera vật lý gắn ở đâu, driver gì, publish qua `rtsp_publisher.py` cũ có tương thích không).
2. Xác nhận luồng: camera thật → RTSP (MediaMTX + rtsp_publisher.py hoặc tương đương) → NAT agent đọc RTSP → điều khiển robot qua `/cmd_vel` (ROS2) → `pi_robot_node.py` → Pi → STM32.
3. Có thể cần viết lại `start_robot.sh` version mới bao gồm cả các bước RTSP/camera cho robot thật (thay vì Gazebo).

---

## Việc cần làm ngay ở chat mới
1. User sẽ upload/paste nội dung `ros2_daemon_script.py` (bản đã sửa) để review.
2. Xác nhận camera vật lý dùng loại gì, kết nối kiểu gì (USB webcam? Pi camera?).
3. Review `config.yml` của multi_function_agent.
4. Viết/sửa script khởi động tương đương `start_robot.sh` cho luồng AI + camera thật.
