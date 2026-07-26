# Robot thật + ROS2 + SLAM + AI — hướng dẫn vận hành

**Trạng thái: pipeline đã thông.** Agent khởi động, đọc camera + LiDAR + odom, ra quyết định
và phát `/cmd_vel` mà robot thi hành được.

Đã kiểm chứng tới **iteration 1** rồi dừng tay. Chưa có lần chạy nào trọn 120 giây — nên
chưa biết hành vi explore dài hạn ra sao (có kẹt xoay vòng không, có spam escape không,
có kết thúc mission đúng cách không). Giai đoạn tiếp theo là tinh chỉnh hành vi, không phải
dựng hệ thống.

---

## Bắt đầu phiên mới — làm theo thứ tự này

```bash
# 1. Khởi động stack
~/robot_ws/start_robot_ai.sh

# 2. Kiểm tra nhanh
ros2 topic list | grep -E 'scan|odom|map'      # đủ 3
# cửa sổ ffplay có hình

# 3. Chạy AI
cd ~/nemo-agent-toolkit/docker && ./run_hybrid_container.sh
#   trong container:
. /workspace/.venv/bin/activate && cd /workspace/mounted_code && uv pip install -e .

nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml \
  --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore for 60 seconds'
```

Lệnh trên là **explore đơn** (không có mệnh đề `if ... otherwise ...`) — chạy trọn 60 giây
để lấy log đầy đủ. Chạy trên **sàn gỗ**: sàn gạch bánh trượt, thêm biến đó vào thì không
tách được "agent lái dở" với "bánh không bám".

Sửa code trên host rồi test lại: chỉ cần `uv pip install -e .` trong container.
Không phải restart stack, không phải dựng lại container.

---

## Kiến trúc

```
Pi camera (CSI/OV5647) ──H264/RTSP──> MediaMTX (laptop:8554) ──> NAT agent (Docker)
                                                                        │
LiDAR ──TCP 9001──> lidar_ros2_node ──> /scan ──> ros2_daemon    /cmd_vel│ 20Hz
                                                                        ▼
Encoder ──TCP 9003──> pi_robot_node ──> /odom ──────────────> pi_robot_node
                                          │                            │
                                    slam_toolbox ──> /map        TCP 9002
                                                                        ▼
                                                              Pi ──serial──> STM32
```

MediaMTX chạy trên **laptop**, Pi đẩy stream lên. Nhờ vậy URL `rtsp://172.17.0.1:8554/robotcam`
giữ nguyên từ thời Gazebo, câu lệnh `nat run` không phải đổi.

| Cổng | Dùng cho |
|---|---|
| 9001 | LiDAR scan (Pi → laptop) |
| 9002 | Lệnh motor (laptop → Pi) |
| 9003 | Encoder (Pi → laptop) |
| 8554 | RTSP camera (Pi → laptop) |

---

# Quy trình chạy

## 1. Khởi động stack

```bash
~/robot_ws/start_robot_ai.sh
```

Mở 8 tab, hỏi mật khẩu SSH 3 lần. Kiểm tra:

```bash
ros2 topic list | grep -E 'scan|odom|map'   # đủ 3
```
Cửa sổ ffplay phải có hình.

## 2. Verify map bằng tay

**Đừng bỏ qua bước này khi đổi môi trường hoặc đổi mặt sàn.**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p publish_rate:=10.0
```

RViz: Fixed Frame = `map`, Add → Map → `/map`. Lái ~2 phút, kiểm tra:

1. Tường thẳng, không dày lên
2. Góc phòng vuông
3. **Đi một vòng khép kín** — nét mới chồng khít nét cũ, không tách thành bản sao lệch

Số 3 quan trọng nhất; sai số odom chỉ lộ khi khép vòng.

> **Sàn gỗ: OK. Sàn gạch: bánh trượt, map chồng chéo.** Thông số bánh xe đúng
> (nếu sai thì gỗ cũng hỏng) — đây thuần là vấn đề ma sát.

**Lái xong đóng tab teleop**, không thì tranh `/cmd_vel` với agent.

## 3. Chạy AI

```bash
cd ~/nemo-agent-toolkit/docker
./run_hybrid_container.sh

# trong container:
. /workspace/.venv/bin/activate
cd /workspace/mounted_code
uv pip install -e .

nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml \
  --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore 120 seconds, if you see a person follow them, otherwise stop'
```

Hỏi `Is SLAM Toolbox running on host? (y/n):` → gõ `y`.

> Log khuyên launch slam_toolbox với `use_sim_time:=True` — **bỏ qua**, chữ sót từ Gazebo.
> `slam_launch.py` để `False`, đúng cho robot thật.

Sửa code trên host rồi test lại: chỉ cần `uv pip install -e .` trong container, không phải restart.

---

# Đã sửa những gì (tham khảo)

## Pi

| Việc | Chi tiết |
|---|---|
| Vô hiệu repo ROS2 chết | `/etc/apt/sources.list.d/ros2.list` → `.disabled`. ROS2 không build cho Debian Trixie, repo 404 chặn mọi `apt update` |
| Cắm lại cáp CSI | Mặt kim loại quay ngược hướng lẫy. Camera nhận ra là **OV5647** (Pi Camera v1) |
| Tạo `~/robot_bridge/pi_camera_stream.sh` | Xem file kèm theo |

## Laptop — `mediamtx.yml` dòng 619

```yaml
paths:
  robotcam:
    source: publisher        # was: source: udp://127.0.0.1:9000
```

**Đây là thủ phạm chính của bế tắc camera.** Path khai `source:` là path kiểu *pull* —
MediaMTX tự đi lấy nguồn và từ chối mọi ai publish vào, trả `400 Bad Request`.
Cấu hình cũ trỏ tới `rtsp_publisher.py` thời Gazebo (đẩy MPEG-TS qua UDP 9000).
Ba cách encode khác nhau trên Pi đều chết vì lý do này, không phải do video.

## Laptop — `pi_robot_node.py`

```python
MAX_ANGULAR_SPEED = 0.5    # thêm mới
...
w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, msg.angular.z))
```

`cmd_cb` clamp `linear.x` nhưng bỏ sót `angular.z`. Với `w = 2.5`:

```
v_left  = 0.1 + 2.5×0.08 = 0.30 m/s  →  pwm1 = 255 (kịch trần)
v_right = 0.1 − 0.20     = −0.10     →  pwm2 = 85
```

PWM 255 tức thời trên gạch = bánh quay tại chỗ, encoder vẫn đếm → odom sai → map vỡ.

## Laptop — `robot_controller_interface.py`

```python
func_config = config.get('functions', {}).get('robot_vision_controller', {})
#                                              ^ bỏ dấu _ ở đầu
```

Code tra key `_robot_vision_controller` nhưng `config.yml` khai `robot_vision_controller`.
Không khớp → dict rỗng → mọi giá trị rơi về default hardcode (`max_angular_velocity = 2.84`,
`use_nav2 = True`). Tên thư mục package `_robot_vision_controller/` không liên quan gì tới
key YAML — chỉ trùng tên nên dễ nhầm.

## Laptop — `config.yml`

```yaml
    max_speed: 0.1                 # 0.6
    exploration_speed_boost: 1.0   # 1.5
    max_linear_velocity: 0.1       # 0.6
    max_angular_velocity: 0.5      # 2.5
    use_nav2: false                # true
    slam_auto_save_interval: 20.0  # 5.0 — 5s là quá dày, mỗi lần spawn map_saver_cli
```

**`critical_distance`, `warning_distance`, `safe_distance`, `min_valid_range` là config chết** —
không code nào đọc. Ngưỡng thật nằm trong `SafetyThresholds` (`utils/safety_checks.py`).

Về `use_nav2: false`: Nav2 ≠ SLAM. slam_toolbox (vẽ bản đồ) vẫn chạy bình thường.
Nav2 (tự lái tới toạ độ đích) chưa hề được khởi động — để `true` chỉ tốn 15 giây chờ timeout
rồi cũng fallback về lái tay.

## Laptop — `utils/safety_checks.py`

```python
HARDWARE_LIMIT = 0.15          # 0.12 — dưới tầm đo tối thiểu A1M8, ngưỡng không thể kích hoạt
MAX_SAFE_LINEAR_VEL = 0.15     # 0.6  — để mô hình chuyển động của agent khớp thực tế
MAX_SAFE_ANGULAR_VEL = 0.6     # 2.5
```

`MAX_SAFE_LINEAR_VEL = 0.6` không gây nguy hiểm trực tiếp (`pi_robot_node` vẫn cắt về 0.1)
nhưng khiến agent tưởng mình đi 0.5 m/s trong khi thực tế 0.1 — **mô hình chuyển động sai 5 lần**,
mọi suy luận "tiến 1m rồi rẽ" đều lệch.

---

# 5 loại nhiệm vụ

Đăng ký trong `mission_controller.py` → `MISSION_CLASSES`. Parser tự chọn loại từ câu lệnh,
không khai báo tường minh được.

| Loại | Câu lệnh mẫu | Yêu cầu | Trạng thái |
|---|---|---|---|
| `explore_area` | `explore for 60 seconds` | slam_toolbox đang chạy (hỏi y/n) | chưa test riêng |
| `follow_target` | `follow the person` | có `target_class` trong câu | chưa |
| `patrol_laps` | `go around 20 times for 60 seconds` | **map có sẵn** trên đĩa | chưa |
| `composite_mission` | câu có `if ... otherwise ...` | validate từng bước lúc chạy | ✅ đã chạy |
| `path_following` | — | chưa rõ | chưa |

## Ràng buộc thứ tự

`patrol_laps` **phụ thuộc** vào map do `explore_area` tạo ra. `_validate_patrol_requirements()`
tìm `my_map.yaml` ở các đường dẫn:

```
/workspace/mounted_code/maps/my_map.yaml     <- slam_map_save_path hiện tại
/workspace/persistent_data/maps/my_map.yaml
/root/maps/my_map.yaml
/root/my_map.yaml
~/my_map.yaml
```

Không có map thì patrol báo lỗi ngay, không chạy.

⚠️ Nó **chỉ kiểm tra file tồn tại và đủ lớn**, không kiểm tra chất lượng. Map nát vẫn qua.
Nên trước khi làm patrol phải tự mở map ra nhìn.

## Thứ tự đề xuất

```
1. explore_area đơn         'explore for 60 seconds'
   → cô lập được hành vi explore, không lẫn logic composite
   → verify map trong RViz, ĐÂY là điều kiện cần cho patrol

2. follow_target đơn        'follow the person'
   → test riêng nhánh YOLO + bám mục tiêu, không dính SLAM

3. patrol_laps              'go around 20 times for 60 seconds'
   → chỉ làm sau khi bước 1 cho ra map đẹp

4. composite_mission        'explore 120 seconds, if you see a person follow them, otherwise stop'
   → ghép lại, lúc này mỗi thành phần đã biết chạy đúng
```

Bạn đang ở bước 4 mà chưa qua 1–3. Chạy được là tốt, nhưng lúc có lỗi sẽ khó biết
tại explore, tại follow, hay tại logic chuyển bước của composite.

# Lỗi thường gặp

| Triệu chứng | Nguyên nhân |
|---|---|
| `Cannot connect to stream` | MediaMTX chưa chạy, hoặc `LAPTOP_IP` sai |
| `400 Bad Request` khi publish | `mediamtx.yml` path còn `source:` cố định |
| `[h264] Missing reference picture` | Bình thường lúc mới bắt stream, hết sau 1 keyframe |
| Stream lag, CPU Pi cao | Pi 5 encode software — hạ `-r 8` hoặc 480x360 |
| Robot giật, map vỡ | Clamp trong `pi_robot_node.py` bị mất |
| Robot không nhúc nhích | Teleop còn mở tranh `/cmd_vel` |
| `ros2 topic pub --once` không ăn | Discovery chưa kịp. Dùng `timeout 3 ros2 topic pub -r 10 ...` |
| Map không save | Thiếu thư mục `maps/` |

---

# Còn tồn đọng

**Hoãn có chủ đích — không chặn việc gì**

- `[DAEMON STDERR] [MOTOR] pyserial chua duoc cai` — `ros2_daemon_script.py` có nhánh code
  mở serial port, nhưng daemon chạy trên **laptop** còn STM32 cắm ở **Pi**. Thiếu thư viện
  nên nhánh đó bị bỏ qua, hiện vô hại. Khi nào lỗi này thực sự cản đường thì mới xử lý.

  🚨 **Nhưng tuyệt đối đừng `pip install pyserial`.** Cài vào là nhánh này sống dậy, cố mở
  một serial port không tồn tại trên laptop, và tranh quyền điều khiển motor với
  `pi_robot_node`. Cách xử lý đúng là **gỡ nhánh đó đi**, không phải cài thư viện cho nó chạy.

- `configs/learned_parameters.json` — tham số ACE tự học **từ Gazebo/TurtleBot3**, đang được
  nạp vào robot thật (`FRONTIER_PREFERENCE_WEIGHT 0.8 → 0.92`). Nên đổi tên file để ACE học
  lại từ đầu; nếu không, khó phân biệt hành vi nào do config mình đặt, hành vi nào do bài học
  môi trường cũ. Đáng làm trước khi bắt đầu tinh chỉnh nghiêm túc.

  ```bash
  cd ~/nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/configs
  mv learned_parameters.json learned_parameters.json.gazebo
  ```

- **API key lộ trong repo**: `docker/run_hybrid_container.sh` hardcode `NVIDIA_API_KEY`,
  `setup.md` chứa một GitHub PAT. Nếu repo từng push public thì revoke cả hai. Không liên
  quan tới robot nhưng để lâu thì quên.

**Tinh chỉnh khi cần**

- `FRONTIER_MIN_CLEARANCE = 1.0` và `ZONE_3_COMFORTABLE = 0.70` là số cho TurtleBot3 Waffle
  (rộng 0.28m) trong `turtlebot3_world` rộng rãi. Phòng thật chật hơn → agent có thể thấy mọi
  hướng đều bị chặn rồi loanh quanh gọi escape. Nếu log spam `escape`/`stuck`: hạ xuống
  0.6 và 0.5.
- `spatial_detector.py` có `recommended_direction = 'forward'` hardcode (`# Nav2 decides`).
  Tắt Nav2 rồi thì việc chọn hướng dồn hết sang `NavigationReasoner` + phân tích ảnh.
  Nếu robot chỉ biết đi thẳng rồi đâm và lùi, xem chỗ này chứ không phải lỗi cảm biến.
- `CRITICAL_ABORT_SIDE = 0.15` bằng đúng tầm đo tối thiểu của lidar. Nâng lên 0.18 sau khi
  đo bề ngang thật của khung xe (`WHEEL_BASE = 0.160` là khoảng cách hai bánh, không phải
  bề ngang xe).

**Cơ khí**

- Bánh trượt trên gạch. Băng dính điện chỉ là tạm. Theo thứ tự hiệu quả:
  bánh cao su gân > thêm tải lên trục bánh chủ động > ramp PWM trong `cmd_cb`
  (hiện set PWM tức thời 0 → 72, cú giật đó là lúc dễ trượt nhất).

**Tiện nghi**

- SSH key laptop → Pi, để khỏi gõ mật khẩu 3 lần mỗi lần khởi động:
  `ssh-copy-id pi@192.168.2.185`
- Vị trí lidar thật chưa verify — `slam_launch.py` giả định ở tâm robot, cao 5cm, không xoay.
