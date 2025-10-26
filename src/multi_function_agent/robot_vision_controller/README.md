# Robot Vision Controller with AI Navigation & Nav2 Integration

Hệ thống điều khiển robot TurtleBot3 tự động với AI Agent thông minh, tích hợp **Nav2 navigation stack** cho path planning an toàn. Robot có thể hiểu và thực hiện nhiệm vụ phức tạp từ natural language prompts như đếm vật thể, bám theo mục tiêu di động, tuần tra vòng tròn, và nhiều hơn nữa.

## 📋 Mục lục
- [Cấu trúc thư mục](#cấu-trúc-thư-mục)
- [Tổng quan hệ thống](#tổng-quan-hệ-thống)
- [Kiến trúc](#kiến-trúc)
- [Nav2 Integration](#nav2-integration)
- [Mission Types](#mission-types)
- [Yêu cầu hệ thống](#yêu-cầu-hệ-thống)
- [Cài đặt](#cài-đặt)
- [Cách chạy](#cách-chạy)
- [Ghi chú quan trọng](#ghi-chú-quan-trọng)

## 📁 Cấu trúc thư mục

```
multi_function_agent/
    ├── configs/
    │   └── config.yml                            # Cấu hình system + Nav2
    └── robot_vision_controller/
        ├── main.py                               # Entry point - HYBRID LOOP
        ├── core/
        │   ├── query_extractor.py                # Prompt information extraction
        │   ├── goal_parser.py                    # LLM mission parser 
        │   ├── mission_controller.py             # Mission state machine 
        │   └── models.py                         # Model management
        ├── navigation/
        │   ├── nav2_interface.py                 # Nav2 Python interface
        │   ├── navigation_reasoner.py            # Mission-aware manual control
        │   └── robot_controller_interface.py     # ROS/Gazebo interface + Nav2
        ├── perception/
        │   ├── lidar_monitor.py                  # Real-time collision avoidance
        │   ├── robot_vision_analyzer.py          # Vision + Object detection
        │   ├── spatial_detector.py               # LIDAR spatial analysis (simplified)
        │   └── rtsp_stream_handler.py            # RTSP stream handler        
        └── utils/
            ├── geometry_utils.py                 # Geometry calculation
            ├── movement_commands.py              # Commands to move
            ├── safety_checks.py                  # Safety First
            ├── ros_interface.py                  # ROS utilities
            └── log/
                ├── error_handlers.py             # Error logging
                ├── output_formatter.py           # Output logging
                └── performance_logger.py         # Performance logging

ros2_bridge_service/
└── robot_bridge_server.py                        # HTTP ↔ ROS2 bridge

turtlebot3_ws/
└── src/
    └── custom_controller/
        └── custom_controller/
            └── rtsp_publisher.py                 # RTSP stream publisher
```

---

---

## 🎯 Tổng quan hệ thống

Hệ thống bao gồm 5 thành phần chính:

1. **TurtleBot3 Simulation (Gazebo)**: Robot ảo với camera, lidar, odometry
2. **RTSP Streaming Pipeline**: Truyền video từ robot qua RTSP protocol
3. **Nav2 Stack**: Global path planning + local obstacle avoidance
4. **AI Agent (NEMO)**: Phân tích vision, mission control, và navigation orchestration
5. **LIDAR Safety Layer**: Real-time collision prevention với veto capability

### Workflow tổng quan - Hybrid Navigation
```
User Prompt → LLM Parser → Mission Controller
                                  ↓
                    ┌─────────────────────────────┐
                    │  MISSION STATE MACHINE      │
                    │  • Track progress           │
                    │  • Generate directives      │
                    └─────────────┬───────────────┘
                                  ↓
                    ┌─────────────────────────────┐
                    │  NAVIGATION DECISION        │
                    │  • Can use Nav2? → Goal     │
                    │  • Need manual? → Command   │
                    └─────────────┬───────────────┘
                                  ↓
         ┌────────────────────────┴─────────────────────────┐
         ▼ Nav2 Path                            Manual Path ▼
┌─────────────────────┐                  ┌──────────────────────┐
│  NAV2 EXECUTION     │                  │  MANUAL EXECUTION    │
│  • Global planning  │                  │  • Direct cmd_vel    │
│  • Local avoidance  │                  │  • LIDAR veto        │
│  • Recovery         │                  │  • Safety scaling    │
└─────────┬───────────┘                  └──────────┬───────────┘
          └────────────────┬────────────────────────┘
                           ↓
                  Robot executes safely
```

---

## 🏗️ Kiến trúc

### Sơ đồ thành phần chi tiết

```
┌─────────────────────────────────────────────────────────────┐
│         GAZEBO SIMULATION (TurtleBot3)                      │
│  • Camera Sensor (/camera_sensor) - 640x480 @ 15fps         │
│  • Lidar (/scan) - 360° laser scanner, 0.12-3.5m range      │
│  • Odometry (/odom) - Position, velocity feedback           │
└────────────┬────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────┐
│         RTSP STREAMING LAYER                                │
│  ┌────────────────┐         ┌──────────────┐                │
│  │ rtsp_publisher │────────▶│   MediaMTX   │                │
│  │   (ROS2 Node)  │  UDP    │ RTSP Server  │                │
│  └────────────────┘  :9000  └──────────────┘                │
│                         rtsp://127.0.0.1:8554/robotcam      │
└────────────┬────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────┐
│         NAV2 NAVIGATION STACK                               │
│  ┌──────────────────────────────────────────────────┐       │
│  │  SLAM & Mapping (slam_toolbox)                   │       │
│  │  • Real-time map building                        │       │
│  │  • Localization (AMCL)                           │       │
│  └──────────────────────────────────────────────────┘       │
│  ┌──────────────────────────────────────────────────┐       │
│  │  Costmap 2D (Layered)                            │       │
│  │  • Global costmap: Static map + inflation        │       │
│  │  • Local costmap: LIDAR obstacles (5m radius)    │       │
│  │  • Update rate: 5Hz (global), 10Hz (local)       │       │
│  └──────────────────────────────────────────────────┘       │
│  ┌──────────────────────────────────────────────────┐       │
│  │  Planners                                        │       │
│  │  • Global: NavFn (Dijkstra/A*)                   │       │
│  │  • Local: DWA (Dynamic Window Approach)          │       │
│  └──────────────────────────────────────────────────┘       │
│  ┌──────────────────────────────────────────────────┐       │
│  │  Recovery Behaviors                              │       │
│  │  • Rotate recovery (clear costmap)               │       │
│  │  • Backup and retry                              │       │
│  └──────────────────────────────────────────────────┘       │
│                                                             │
│  Action Server: /navigate_to_pose (NavigateToPose)          │
└────────────┬────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────┐
│         AI AGENT - HYBRID CONTROL LOOP                      │
│                                                             │
│  ┌──────────────────────────────────────────────────┐       │
│  │  ITERATION START (2-10Hz adaptive)               │       │
│  └────────────┬─────────────────────────────────────┘       │
│               ▼                                             │
│  ┌──────────────────────────────────────────────────┐       │
│  │  PRIORITY 0: LIDAR SAFETY VETO                   │       │
│  │  • Check collision risk (<0.3m)                  │       │
│  │  • Can abort Nav2 or manual commands             │       │
│  │  • Blocking escape if critical                   │       │
│  └────────────┬─────────────────────────────────────┘       │
│               │ VETO → Execute Escape → GOTO START          │
│               ▼ SAFE → Continue                             │
│  ┌──────────────────────────────────────────────────┐       │
│  │  PRIORITY 1: VISION ANALYSIS                     │       │
│  │  • YOLO Object Detection (YOLOv11n)              │       │
│  │  • Spatial Analysis (LIDAR-based)                │       │
│  │  • Target Tracking & Distance Estimation         │       │
│  └────────────┬─────────────────────────────────────┘       │
│               ▼                                             │
│  ┌──────────────────────────────────────────────────┐       │
│  │  PRIORITY 2: MISSION STATE UPDATE                │       │
│  │  • Update progress (count, tracking, laps)       │       │
│  │  • Check completion conditions                   │       │
│  │  • Generate mission directive                    │       │
│  └────────────┬─────────────────────────────────────┘       │
│               ▼                                             │
│  ┌──────────────────────────────────────────────────┐       │
│  │  PRIORITY 3: NAVIGATION STRATEGY                 │       │
│  │  ┌────────────────────────────────────────────┐  │       │
│  │  │ Can convert directive to Nav2 goal?        │  │       │
│  │  │  YES: explore, patrol, track (stable)      │  │       │
│  │  │  NO: backup, spin, precise maneuvers       │  │       │
│  │  └────────────┬───────────────────────────────┘  │       │
│  │               ▼                                  │       │
│  │  ┌────────────────────┬─────────────────────┐    │       │
│  │  ▼ USE NAV2           ▼ USE MANUAL          │    │       │
│  └──┼────────────────────┼─────────────────────┼────┘       │
│     ▼                    ▼                     ▼            │
│  ┌──────────────┐  ┌───────────────┐  ┌──────────────┐      │
│  │ Nav2 Goal    │  │ Manual Cmd    │  │ Safety Check │      │
│  │ • (x,y,θ)    │  │ • Twist msg   │  │ • LIDAR scan │      │
│  │ • Non-block  │  │ • Duration    │  │ • Abort Nav2 │      │
│  └──────┬───────┘  └───────┬───────┘  └──────┬───────┘      │
│         │                  │                 │              │
│         │ Nav2 handles     │ Direct control  │ Continuous   │
│         │ everything       │ with monitoring │ monitoring   │
│         ▼                  ▼                 ▼              │
│  ┌──────────────────────────────────────────────────┐       │
│  │  EXECUTION LAYER                                 │       │
│  │  • Nav2: Smooth path following                   │       │
│  │  • Manual: 20Hz safety-monitored cmd_vel         │       │
│  │  • Both: Can be aborted by LIDAR veto            │       │
│  └──────────────────────────────────────────────────┘       │
└───────────────┼─────────────────────────────────────────────┘
                │
                ▼ (Nav2: action msgs, Manual: /cmd_vel)
┌─────────────────────────────────────────────────────────────┐
│         ROS2 TOPICS & SERVICES                              │
│  • /cmd_vel (Twist) - Direct velocity commands              │
│  • /navigate_to_pose (Action) - Nav2 goal interface         │
│  • /scan (LaserScan) - LIDAR data for safety                │
│  • /odom (Odometry) - Position feedback                     │
│  • /map (OccupancyGrid) - SLAM map                          │
└───────────────┬─────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────┐
│         Back to Gazebo Robot                                │
│  • Robot executes movement                                  │
│  • Updates camera, lidar, odom                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 🗺️ Nav2 Integration

### **Lợi ích của Nav2**

#### **1. Proactive Navigation**
- **Trước (Reactive LIDAR):** Robot chỉ phản ứng khi gần vật cản
- **Sau (Nav2):** Robot biết map, plan đường trước, tránh chướng ngại vật sớm

#### **2. Global Path Planning**
- Dijkstra/A* algorithm trên map
- Tìm đường tối ưu từ A → B
- Tránh vùng nguy hiểm trên costmap

#### **3. Local Obstacle Avoidance**
- DWA (Dynamic Window Approach)
- Real-time trajectory adjustment
- Tránh chướng ngại vật động

#### **4. Recovery Behaviors**
- Tự động thoát khi bị stuck
- Rotate → Clear costmap → Retry
- Backup và tìm đường khác

#### **5. Code Reduction**
- Xóa ~100 lines path planning logic
- Nav2 thay thế `spatial_detector` direction finding
- Chỉ giữ mission-specific behaviors

### **Khi nào dùng Nav2 vs Manual**

|      Directive      | Nav2 | Manual |             Lý do            |
|---------------------|------|--------|------------------------------|
| `explore_random`    |  ✅  |        |       Random goals on map    |
| `patrol_circle`     |  ✅  |        |  Arc navigation with goals   |
| `track_follow`      |  ⚠️  |   ✅   | Target di động, cần reactive |
| `track_backup`      |      |   ✅   |   Precise distance control   |
| `track_search_spin` |      |   ✅   |     360° rotation in place   |
| `track_approach`    |  ✅  |        |     Goal-based approach      |

---

## 🎮 Mission Types

Robot hỗ trợ 3 loại nhiệm vụ thông qua natural language:

```
**Navigation:** Nav2 exploration + object detection  
**Behavior:** Robot explore và đếm objects, dừng khi đủ số lượng

### 1. **Follow Target** (Bám theo mục tiêu - Dog-like)
```bash
"Theo sau người đang đi"
"Follow the person"
"Đi theo mục tiêu di động"
```
**Navigation:** Hybrid (Nav2 approach + manual tracking)  
**Behavior:** 
- Track target at safe distance (1.0-2.5m)
- Follow when target moves
- **Predict behavior** when lost: continue in last known direction
- Search pattern if lost >3s

### 2. **Patrol Laps** (Tuần tra vòng)
```bash
"Đi 20 vòng tròn"
"Patrol 5 laps"
"Tuần tra 10 vòng"
```
**Navigation:** Nav2 arc goals  
**Behavior:** Complete N circular laps, return to start position

### 3. **Explore Area** (Khám phá)
```bash
"Khám phá tự do"
"Explore the environment"
"Run wide automatically in 60 seconds"
```
**Navigation:** Nav2 random goals  
**Behavior:** Random waypoint generation on map, smooth navigation

---

## 💻 Yêu cầu hệ thống

### Phần mềm
- **Ubuntu 22.04 LTS** (khuyến nghị)
- **ROS2 Humble** 
- **Python 3.10+**
- **Gazebo 11**

### Thư viện Python chính
- `ultralytics` (YOLOv11n)
- `opencv-python`
- `flask`
- `numpy`
- `pyyaml`
- `torch` (for YOLO)
- `transforms3d` (for Nav2 quaternion)

### ROS2 Packages
- `ros-humble-turtlebot3*`
- `ros-humble-navigation2`
- `ros-humble-slam-toolbox`
- `ros-humble-nav2-bringup`

### Tools
- **MediaMTX**: RTSP streaming server
- **FFmpeg**: Video encoding/decoding
- **gnome-terminal**: Để chạy script tự động

### Phần cứng khuyến nghị
- RAM: 8GB+
- GPU: Optional (YOLO inference faster with CUDA)
- CPU: 4+ cores

---

## 🔧 Cài đặt

### 1. Cài đặt ROS2 và TurtleBot3

```bash
# Cài ROS2 Humble
# Follow: https://docs.ros.org/en/humble/Installation.html

# Cài TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Cài Nav2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Cài SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 2. Setup Workspace ROS2

```bash
# Tạo workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone custom controller (nếu có repo)
# hoặc copy thư mục custom_controller vào đây

# Build workspace
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

### 3. Cài đặt MediaMTX

```bash
cd ~
# Download từ: https://github.com/bluenviron/mediamtx/releases
wget https://github.com/bluenviron/mediamtx/releases/download/v1.8.0/mediamtx_v1.8.0_linux_amd64.tar.gz
tar -xzf mediamtx_v1.8.0_linux_amd64.tar.gz
chmod +x mediamtx
```

### 4. Setup ROS2 Bridge Service

```bash
cd ~
mkdir ros2_bridge_service
cd ros2_bridge_service

# Tạo virtual environment
python3 -m venv ros_env
source ros_env/bin/activate

# Cài dependencies
pip install flask rclpy geometry-msgs
```

### 5. Cài đặt NEMO Agent Toolkit

```bash
cd ~
git clone <your-repo-url> nemo-agent-toolkit

cd nemo-agent-toolkit
python3 -m venv .venv
source .venv/bin/activate

# Cài dependencies
pip install ultralytics opencv-python pyyaml numpy torch transforms3d
```

### 6. Tạo Map với SLAM

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3: Launch RViz
rviz2

# Terminal 4: Teleop để khám phá (tạo map)
ros2 run turtlebot3_teleop teleop_keyboard

# Sau khi map đủ, save map (Terminal 5)
cd ~
ros2 run nav2_map_server map_saver_cli -f turtlebot3_nav2_map
```

### 7. Copy Project Files

```bash
# Copy multi_function_agent vào nemo-agent-toolkit
cp -r multi_function_agent ~/nemo-agent-toolkit/examples/

# Copy robot_bridge_server.py
cp robot_bridge_server.py ~/ros2_bridge_service/

# Copy rtsp_publisher.py
cp rtsp_publisher.py ~/turtlebot3_ws/src/custom_controller/custom_controller/
```

---

## 🚀 Cách chạy

### Bước 1: Khởi động Robot + Nav2 Stack

Tạo script `start_robot_nav2.sh`:

```bash
#!/bin/bash
run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

echo "Starting robot + Nav2 stack..."

# Start Gazebo
run_in_terminal "cd ~ && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
sleep 5

# Start Nav2 with saved map
run_in_terminal "cd ~ && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml"
sleep 3

# Start MediaMTX
run_in_terminal "cd ~ && ./mediamtx"
sleep 2  

# Start RTSP publisher
run_in_terminal "cd ~/turtlebot3_ws && source install/setup.bash && cd src/custom_controller/custom_controller && python3 rtsp_publisher.py"
sleep 2

# Start bridge server
run_in_terminal "cd ~/ros2_bridge_service && source ros_env/bin/activate && python robot_bridge_server.py"

echo "All services started!"
echo "In RViz: Set initial pose with '2D Pose Estimate' tool"
```

Chạy script:
```bash
chmod +x start_robot_nav2.sh
./start_robot_nav2.sh
```

**QUAN TRỌNG:** Trong RViz, dùng "2D Pose Estimate" để set vị trí ban đầu của robot.

### Bước 2: Chạy AI Agent với Mission

```bash
cd ~/nemo-agent-toolkit
source .venv/bin/activate

# Example missions:

# Explore with Nav2
nat run --config_file configs/config.yml --input "Run wide automatically in 60 seconds in rtsp://127.0.0.1:8554/robotcam"

# Count objects
nat run --config_file configs/config.yml --input "Đếm 10 chai nước trong rtsp://127.0.0.1:8554/robotcam"

# Follow target (hybrid)
nat run --config_file configs/config.yml --input "Theo sau người đang đi trong rtsp://127.0.0.1:8554/robotcam"

# Patrol laps
nat run --config_file configs/config.yml --input "Đi 5 vòng tròn trong rtsp://127.0.0.1:8554/robotcam"
```

---

## 📝 Ghi chú quan trọng

### Hybrid Navigation Strategy

**Nav2 Usage (70-80% of time):**
- ✅ `explore_random`: Random waypoints on map
- ✅ `patrol_*`: Arc-based circular motion
- ✅ `track_approach`: Goal-based target approach
- ✅ Smooth, collision-free paths
- ✅ Auto recovery from stuck situations

**Manual Control (20-30% of time):**
- ✅ `track_follow`: Real-time target tracking
- ✅ `track_backup`: Precise reverse movements
- ✅ `track_search_spin`: 360° search rotation
- ✅ Nav2 fallback when goal rejected
- ✅ Emergency behaviors

### Safety Features

**Multi-Level Protection:**
- 🛡️ **Level 0 (Nav2 Costmap)**: Proactive path planning around obstacles
- 🛡️ **Level 1 (DWA Local Planner)**: Real-time trajectory adjustment
- 🛡️ **Level 2 (LIDAR Veto - Pre-Execution)**: Check before ANY command
- 🛡️ **Level 3 (During Execution)**: 20Hz monitoring while moving
- 🛡️ **Level 4 (Immediate Abort)**: Stop within 50ms at <0.3m
- 🛡️ **Level 5 (Progressive Reduction)**: Gradual slowdown at 0.3-0.5m

**Safety Guarantees:**
- ⚡ Response time: <50ms from detection to stop
- 🎯 Abort accuracy: 100% (blocking execution)
- 📊 Monitoring rate: 20Hz during movement
- 🔒 Override capability: LIDAR Safety ALWAYS wins over Nav2

### Mission System Features

- **LLM-Powered Parsing**: Natural language → Structured missions
- **State Machine**: Track progress for complex tasks
- **Completion Detection**: Auto-stop when mission done
- **Adaptive Navigation**: Hybrid Nav2/Manual based on directive
- **Object Detection**: YOLO integration for target recognition
- **Predictive Tracking**: Dog-like following behavior

### Limitations

**Technical Constraints:**
- **YOLO Classes**: Limited to COCO dataset (80 classes)
- **Distance Estimation**: Rough approximation from bbox size + LIDAR
- **Lap Detection**: Simple return-to-start logic (no SLAM loop closure)
- **LLM Dependency**: Mission parsing requires LLM access
- **Single Robot**: No multi-robot coordination yet
- **Map Dependency**: Nav2 requires pre-built map (SLAM phase needed)
