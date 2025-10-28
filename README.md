# Robot Vision Controller with AI Navigation & Nav2 Integration

Hệ thống điều khiển robot TurtleBot3 tự động với AI Agent thông minh, tích hợp **Nav2 navigation stack** cho path planning an toàn. Robot có thể hiểu và thực hiện nhiệm vụ phức tạp từ natural language prompts như đếm vật thể, bám theo mục tiêu di động, tuần tra vòng tròn, và nhiều hơn nữa.

## 🤖 AI Models Used

|         Model           |                     Purpose                        |        When Used        | Critical |
|-------------------------|----------------------------------------------------|-------------------------|----------|
| **LLM (Llama 3.1 70B)** | Parse natural language prompt → structured mission |       1x at startup     |  ✅ Yes  |
|    **YOLO (v11n)**      |            Object detection & tracking             | Continuous (2Hz cached) |  ✅ Yes  |

**Performance:**
- 🚀 Real-time navigation: <100ms per iteration
- 💾 Memory usage: ~0.5GB (YOLO only)
- ⚡ Startup time: ~1 second

---

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
        │   └── models.py                         # YOLO model management
        ├── navigation/
        │   ├── nav2_interface.py                 # Nav2 Python interface
        │   ├── navigation_reasoner.py            # Mission-aware manual control
        │   └── robot_controller_interface.py     # ROS/Gazebo interface + Nav2
        ├── perception/
        │   ├── lidar_monitor.py                  # Real-time collision avoidance
        │   ├── robot_vision_analyzer.py          # YOLO + LIDAR spatial analysis
        │   ├── spatial_detector.py               # LIDAR spatial analysis
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

docker/   
    ├── docker-compose.yml                        # Multi-container orchestration
    ├── Dockerfile.ros2                           # ROS2 + Nav2 + Gazebo container
    ├── Dockerfile.nat                            # NAT + AI Agent container
    ├── run.sh                                    # Main launcher script
    ├── stop.sh                                   # Graceful shutdown
    ├── status.sh                                 # Health monitoring
    ├── Makefile                                  # Convenience commands
    └── scripts/
        └── ros2_entrypoint.sh                    # ROS2 service startup

ros2_bridge_service/
└── robot_bridge_server.py                        # HTTP ↔ ROS2 bridge

turtlebot3_ws/
└── src/
    └── custom_controller/
        └── custom_controller/
            └── rtsp_publisher.py                 # RTSP stream publisher
```

---

## 🎯 Tổng quan hệ thống

Hệ thống được thiết kế theo **kiến trúc Docker Multi-Container**, tách biệt hoàn toàn giữa AI Agent và ROS2 stack, đồng thời cho phép giao tiếp real-time qua ROS2 DDS network.

### **Thành phần chính:**

#### **1. ROS2-Nav2 Container** (Python 3.10)
```
┌─────────────────────────────────────────────┐
│   ROS2 Humble + Nav2 + Gazebo Container     │
├─────────────────────────────────────────────┤
│  • TurtleBot3 Simulation (Gazebo)           │
│  • Nav2 Navigation Stack                    │
│    - Global Planner (Dijkstra/A*)           │
│    - Local Planner (DWA)                    │
│    - Costmap (Obstacle inflation)           │
│    - Recovery Behaviors                     │
│  • SLAM Toolbox (Real-time mapping)         │
│  • MediaMTX (RTSP streaming)                │
│  • Bridge Server (HTTP ↔ ROS2)              │
│  • LIDAR Scanner (360° safety)              │
└─────────────────────────────────────────────┘
```

**Exposed Services:**
- `localhost:11345` - Gazebo GUI
- `localhost:8554` - RTSP stream (rtsp://localhost:8554/robotcam)
- `localhost:8080` - Bridge API (HTTP fallback)
- ROS2 Topics: `/cmd_vel`, `/scan`, `/odom`, `/map`, `/plan`

---

#### **2. NAT-Agent Container** (Python 3.11)
```
┌─────────────────────────────────────────────┐
│     NVIDIA NAT + AI Agent Container         │
├─────────────────────────────────────────────┤
│  • LLM Parser (Llama 3.1 70B)               │
│    - Natural language → Mission structure   │
│    - 1x at startup only                     │
│  • YOLO Object Detection (v11n)             │
│    - 80 COCO classes                        │
│    - 2Hz cached inference                   │
│  • Mission Controller                       │
│    - State machine for mission tracking     │
│    - Progress monitoring                    │
│  • Navigation Reasoner                      │
│    - Hybrid Nav2/Manual decision logic      │
│  • Vision Analyzer                          │
│    - YOLO + LIDAR fusion                    │
│    - Spatial awareness                      │
│  • ROS2 Client Libraries                    │
│    - rclpy for native ROS2 communication    │
│    - Nav2 action client                     │
└─────────────────────────────────────────────┘
```

**Key Features:**
- Native ROS2 integration (not bridge-only)
- Direct Nav2 action client for goal sending
- YOLO-only pipeline (BLIP2 removed)
- Mission-driven autonomous behavior

---

### **Workflow tổng quan - Hybrid Docker Architecture**
```
┌─────────────────────────────────────────────────────────────────────────┐
│                          HOST SYSTEM                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  User Input: "Run wide automatically in 60 seconds"                     │
│       │                                                                 │
│       └──────────────────────────────────────────┐                      │
│                                                  ▼                      │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  NAT-Agent Container (Python 3.11)                                │  │
│  │  ┌─────────────────────────────────────────────────────────────┐  │  │
│  │  │  1. LLM Parser (1x startup)                                 │  │  │
│  │  │     Input → Mission Structure                               │  │  │
│  │  └─────────────────────────────────────────────────────────────┘  │  │
│  │  ┌─────────────────────────────────────────────────────────────┐  │  │
│  │  │  2. Main Control Loop (10Hz)                                │  │  │
│  │  │     ┌────────────────────────────────────────────────────┐  │  │  │
│  │  │     │ 2a. Vision Analysis (YOLO 2Hz cached)              │  │  │  │
│  │  │     │     • Object detection                             │  │  │  │
│  │  │     │     • LIDAR spatial analysis                       │  │  │  │
│  │  │     └────────────────────────────────────────────────────┘  │  │  │
│  │  │     ┌────────────────────────────────────────────────────┐  │  │  │
│  │  │     │ 2b. Mission State Update                           │  │  │  │
│  │  │     │     • Progress tracking                            │  │  │  │
│  │  │     │     • Generate directive                           │  │  │  │
│  │  │     └────────────────────────────────────────────────────┘  │  │  │
│  │  │     ┌────────────────────────────────────────────────────┐  │  │  │
│  │  │     │ 2c. Navigation Decision (Hybrid)                   │  │  │  │
│  │  │     │     • Can use Nav2? → Send goal via ROS2 action    │  │  │  │
│  │  │     │     • Need manual? → Generate cmd_vel              │  │  │  │
│  │  │     └────────────────────────────────────────────────────┘  │  │  │
│  │  └─────────────────────────────────────────────────────────────┘  │  │
│  │                                                                   │  │
│  │  ROS2 Client (rclpy) ──────────────────────────────┐              │  │
│  └────────────────────────────────────────────────────│──────────────┘  │
│                                                       │                 │
│                              ROS2 DDS Network (Domain ID: 42)           │
│                                                       │                 │
│  ┌────────────────────────────────────────────────────│────────────────┐│
│  │  ROS2-Nav2 Container (Python 3.10)                 ▼                ││
│  │  ┌─────────────────────────────────────────────────────────────┐    ││
│  │  │  Gazebo Simulation                                          │    ││
│  │  │    • TurtleBot3 Waffle (camera + lidar)                     │    ││
│  │  │    • Physics engine                                         │    ││
│  │  └─────────────────────────────────────────────────────────────┘    ││
│  │  ┌─────────────────────────────────────────────────────────────┐    ││
│  │  │  Nav2 Stack                                                 │    ││
│  │  │    • /navigate_to_pose action server ◄── Receives goals     │    ││
│  │  │    • Global Planner: A* on costmap                          │    ││
│  │  │    • Local Planner: DWA real-time obstacles                 │    ││
│  │  │    • Publishes: /plan, /cmd_vel                             │    ││
│  │  └─────────────────────────────────────────────────────────────┘    ││
│  │  ┌─────────────────────────────────────────────────────────────┐    ││
│  │  │  Sensor Pipeline                                            │    ││
│  │  │    • Camera → MediaMTX → RTSP stream                        │    ││
│  │  │    • LIDAR → /scan topic (20Hz)                             │    ││
│  │  │    • Odometry → /odom topic                                 │    ││
│  │  └─────────────────────────────────────────────────────────────┘    ││
│  │  ┌─────────────────────────────────────────────────────────────┐    ││
│  │  │  Bridge Server (Fallback HTTP API)                          │    ││
│  │  │    • POST /robot/command → /cmd_vel                         │    ││
│  │  │    • GET /robot/lidar → LIDAR data                          │    ││
│  │  └─────────────────────────────────────────────────────────────┘    ││
│  └─────────────────────────────────────────────────────────────────────┘│
│                                                                         │
│                          Robot executes safely                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

### **Communication Flow:**

#### **Primary: Native ROS2 (Recommended)**
```
NAT Container ─(rclpy)─► ROS2 DDS Network ◄─(rclpy)─ ROS2 Container
     │                                                      │
     ├─ Publishes: /cmd_vel (manual control)                │
     ├─ Calls: /navigate_to_pose (Nav2 goals) ─────────────►│
     ├─ Subscribes: /scan (LIDAR)  ◄────────────────────────┤
     └─ Subscribes: /odom (position) ◄──────────────────────┘
```

#### **Fallback: HTTP Bridge (If ROS2 fails)**
```
NAT Container ─(HTTP)─► Bridge Server ─(ROS2)─► Robot
     POST /robot/command → /cmd_vel
     GET /robot/lidar    ← /scan
```

---

## 🏗️ Kiến trúc

### AI Pipeline (Optimized)

```
┌──────────────────────────────────────────────────────────┐
│  STARTUP PHASE (1 second)                                │
├──────────────────────────────────────────────────────────┤
│  1. Load YOLO model (0.1s)                               │
│  2. User prompt → LLM parse (3s)                         │
│  3. Initialize Nav2 + ROS2 (1s)                          │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  MAIN LOOP (10 Hz - 100ms per iteration)                 │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  Priority 0: LIDAR Safety Check (10ms)                   │
│  ├─ Min distance check                                   │
│  └─ Veto Nav2/Manual if critical                         │
│                                                          │
│  Priority 1: Vision Analysis (50ms, cached 0.5s)         │
│  ├─ YOLO object detection (if mission needs)             │
│  ├─ LIDAR spatial analysis                               │
│  └─ Target tracking (bbox center + distance)             │
│                                                          │
│  Priority 2: Mission State Update (5ms)                  │
│  ├─ Count progress / Lap tracking                        │
│  ├─ Target visibility check                              │
│  └─ Generate directive (explore/track/patrol)            │
│                                                          │
│  Priority 3: Navigation Decision (5ms)                   │
│  ├─ Can use Nav2? → Send goal (non-blocking)             │
│  └─ Need manual? → Generate cmd_vel                      │
│                                                          │
│  Priority 4: Execute Command (20ms)                      │
│  ├─ Publish to /cmd_vel or Nav2 action                   │
│  └─ Monitor safety during execution                      │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│         GAZEBO SIMULATION (TurtleBot3)                      │
│  • Camera: 640x480 @ 15fps                                  │
│  • Lidar: 360° laser, 0.12-3.5m range                       │
│  • Odometry: Position + velocity                            │
└────────────┬────────────────────────────────────────────────┘
             ▼
┌─────────────────────────────────────────────────────────────┐
│         RTSP STREAMING                                      │
│  Camera → rtsp_publisher → MediaMTX → rtsp://...            │
└────────────┬────────────────────────────────────────────────┘
             ▼
┌─────────────────────────────────────────────────────────────┐
│         NAV2 STACK                                          │
│  • SLAM: Real-time mapping                                  │
│  • Costmap: Obstacle inflation                              │
│  • Planners: A* global + DWA local                          │
│  • Recovery: Rotate, backup behaviors                       │
└────────────┬────────────────────────────────────────────────┘
             ▼
┌─────────────────────────────────────────────────────────────┐
│         AI AGENT (YOLO + LIDAR)                             │
│  ┌───────────────────────────────────────────┐              │
│  │ YOLO Object Detection (50ms)              │              │
│  │  • Person, bottle, cup, chair, etc.       │              │
│  │  • Bbox, center, distance estimation      │              │
│  └───────────────────────────────────────────┘              │
│  ┌───────────────────────────────────────────┐              │
│  │ LIDAR Spatial Analysis (10ms)             │              │
│  │  • Safety score calculation               │              │
│  │  • Clear path detection                   │              │
│  │  • Obstacle distance mapping              │              │
│  └───────────────────────────────────────────┘              │
│  ┌───────────────────────────────────────────┐              │
│  │ Navigation Reasoner (5ms)                 │              │
│  │  • Mission directive execution            │              │
│  │  • Hybrid Nav2/Manual selection           │              │
│  └───────────────────────────────────────────┘              │
└────────────┬────────────────────────────────────────────────┘
             ▼
┌─────────────────────────────────────────────────────────────┐
│         ROS2 INTERFACE                                      │
│  • /cmd_vel: Direct velocity control                        │
│  • /navigate_to_pose: Nav2 goals                            │
│  • /scan: LIDAR input                                       │
│  • /odom: Position feedback                                 │
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
- Simplified vision pipeline (YOLO + LIDAR only)
- Nav2 handles complex path planning
- Focus on mission-specific behaviors

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

### 0. **Count Objects** (Đếm vật thể)
```bash
"Đếm 10 chai nước"
"Count 5 cups"
"Tìm 3 người"
```
**Navigation:** Nav2 exploration + YOLO detection  
**Behavior:** Explore và đếm objects, dừng khi đủ số lượng

### 1. **Follow Target** (Bám theo mục tiêu)
```bash
"Theo sau người đang đi"
"Follow the person"
"Đi theo mục tiêu di động"
```
**Navigation:** Hybrid (Nav2 approach + manual tracking)  
**Behavior:** 
- Track target at safe distance (1.0-2.5m)
- YOLO bbox tracking với real-time adjustment
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
- `ultralytics` (YOLOv11n) - Object detection
- `opencv-python` - Image processing
- `flask` - ROS2 bridge server
- `numpy` - Math operations
- `pyyaml` - Config parsing
- `torch` - YOLO backend
- `transforms3d` - Nav2 quaternion conversion

### ROS2 Packages
- `ros-humble-turtlebot3*`
- `ros-humble-navigation2`
- `ros-humble-slam-toolbox`
- `ros-humble-nav2-bringup`

### Tools
- **MediaMTX**: RTSP streaming server
- **FFmpeg**: Video encoding/decoding
- **gnome-terminal**: Script automation

### Phần cứng khuyến nghị
- **RAM**: 8GB+ (YOLO + Nav2)
- **GPU**: Optional (YOLO faster with CUDA)
- **CPU**: 4+ cores

**Performance Notes:**
- CPU-only: ~50ms YOLO inference
- With GPU: ~20ms YOLO inference
- Memory: 0.5GB (YOLO) + 1GB (Nav2) = ~2GB total

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

# Cài dependencies (YOLO only, no BLIP2)
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
run_in_terminal "cd ~ && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/turtlebot3_nav2_map.yaml"
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
nat run --config_file examples/multi_function_agent/configs/config.yml --input "Run wide automatically in 60 seconds"

# Count objects (YOLO)
nat run --config_file examples/multi_function_agent/configs/config.yml --input "Đếm 10 chai nước"

# Follow target (Hybrid Nav2 + YOLO)
nat run --config_file examples/multi_function_agent/configs/config.yml --input "Theo sau người đang đi"

# Patrol laps (Nav2)
nat run --config_file examples/multi_function_agent/configs/config.yml --input "Đi 5 vòng tròn"
```

---

## 📝 Ghi chú quan trọng

### Hybrid Navigation Strategy

**Nav2 Usage (70-80% of time):**
- ✅ `explore_random`: Random waypoints on the map
- ✅ `patrol_*`: Arc-based circular motion
- ✅ `track_approach`: Goal-based target approach
- ✅ Smooth, collision-free paths
- ✅ Auto recovery from stuck situations

**Manual Control (20-30% of time):**
- ✅ `track_follow`: Real-time YOLO bbox tracking
- ✅ `track_backup`: Precise reverse movements
- ✅ `track_search_spin`: 360° search rotation
- ✅ Nav2 fallback when goal rejected
- ✅ Emergency behaviors

### Safety Features

**Multi-Level Protection:**
- 🛡️ **Level 0 (Nav2 Costmap)**: Proactive path planning around obstacles
- 🛡️ **Level 1 (DWA Local Planner)**: Real-time trajectory adjustment
- 🛡️ **Level 2 (LIDAR Veto)**: Pre-execution safety check
- 🛡️ **Level 3 (20Hz Monitor)**: Continuous safety during movement
- 🛡️ **Level 4 (Immediate Abort)**: <50ms stop at critical distance
- 🛡️ **Level 5 (Progressive Scale)**: Speed reduction near obstacles

**Safety Guarantees:**
- ⚡ Response time: <50ms from detection to stop
- 🎯 Abort accuracy: 100% (blocking execution)
- 📊 Monitoring rate: 20Hz during movement
- 🔒 Override capability: LIDAR Safety > Nav2 > Manual

### AI Pipeline Features

- **LLM-Powered Parsing**: Natural language → Structured missions (1x startup)
- **YOLO Detection**: 80 COCO classes, 50ms inference, 2Hz cached
- **State Machine**: Mission progress tracking
- **Completion Detection**: Auto-stop when goal achieved
- **Adaptive Navigation**: Hybrid Nav2/Manual based on directive
- **Real-time Tracking**: YOLO bbox center + distance estimation

### Limitations

**Technical Constraints:**
- **YOLO Classes**: Limited to 80 COCO classes
- **Distance Accuracy**: LiDAR-fused (±5cm), fallback heuristic when out of LiDAR range (0.12-3.5m)
- **Lap Detection**: Simple odometry-based (no SLAM loop closure)
- **LLM Dependency**: Requires NIM API for prompt parsing
- **Single Robot**: No multi-robot coordination
- **Map Dependency**: Nav2 requires pre-built SLAM map

**Known Issues:**
- Camera resolution fixed at 640x480
- YOLO confidence threshold: 0.5 (adjustable)
- Nav2 goal tolerance: 0.2m position, 0.1rad orientation
- Bridge mode: No Nav2 support (requires native ROS2)