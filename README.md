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

![Workflow Diagram](src/multi_function_agent/robot_vision_controller/images/nat_container.png)

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

#### **5. Code Reduction & Reliability**
- Simplified vision pipeline (YOLO + LIDAR only)
- Nav2 handles complex path planning
- Focus on mission-specific behaviors
- Battle-tested navigation algorithms

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

Robot hỗ trợ 4 loại nhiệm vụ thông qua natural language:

### **1. Count Objects** (Đếm vật thể)
```bash
"Đếm 10 chai nước"
"Count 5 cups"
"Tìm 3 người"
```
**Navigation:** Nav2 exploration + YOLO detection  
**Behavior:** 
- Explore environment và đếm objects
- Dừng khi đủ số lượng
- Track progress: current_count / target_count

**Example Output:**
```
[MISSION] Count 10 bottles
Progress: 3/10 bottles detected
Directive: explore_random (searching for more)
```

---

### **2. Follow Target** (Bám theo mục tiêu)
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
- Recovery: rotate, explore, approach

**State Machine:**
```
TRACKING → target visible, distance OK
APPROACHING → target far, move closer
BACKING_UP → target too close (<1.0m)
SEARCHING → target lost, scan environment
```

---

### **3. Patrol Laps** (Tuần tra vòng)
```bash
"Đi 20 vòng tròn"
"Patrol 5 laps"
"Tuần tra 10 vòng"
```
**Navigation:** Nav2 arc goals  
**Behavior:** 
- Complete N circular laps
- Arc-based waypoint generation
- Return to start position after completion
- Track progress: current_lap / target_laps

**Implementation:**
```python
# Generate arc waypoint
angle_offset = 0.3 rad  # 17 degrees
distance = 1.0m
goal = (x + d*cos(θ+offset), y + d*sin(θ+offset), θ+offset)
```

---

### **4. Explore Area** (Khám phá)
```bash
"Khám phá tự do"
"Explore the environment"
"Run wide automatically in 60 seconds"
```
**Navigation:** Nav2 random goals  
**Behavior:** 
- Random waypoint generation on map
- Smooth navigation with obstacle avoidance
- Time-based or continuous exploration
- Coverage maximization

**Parameters:**
- `duration`: Time limit (seconds)
- `coverage`: "full" | "partial" | "random"

---

## 💻 Yêu cầu hệ thống

### **Phần mềm bắt buộc**
- **Ubuntu 22.04 LTS** (khuyến nghị)
- **Docker** 20.10+ with Docker Compose V2
- **NVIDIA GPU** (optional, for faster YOLO)

### **Phần cứng khuyến nghị**
- **RAM**: 8GB+ (Docker containers: ~3GB)
- **GPU**: NVIDIA with CUDA support (optional)
- **CPU**: 4+ cores
- **Disk**: 20GB free space (Docker images)

### **Dependencies tự động cài qua Docker:**
- ROS2 Humble
- Nav2 navigation stack
- TurtleBot3 packages
- YOLO model
- Python libraries

---

## 🔧 Cài đặt

### **Bước 1: Cài đặt Docker**

```bash
# Cài Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group (no sudo needed)
sudo usermod -aG docker $USER
newgrp docker

# Verify installation
docker --version
docker compose version
```

### **Bước 2: Enable X11 forwarding (cho Gazebo GUI)**

```bash
# Install xhost
sudo apt-get update
sudo apt-get install x11-xserver-utils

# Allow Docker to access X server
xhost +local:docker
```

### **Bước 3: Clone Repository**

```bash
cd ~
git clone https://github.com/jerrynguy/final-project.git nemo-agent-toolkit
cd nemo-agent-toolkit
```

### **Bước 4: Tạo Map (chỉ cần 1 lần)**

**Option A: Tạo map mới với SLAM**

```bash
# Terminal 1: Launch Gazebo (native ROS2, không dùng Docker)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3: Launch RViz
rviz2

# Terminal 4: Teleop để khám phá (tạo map)
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 5: Save map khi đủ
cd ~
ros2 run nav2_map_server map_saver_cli -f my_map
# Tạo ra: my_map.yaml và my_map.pgm
```

**Option B: Dùng map có sẵn**

Nếu bạn đã có map, đảm bảo files nằm ở `~/my_map.yaml` và `~/my_map.pgm`.

### **Bước 5: Build Docker Images**

```bash
cd ~/nemo-agent-toolkit/docker

# Make scripts executable
chmod +x *.sh
chmod +x scripts/*.sh

# Build images (5-10 phút lần đầu)
./run.sh
```

Lệnh này sẽ:
1. Build ROS2-Nav2 container
2. Build NAT-Agent container
3. Start all services
4. Wait for health checks
5. Display status

---

## 🚀 Cách chạy

### **Quick Start (Recommended)**

```bash
cd ~/nemo-agent-toolkit/docker

# Start entire stack
./run.sh
```

Script tự động:
- ✅ Pre-flight checks (Docker, X11, maps)
- ✅ Build images if needed
- ✅ Start containers
- ✅ Wait for services ready
- ✅ Display status and commands

### **Chạy NAT Agent với Mission**

Sau khi `./run.sh` hoàn tất:

```bash
# Enter NAT container
docker exec -it nat_agent_container bash

# Run mission (inside container)
nat run --config_file /workspace/configs/config.yml --input "YOUR_MISSION_HERE"
```

**Example Missions:**

```bash
# Explore với Nav2
nat run --config_file /workspace/configs/config.yml --input "Run wide automatically in 60 seconds"

# Count objects (YOLO)
nat run --config_file /workspace/configs/config.yml --input "Đếm 10 chai nước"

# Follow target (Hybrid Nav2 + YOLO)
nat run --config_file /workspace/configs/config.yml --input "Theo sau người đang đi"

# Patrol laps (Nav2)
nat run --config_file /workspace/configs/config.yml --input "Đi 5 vòng tròn"
```

### **Set Initial Pose in RViz (QUAN TRỌNG)**

Nav2 cần biết vị trí ban đầu của robot:

```bash
# Enter ROS2 container
docker exec -it ros2_nav2_container bash

# Launch RViz
rviz2

# Trong RViz:
# 1. Click "2D Pose Estimate" tool
# 2. Click vào vị trí robot trên map
# 3. Drag để set hướng
```

### **Monitor System**

```bash
# Check status
./status.sh

# View logs
docker compose logs -f ros2-nav2
docker compose logs -f nat-agent

# Enter containers
docker exec -it ros2_nav2_container bash
docker exec -it nat_agent_container bash

# Check ROS2 topics
docker exec -it ros2_nav2_container bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 node list
```

### **Stop System**

```bash
# Graceful shutdown
./stop.sh

# Or force stop
docker compose down

# Clean everything (images, volumes)
docker compose down -v --rmi all
```

---

## 📝 Ghi chú quan trọng

### **Docker Multi-Container Benefits**

✅ **Isolated Environments:**
- Python 3.11 (NAT) + Python 3.10 (ROS2) no conflict
- Separate dependencies, no version clashes

✅ **Native ROS2 Communication:**
- Direct DDS network between containers
- No HTTP bridge overhead for ROS2 topics
- Nav2 action client works natively

✅ **Scalability:**
- Easy to add more containers (vision processing, planning)
- Horizontal scaling possible
- Service-oriented architecture

✅ **Reproducibility:**
- Identical environment on any machine
- Version-locked dependencies
- Easy deployment to cloud/edge

---

### **Hybrid Navigation Strategy**

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

---

### **Safety Features**

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

---

### **AI Pipeline Features**

- **LLM-Powered Parsing**: Natural language → Structured missions (1x startup)
- **YOLO Detection**: 80 COCO classes, 50ms inference, 2Hz cached
- **State Machine**: Mission progress tracking
- **Completion Detection**: Auto-stop when goal achieved
- **Adaptive Navigation**: Hybrid Nav2/Manual based on directive
- **Real-time Tracking**: YOLO bbox center + distance estimation

---

### **Troubleshooting**

**Problem: Gazebo không hiển thị GUI**
```bash
# Check DISPLAY variable
echo $DISPLAY

# Re-enable X11
xhost +local:docker

# Restart containers
./stop.sh && ./run.sh
```

**Problem: Nav2 không nhận goal**
```bash
# Check Nav2 status
docker exec -it ros2_nav2_container bash
ros2 node list | grep bt_navigator

# Check if initial pose set
ros2 topic echo /initialpose --once

# Manually set initial pose in RViz
rviz2  # Use "2D Pose Estimate" tool
```

**Problem: RTSP stream không hoạt động**
```bash
# Check MediaMTX
docker exec -it ros2_nav2_container bash
ps aux | grep mediamtx

# Test stream
ffprobe rtsp://localhost:8554/robotcam

# Restart RTSP publisher
docker compose restart ros2-nav2
```

**Problem: Bridge server không response**
```bash
# Check bridge
curl http://localhost:8080/robot/status

# View logs
docker compose logs ros2-nav2 | grep bridge

# Restart service
docker compose restart ros2-nav2
```

**Problem: Container không start**
```bash
# Check Docker logs
docker compose logs

# Check resource usage
docker stats

# Clean and rebuild
docker compose down -v
./run.sh
```

---

### **Limitations**

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
- Docker requires ~20GB disk space for images

---

## 📚 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [Docker Documentation](https://docs.docker.com/)