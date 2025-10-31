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
        ├── main.py                               # Entry point - ROS2 integration
        ├── core/
        │   ├── query_extractor.py                # Prompt information extraction
        │   ├── goal_parser.py                    # LLM mission parser 
        │   ├── mission_controller.py             # Mission state machine
        │   ├── ros2_node.py                      # ✅ NEW: Centralized ROS2 node
        │   └── models.py                         # YOLO model management
        ├── navigation/
        │   ├── nav2_interface.py                 # Nav2 Python interface
        │   ├── navigation_reasoner.py            # Mission-aware navigation logic
        │   └── robot_controller_interface.py     # ROS2 DDS communication
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
    ├── Dockerfile                                # NAT container with ROS2 packages
    └── build_container.sh                        # Container build script

turtlebot3_ws/
└── src/
    └── custom_controller/
        └── custom_controller/
            └── rtsp_publisher.py                 # RTSP stream publisher
```

---

## 🎯 Tổng quan hệ thống

Hệ thống được thiết kế theo **kiến trúc ROS2 DDS Native Communication**, AI Agent container giao tiếp trực tiếp với ROS2 nodes qua DDS network (không qua HTTP bridge).

### **Thành phần chính:**

#### **1. ROS2 Environment (Native Host)**
```
┌─────────────────────────────────────────────┐
│   ROS2 Humble + Nav2 + Gazebo (Host)        │
├─────────────────────────────────────────────┤
│  • TurtleBot3 Simulation (Gazebo)           │
│  • Nav2 Navigation Stack                    │
│    - Global Planner (Dijkstra/A*)           │
│    - Local Planner (DWA)                    │
│    - Costmap (Obstacle inflation)           │
│    - Recovery Behaviors                     │
│  • SLAM Toolbox (Real-time mapping)         │
│  • MediaMTX (RTSP streaming)                │
│  • LIDAR Scanner (360° safety)              │
└─────────────────────────────────────────────┘
```

**ROS2 Topics:**
- `/cmd_vel` - Velocity commands
- `/scan` - LIDAR data
- `/odom` - Odometry
- `/map` - SLAM map
- `/plan` - Nav2 path

---

#### **2. NAT-Agent Container (Python 3.12 + ROS2 Client)**
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
│  • ROS2 Native Communication                │
│    - rclpy for DDS communication            │
│    - Direct topic pub/sub                   │
│    - Nav2 action client                     │
└─────────────────────────────────────────────┘
```

**Key Features:**
- Native ROS2 DDS communication (no HTTP bridge)
- Direct topic publishing/subscribing
- Nav2 action client for goal sending
- YOLO-only pipeline (BLIP2 removed)
- Mission-driven autonomous behavior

---

### **Workflow tổng quan - Native ROS2 DDS Architecture**


![Workflow Diagram](src/multi_function_agent/robot_vision_controller/images/nat_container.png)

```
┌─────────────────────────────────────────────────────────────┐
│                    HOST MACHINE                             │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐  │
│  │   ROS2 Humble (Native)                              │  │
│  │   - Gazebo + Nav2 + TurtleBot3                      │  │
│  │   - Topics: /cmd_vel, /scan, /odom, /map           │  │
│  └──────────────────┬──────────────────────────────────┘  │
│                     │                                      │
│                     │ ROS2 DDS Network (FastRTPS)          │
│                     │ (Host Network Mode)                  │
│                     │                                      │
│  ┌──────────────────▼──────────────────────────────────┐  │
│  │   NAT Container (nvidia-nat)                        │  │
│  │   - Python 3.12 + ROS2 Client Libraries            │  │
│  │   - core/ros2_node.py (Centralized Node)           │  │
│  │   - Direct pub/sub: /cmd_vel, /scan, /odom         │  │
│  │   - Nav2 action client: /navigate_to_pose          │  │
│  │   - AI Agent + YOLO + Mission Controller            │  │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘

```

**Communication Flow:**
1. **Sensor Data:** `/scan`, `/odom` → ROS2 DDS → NAT container subscribers
2. **Commands:** NAT container publisher → ROS2 DDS → `/cmd_vel` → Gazebo
3. **Nav2 Goals:** NAT container action client → ROS2 DDS → Nav2 action server
4. **Latency:** <1ms (shared memory), <3ms (localhost UDP)

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

---

## 💻 Yêu cầu hệ thống

### **Phần mềm bắt buộc**
- **Ubuntu 22.04 LTS** (khuyến nghị)
- **ROS2 Humble** (native install on host)
- **Docker** 20.10+ 
- **NVIDIA GPU** (optional, for faster YOLO)

### **Phần cứng khuyến nghị**
- **RAM**: 8GB+ (ROS2 + Docker: ~3GB)
- **GPU**: NVIDIA with CUDA support (optional)
- **CPU**: 4+ cores
- **Disk**: 10GB free space (Docker image)

### **Dependencies tự động cài:**
- ROS2 Humble packages (host)
- Nav2 navigation stack (host)
- TurtleBot3 packages (host)
- ROS2 client libraries (container)
- YOLO model (container)
- Python libraries (container)

---

## 🔧 Cài đặt

### **Bước 1: Cài đặt ROS2 Humble (Host)**

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Nav2
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Install TurtleBot3
sudo apt install -y ros-humble-turtlebot3*

# Install SLAM Toolbox
sudo apt install -y ros-humble-slam-toolbox

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **Bước 2: Cài đặt Docker**

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Verify installation
docker --version
```

### **Bước 3: Clone Repository**

```bash
cd ~
git clone https://github.com/jerrynguy/final-project.git nemo-agent-toolkit
cd nemo-agent-toolkit
```

### **Bước 4: Tạo Map (chỉ cần 1 lần)**

```bash
# Terminal 1: Launch Gazebo
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3: Launch RViz
rviz2

# Terminal 4: Teleop để khám phá
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 5: Save map khi đủ
cd ~
ros2 run nav2_map_server map_saver_cli -f my_map
```

### **Bước 5: Build Docker Container**

```bash
cd ~/nemo-agent-toolkit/docker
./build_container.sh
```

---

## 🚀 Cách chạy

### **Bước 1: Start ROS2 Environment (Host)**

```bash
# Terminal 1: Launch Nav2
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=$HOME/my_map.yaml

# Terminal 2: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 3: Launch RTSP publisher (if needed)
cd ~/nemo-agent-toolkit/turtlebot3_ws
source install/setup.bash
python3 src/custom_controller/custom_controller/rtsp_publisher.py
```

### **Bước 2: Set Initial Pose in RViz**

```bash
# Terminal 4: Launch RViz
rviz2

# In RViz:
# 1. Click "2D Pose Estimate" tool
# 2. Click on robot's position on map
# 3. Drag to set orientation
```

### **Bước 3: Run NAT Container**

```bash
# Terminal 5: Start NAT container
docker run -it --rm \
    --network=host \
    --name nat_container \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v $(pwd)/../multi_function_agent:/workspace/multi_function_agent:rw \
    -v $(pwd)/../configs:/workspace/configs:ro \
    --runtime=nvidia \
    nvidia-nat:latest bash
```

### **Bước 4: Verify ROS2 Connection**

```bash
# Inside container
source /opt/ros/humble/setup.bash

# Check nodes
ros2 node list
# Expected: /gazebo, /bt_navigator, /controller_server, /nat_agent_node

# Check topics
ros2 topic list | grep -E "(cmd_vel|scan|odom)"

# Test echo
ros2 topic echo /scan --once
```

### **Bước 5: Run Mission**

```bash
# Inside container
nat run --config_file /workspace/configs/config.yml --input "YOUR_MISSION"
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

---

## 📝 Ghi chú quan trọng

### **Native ROS2 DDS Communication**

✅ **Benefits:**
- **Zero HTTP overhead:** Direct DDS communication (<1ms latency)
- **Real-time callbacks:** Sensor data updates via subscribers
- **Native Nav2 integration:** Action client works natively
- **Production-ready:** Same architecture as real hardware
- **Simplified codebase:** No bridge server maintenance

✅ **Architecture:**
- **Host network mode:** Container shares host's network stack
- **Centralized node:** Single ROS2 node (`core/ros2_node.py`) manages all communication
- **Thread-safe data access:** Lock-protected sensor data storage
- **Background spinning:** MultiThreadedExecutor in daemon thread

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

**Problem: ros2 node list không thấy nodes**
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Phải = 0

# Check if ROS2 running on host
ps aux | grep ros2

# Restart ROS2 environment
```

**Problem: No LIDAR data trong container**
```bash
# Check topic on host
ros2 topic hz /scan

# Check QoS compatibility
ros2 topic info /scan -v

# Inside container
ros2 topic echo /scan --once
```

**Problem: Nav2 không nhận goal**
```bash
# Check Nav2 status
ros2 node list | grep bt_navigator

# Set initial pose in RViz (REQUIRED!)
rviz2  # Use "2D Pose Estimate" tool

# Check action server
ros2 action list | grep navigate
```

**Problem: Container không kết nối ROS2**
```bash
# Verify host network mode
docker inspect nat_container | grep NetworkMode
# Should be "host"

# Check ROS_DOMAIN_ID match
# Host
echo $ROS_DOMAIN_ID

# Container
docker exec nat_container bash -c "echo \$ROS_DOMAIN_ID"
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
- **Host Network Required**: Container must use host network mode for ROS2 DDS

**Known Issues:**
- Camera resolution fixed at 640x480
- YOLO confidence threshold: 0.5 (adjustable)
- Nav2 goal tolerance: 0.2m position, 0.1rad orientation
- Container requires ROS2 packages (~500MB added to base image)

---

## 📚 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [Docker Documentation](https://docs.docker.com/)
- [FastRTPS Documentation](https://fast-dds.docs.eprosima.com/)