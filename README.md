# Robot Vision Controller with AI Navigation & Autonomous SLAM Mapping

Hệ thống điều khiển robot TurtleBot3 tự động với AI Agent thông minh, tích hợp **SLAM Toolbox** cho autonomous mapping và **Nav2 navigation stack** cho path planning an toàn. Robot có thể hiểu và thực hiện nhiệm vụ phức tạp từ natural language prompts như tạo map tự động, tuần tra theo map, bám theo mục tiêu di động, và nhiều hơn nữa.

## 🤖 AI Models Used

|         Model           |                     Purpose                        |        When Used        | Critical |
|-------------------------|----------------------------------------------------|-------------------------|----------|
| **LLM (Llama 3.1 70B)** | Parse natural language prompt → structured mission |       1x at startup     |  ✅ Yes  |
|    **YOLO (v11n)**      |            Object detection & tracking             | Continuous (2Hz cached) |  ✅ Yes  |

**Performance:**
- 🚀 Real-time navigation: <100ms per iteration
- 💾 Memory usage: ~0.5GB (YOLO only)
- ⚡ Startup time: ~1 second
- 🗺️ SLAM mapping: Auto-save every 5s

---

## 📋 Mục lục
- [Cấu trúc thư mục](#cấu-trúc-thư-mục)
- [Tổng quan hệ thống](#tổng-quan-hệ-thống)
- [Kiến trúc Native ROS2](#kiến-trúc-native-ros2)
- [Mission Types](#mission-types)
- [Mission Requirements](#mission-requirements)
- [Cài đặt](#cài-đặt)
- [Cách chạy](#cách-chạy)
- [Troubleshooting](#troubleshooting)

## 📁 Cấu trúc thư mục

```
multi_function_agent/
    ├── configs/
    │   └── config.yml                            # Cấu hình system + Nav2 + SLAM
    ├── register.py                               # Create agent's function
    └── robot_vision_controller/
        ├── main.py                               # Entry point - ROS2 + SLAM integration
        ├── test_integration.py                   # Test the system
        ├── core/
        │   ├── query_extractor.py                # Prompt information extraction
        │   ├── goal_parser.py                    # LLM mission parser with validation
        │   ├── mission_controller.py             # Mission state machine + requirements check
        │   ├── ros2_node.py                      # Centralized ROS2 node
        │   └── models.py                         # AI model management
        ├── navigation/
        │   ├── nav2_interface.py                 # Nav2 Python interface
        │   ├── navigation_reasoner.py            # Mission-aware navigation logic (SLAM-optimized)
        │   └── robot_controller_interface.py     # ROS2 DDS communication
        ├── perception/
        │   ├── slam_controller.py                # SLAM Toolbox subprocess manager
        │   ├── lidar_monitor.py                  # Real-time collision avoidance
        │   ├── robot_vision_analyzer.py          # YOLO + LIDAR spatial analysis
        │   ├── spatial_detector.py               # LIDAR spatial analysis
        │   └── rtsp_stream_handler.py            # RTSP stream handler        
        └── utils/
            ├── geometry_utils.py                 # Geometry calculation
            ├── movement_commands.py              # Commands to move
            ├── safety_checks.py                  # Safety First
            ├── ros_interface.py                  # ROS utilities
            ├── ros2_stubs.py                     # ROS2 message stubs
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
│   ROS2 Humble + Nav2 + SLAM + Gazebo        │
├─────────────────────────────────────────────┤
│  • TurtleBot3 Burger Simulation (Gazebo)    │
│  • SLAM Toolbox (Autonomous mapping)        │
│    - Real-time map building                 │
│    - Auto-save every 5s                     │
│    - Map quality validation                 │
│  • Nav2 Navigation Stack                    │
│    - Global Planner (Dijkstra/A*)           │
│    - Local Planner (DWA)                    │
│    - Costmap (Obstacle inflation)           │
│    - Recovery Behaviors                     │
│  • LIDAR Scanner (360° safety)              │
│  • Cyclone DDS (RMW middleware)             │
└─────────────────────────────────────────────┘
```

**ROS2 Topics:**
- `/cmd_vel` - Velocity commands
- `/scan` - LIDAR data (360 points)
- `/odom` - Odometry
- `/map` - SLAM map (real-time)
- `/plan` - Nav2 path

---

#### **2. NAT-Agent Container (Python 3.11 + ROS2 Bridge)**
```
┌─────────────────────────────────────────────┐
│     NVIDIA NAT + AI Agent Container         │
├─────────────────────────────────────────────┤
│  • LLM Parser (Llama 3.1 70B)               │
│    - Natural language → Mission structure   │
│    - Mission validation & requirements      │
│    - 1x at startup only                     │
│  • YOLO Object Detection (v11n)             │
│    - 80 COCO classes                        │
│    - 2Hz cached inference                   │
│  • Mission Controller                       │
│    - State machine for mission tracking     │
│    - Mission requirements validation        │
│    - Progress monitoring                    │
│  • SLAM Controller (NEW!)                   │
│    - Autonomous map generation              │
│    - Subprocess lifecycle management        │
│    - Auto-save + quality validation         │
│  • Navigation Reasoner                      │
│    - Hybrid Nav2/Manual decision logic      │
│    - SLAM-optimized exploration             │
│  • Vision Analyzer                          │
│    - YOLO + LIDAR fusion                    │
│    - Spatial awareness                      │
│  • ROS2 Subprocess Bridge                   │
│    - Python 3.11 → System Python 3.10       │
│    - Persistent daemon for sensor data      │
│    - Cyclone DDS communication              │
└─────────────────────────────────────────────┘
```

**Key Features:**
- Native ROS2 DDS communication (no HTTP bridge)
- Subprocess wrapper giải quyết Python version conflict
- Cyclone DDS for stable discovery
- YOLO-only pipeline (BLIP2 removed)
- **Autonomous SLAM mapping** - no manual intervention
- Mission-driven autonomous behavior with validation

---

## 🏗️ Kiến trúc Native ROS2

### **Python Version Challenge**
- **NAT Agent:** Requires Python 3.11+
- **ROS2 Humble:** Supports Python 3.10 only
- **SLAM Toolbox:** System Python 3.10 subprocess
- **Solution:** Subprocess wrapper - Python 3.11 venv calls system Python 3.10 (rclpy + SLAM)

### **Communication Architecture**

![Workflow Diagram](src/multi_function_agent/_robot_vision_controller/images/nat_container.png)

```
┌─────────────────────────────────────────────────────────────┐
│                    HOST MACHINE                             │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │   ROS2 Humble (Native)                              │    │
│  │   - Gazebo + Nav2 + SLAM Toolbox + TurtleBot3       │    │
│  │   - Topics: /cmd_vel, /scan, /odom, /map            │    │
│  │   - Cyclone DDS (RMW)                               │    │
│  └──────────────────┬──────────────────────────────────┘    │
│                     │                                       │
│                     │ ROS2 DDS Network (Cyclone DDS)        │
│                     │ (Host Network Mode)                   │
│                     │                                       │
│  ┌──────────────────▼──────────────────────────────────┐    │
│  │   NAT Container (nvidia-nat)                        │    │
│  │   - Python 3.11 venv (NAT Agent)                    │    │
│  │   - System Python 3.10 (rclpy + SLAM subprocess)    │    │
│  │   - core/ros2_node.py (Subprocess Bridge)           │    │
│  │   - perception/slam_controller.py (SLAM Manager)    │    │
│  │   - Persistent daemon for sensor streaming          │    │
│  │   - AI Agent + YOLO + Mission Controller            │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Communication Flow:**
1. **Sensor Data:** Host publishes → Cyclone DDS → Container daemon subprocess → JSON stdout → Python 3.11 cache
2. **Commands:** Python 3.11 → subprocess call → System Python 3.10 publish → Cyclone DDS → Host
3. **SLAM Control:** Python 3.11 → subprocess spawn → SLAM Toolbox (Python 3.10) → Auto-save maps
4. **Latency:** <10ms for cached reads, ~50ms for commands

**Why Cyclone DDS?**
- ✅ FastDDS had discovery issues with Docker host networking
- ✅ Cyclone DDS: stable, immediate discovery, zero extra config
- ✅ Tested: 360 LIDAR points @ 5Hz, zero packet loss

---

## 🎮 Mission Types

Robot hỗ trợ 3 loại nhiệm vụ thông qua natural language với **progressive unlock system**:

### **1. 🗺️ Explore Area** (Khám phá + SLAM Mapping)
```bash
"Khám phá tự do trong 60 giây"
"Explore the environment for 2 minutes"
"Run wide automatically and map the area"
```

**Requirements:** ✅ SLAM Toolbox installed  
**Navigation:** Manual exploration với SLAM-optimized movements  
**Behavior:** 
- Tự động tạo map trong quá trình explore
- Auto-save map mỗi 5 giây
- Wide sweeping motions để cover nhiều area
- Map validation khi hoàn thành

**Output:** 
- Map saved tại `~/my_map.yaml` và `~/my_map.pgm`
- Coverage statistics
- Mapping duration

**⚠️ Important:** Đây là mission bắt buộc chạy đầu tiên để tạo map cho Patrol!

---

### **2. 🔄 Patrol Laps** (Tuần tra theo map)
```bash
"Đi 20 vòng tròn"
"Patrol 5 laps"
"Go around the room 10 times"
```

**Requirements:** ✅ Map file exists (`~/my_map.yaml`)  
**Navigation:** Nav2 arc goals với pre-built map  
**Behavior:** 
- Complete N circular laps theo map
- Obstacle avoidance qua Nav2 costmap
- Return to start sau khi hoàn thành

**⚠️ Important:** Cần chạy Explore trước để tạo map, hoặc tạo map manual!

---

### **3. 🐕 Follow Target** (Bám theo mục tiêu)
```bash
"Theo sau người đang đi"
"Follow the person"
"Track the dog in front"
```

**Requirements:** ✅ Target class hợp lệ (COCO 80 classes)  
**Navigation:** Hybrid (Nav2 approach + manual tracking)  
**Behavior:** 
- Track target at safe distance (1.0-2.5m)
- Search behavior nếu mất target >3s
- Adaptive speed based on target distance

**⚠️ Important:** Mission này không cần map, có thể chạy standalone!

---

## 🔒 Mission Requirements

### **Mission Validation System**

Hệ thống tự động kiểm tra requirements trước khi start mission:

| Mission Type | Requirements | Auto-Check | Error Message |
|--------------|-------------|-----------|---------------|
| **Explore Area** | ✅ SLAM Toolbox installed | `ros2 pkg list \| grep slam_toolbox` | "Install: sudo apt install ros-humble-slam-toolbox" |
| **Patrol Laps** | ✅ Map file exists (`~/my_map.yaml`) | `os.path.exists("~/my_map.yaml")` | "Run explore mission first to create map" |
| **Follow Target** | ✅ Valid target_class | LLM parse check | "Specify target: 'Follow the person'" |

### **Progressive Unlock Flow**

```
┌─────────────────┐
│  Start System   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│ Mission: "Explore 60s"      │ ◄─── First mission (creates map)
│ ✅ SLAM Toolbox installed   │
│ ➜ Start SLAM subprocess     │
│ ➜ Navigate + auto-save map  │
│ ➜ Save final map on complete│
└────────┬────────────────────┘
         │
         │ Map created: ~/my_map.yaml
         │
         ▼
┌─────────────────────────────┐
│ Mission: "Patrol 5 laps"    │ ◄─── Unlocked after explore
│ ✅ Map found: ~/my_map.yaml │
│ ➜ Load map to Nav2          │
│ ➜ Execute circular laps     │
└─────────────────────────────┘

┌─────────────────────────────┐
│ Mission: "Follow person"    │ ◄─── Always available (no map needed)
│ ✅ Target class: person     │
│ ➜ YOLO tracking active      │
└─────────────────────────────┘
```

### **Error Handling Examples**

```bash
# ❌ Unknown mission
Input: "Make me coffee"
Error: "Mission Not Supported. Available: follow_target, patrol_laps, explore_area"

# ❌ Patrol without map
Input: "Patrol 5 laps"
Error: "Map not found at ~/my_map.yaml. Run explore mission first."

# ❌ SLAM not installed
Input: "Explore freely"
Error: "slam_toolbox package required. Install: sudo apt install ros-humble-slam-toolbox"

# ❌ Follow without target
Input: "Follow"
Error: "Target class required. Example: 'Follow the person'"
```

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

# Install ROS2 Humble Desktop + Nav2 + TurtleBot3 + SLAM Toolbox
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp

# Setup environment (IMPORTANT!)
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### **Bước 2: Verify SLAM Toolbox**

```bash
# Check SLAM Toolbox installed
ros2 pkg list | grep slam_toolbox
# Should output: slam_toolbox

# Test SLAM launch file exists
ros2 launch slam_toolbox online_async_launch.py --show-args
# Should show launch arguments
```

### **Bước 3: Cài đặt Docker**

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### **Bước 4: Clone Repository**

```bash
cd ~
git clone https://github.com/jerrynguy/final-project.git nemo-agent-toolkit
cd nemo-agent-toolkit
```

### **Bước 5: Build Docker Container**

```bash
cd ~/nemo-agent-toolkit/docker
./build_container.sh
```

**Note:** Dockerfile đã include Cyclone DDS và ROS2 packages. Build time: ~5-10 phút.

---

## 🚀 Cách chạy

### **Bước 1: Start ROS2 Environment (Host)**

```bash
# Function to run command in new terminal
run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

echo "Starting robot stack..."

# Start Gazebo
run_in_terminal "cd ~ && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

sleep 5

# Start Nav2 (for patrol missions - map will be loaded automatically)
run_in_terminal "cd ~ && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml"

sleep 3

# Start MediaMTX
run_in_terminal "cd ~ && ./mediamtx"

sleep 2  

# Start RTSP publisher
run_in_terminal "cd ~/turtlebot3_ws/src/custom_controller/custom_controller && python3 rtsp_publisher.py"

sleep 2

# Run ffplay (optional - for monitoring)
run_in_terminal "cd ~ && ffplay rtsp://127.0.0.1:8554/robotcam"

echo "All services started!"
echo "You can now run your AI agent in this terminal:"
echo "cd ~/nemo-agent-toolkit && source .venv/bin/activate"
```

**⚠️ Note:** Nav2 sẽ báo lỗi nếu `~/my_map.yaml` chưa tồn tại - đây là bình thường, chạy Explore mission trước!

### **Bước 2: Run NAT Container**

```bash
cd ~/nemo-agent-toolkit/docker

# Create named volumes (chỉ chạy 1 lần)
docker volume create nat_models 2>/dev/null || true
docker volume create nat_ros2 2>/dev/null || true

# Run container với hybrid mounts
docker run -it --rm \
    --network=host \
    --name nat_container \
    -e ROS_DOMAIN_ID=0 \
    -e RTSP_URL="${RTSP_URL:-rtsp://host.docker.internal:8554/robotcam}" \
    -e NVIDIA_API_KEY="${NVIDIA_API_KEY:-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}" \
    -e NGC_API_KEY="${NGC_API_KEY:-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}" \
    -v ~/nemo-agent-toolkit/examples/multi_function_agent:/workspace/mounted_code:rw \
    -v nat_models:/workspace/persistent_data/models:rw \
    -v nat_ros2:/workspace/persistent_data/ros2_packages:ro \
    nvidia-nat:v1.2.1 \
    "${@:-bash}"

# Usage examples:
# root@dung-HP-ZBook-Firefly-15-6-inch-G8-Mobile-Workstation-PC:/workspace/mounted_code# python3 /workspace/mounted_code/src/multi_function_agent/robot_vision_controller/test_integration.py
# ./run_hybrid_container.sh  # Interactive bash
# ./run_hybrid_container.sh "nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Navigate to (2.0, 3.0)'"
# ./run_hybrid_container.sh "nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Explore the area for 60 seconds'"
```

---

## 🔧 Troubleshooting

### **Problem: SLAM không start**

**Symptoms:**
```
[SLAM] Failed to start:
slam_toolbox: command not found
```

**Solution:**
```bash
# Check SLAM installed
ros2 pkg list | grep slam_toolbox

# If not found, install
sudo apt install ros-humble-slam-toolbox

# Verify
ros2 launch slam_toolbox online_async_launch.py --show-args
```

---

### **Problem: Map không được tạo sau explore**

**Symptoms:**
```
[SLAM] Map saved: False
Map files not created
```

**Check:**
```bash
# Verify SLAM process running
ps aux | grep slam_toolbox

# Check map_saver_cli available
ros2 run nav2_map_server map_saver_cli --help

# Check write permissions
ls -ld ~
```

**Solution:**
```bash
# Ensure nav2_map_server installed
sudo apt install ros-humble-nav2-map-server

# Test manual save
ros2 run nav2_map_server map_saver_cli -f ~/test_map
```

---

### **Problem: Patrol reject map dù đã có file**

**Symptoms:**
```
❌ Map not found at ~/my_map.yaml
```

**Check:**
```bash
# Verify file exists
ls -lh ~/my_map.yaml ~/my_map.pgm

# Check file sizes (should be >100B YAML, >1KB PGM)
du -h ~/my_map.*
```

**Solution:**
```bash
# If files too small, re-run explore with longer duration
nat run --input "Explore for 90 seconds"
```

---

### **Problem: LIDAR/Odom trả về None**

**Check ROS2 environment variables:**
```bash
# On host
echo $ROS_DOMAIN_ID  # Should be 0
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp

# Inside container
echo $ROS_DOMAIN_ID  # Should be 0
```

**Verify topics visible:**
```bash
# Inside container
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "(scan|odom)"
```

**Solution:** Ensure matching `ROS_DOMAIN_ID` và `RMW_IMPLEMENTATION` on both host and container.

---

### **Problem: Nav2 không nhận goal**

**Check Nav2 status:**
```bash
ros2 node list | grep bt_navigator
ros2 action list | grep navigate
```

**Solution:** Set initial pose in RViz (REQUIRED!):
1. Open RViz
2. Click "2D Pose Estimate" tool
3. Click on robot's position on map
4. Drag to set orientation

---

### **Problem: Container không connect ROS2**

**Verify host network mode:**
```bash
docker inspect nat_container | grep NetworkMode
# Should be "host"
```

**Check Cyclone DDS installed:**
```bash
# Inside container
dpkg -l | grep cyclonedds
```

**Solution:** Rebuild Docker image nếu thiếu Cyclone DDS.

---

## 📝 Ghi chú quan trọng

### **SLAM Integration**

✅ **Architecture:**
- **Subprocess management:** Python 3.11 spawns SLAM Toolbox (Python 3.10)
- **Auto-save:** Map saved every 5s during exploration
- **Quality validation:** File size + existence checks
- **Graceful shutdown:** SIGTERM → SIGKILL fallback
- **Error handling:** Emergency save on crash/interrupt

✅ **Performance:**
- SLAM startup: ~3s
- Map save latency: ~1s
- Auto-save interval: 5s (configurable)
- Recommended explore duration: 60-180s

---

### **Mission Validation System**

✅ **Progressive Unlock:**
- **Level 0:** Follow Target (always available)
- **Level 1:** Explore Area (requires SLAM Toolbox)
- **Level 2:** Patrol Laps (requires map from explore)

✅ **Error Messages:**
- **User-friendly:** Clear fix instructions
- **Context-aware:** Show current state + requirements
- **Actionable:** Direct commands to resolve issues

---

### **Safety Features**

**Multi-Level Protection:**
- 🛡️ **Level 0 (Nav2 Costmap)**: Proactive path planning around obstacles
- 🛡️ **Level 1 (DWA Local Planner)**: Real-time trajectory adjustment
- 🛡️ **Level 2 (LIDAR Veto)**: Pre-execution safety check
- 🛡️ **Level 3 (20Hz Monitor)**: Continuous safety during movement
- 🛡️ **Level 4 (Immediate Abort)**: <50ms stop at critical distance

**Safety Guarantees:**
- ⚡ Response time: <50ms from detection to stop
- 🎯 Abort accuracy: 100% (blocking execution)
- 📊 Monitoring rate: 20Hz during movement
- 🔒 Override capability: LIDAR Safety > Nav2 > Manual

---

### **Limitations**

**Technical Constraints:**
- **YOLO Classes**: Limited to 80 COCO classes
- **Distance Accuracy**: LiDAR-fused (±5cm), fallback heuristic (0.12-3.5m)
- **Python Version**: Subprocess overhead (~50ms per command)
- **SLAM Quality**: Depends on exploration duration (recommend 60s+ for good maps)
- **Map Size**: Larger environments require longer explore duration
- **Host Network Required**: Container must use host network mode for ROS2 DDS

**Known Issues:**
- SLAM may produce incomplete maps if exploration too short (<30s)
- Nav2 requires manual initial pose estimate in RViz
- Map overwrite warning: Re-running explore will overwrite existing map

---

## 📚 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [Docker Documentation](https://docs.docker.com/)