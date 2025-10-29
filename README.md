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

## 🗝️ Kiến trúc

### **1. Docker Multi-Container Architecture**
```
┌────────────────────────────────────────────────────────────────────────────┐
│                            HOST SYSTEM (Ubuntu 22.04)                      │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  Docker Daemon                                                             │
│  ├─ Network: host mode (ROS2 DDS multicast)                                │
│  ├─ Volume mounts: /tmp/.X11-unix, workspaces, maps                        │
│  └─ ROS_DOMAIN_ID: 42 (isolated from host)                                 │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  Container 1: ros2-nav2                                              │  │
│  │  ════════════════════════════════════════════════════════════════════│  │
│  │  Base Image: osrf/ros:humble-desktop-full                            │  │
│  │  Python: 3.10 (ROS2 Humble compatible)                               │  │
│  │                                                                      │  │
│  │  Installed Packages:                                                 │  │
│  │  ├─ ros-humble-turtlebot3* (simulation + drivers)                    │  │
│  │  ├─ ros-humble-navigation2 (Nav2 stack)                              │  │
│  │  ├─ ros-humble-nav2-bringup (launchers)                              │  │
│  │  ├─ ros-humble-slam-toolbox (SLAM)                                   │  │
│  │  ├─ ros-humble-gazebo-ros-pkgs (simulation)                          │  │
│  │  ├─ ros-humble-rmw-cyclonedds-cpp (DDS middleware)                   │  │
│  │  ├─ flask, requests (bridge server)                                  │  │
│  │  └─ mediamtx (RTSP server binary)                                    │  │
│  │                                                                      │  │
│  │  Running Services (via ros2_entrypoint.sh):                          │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Service 1: Gazebo (PID monitoring)                             │  │  │
│  │  │   • Launch: turtlebot3_world.launch.py                         │  │  │
│  │  │   • Robot model: waffle (camera + lidar + diff_drive)          │  │  │
│  │  │   • World: turtlebot3_world (obstacles)                        │  │  │
│  │  │   • Physics: ODE, 1000Hz                                       │  │  │
│  │  │   • GUI: Gazebo client via X11 forwarding                      │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Service 2: Nav2 Stack                                          │  │  │
│  │  │   • Launch: navigation2.launch.py                              │  │  │
│  │  │   • Map: /root/my_map.yaml (pre-built) OR SLAM                 │  │  │
│  │  │   • Global Planner: NavFn (Dijkstra/A*)                        │  │  │
│  │  │   • Local Planner: DWA (Dynamic Window Approach)               │  │  │
│  │  │   • Costmap:                                                   │  │  │
│  │  │     - Static layer (map)                                       │  │  │
│  │  │     - Obstacle layer (lidar)                                   │  │  │
│  │  │     - Inflation layer (safety buffer)                          │  │  │
│  │  │   • Recovery Behaviors:                                        │  │  │
│  │  │     - Rotate recovery                                          │  │  │
│  │  │     - Back up recovery                                         │  │  │
│  │  │     - Clear costmap recovery                                   │  │  │
│  │  │   • Action Server: /navigate_to_pose                           │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Service 3: MediaMTX (RTSP Server)                              │  │  │
│  │  │   • Binary: /root/mediamtx                                     │  │  │
│  │  │   • Protocol: RTSP/RTP                                         │  │  │
│  │  │   • Stream path: /robotcam                                     │  │  │
│  │  │   • Port: 8554                                                 │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Service 4: RTSP Publisher                                      │  │  │
│  │  │   • Script: rtsp_publisher.py                                  │  │  │
│  │  │   • Subscribes: /camera/image_raw                              │  │  │
│  │  │   • Publishes: rtsp://localhost:8554/robotcam                  │  │  │
│  │  │   • Encoding: H.264, 15fps                                     │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Service 5: Bridge Server (HTTP ↔ ROS2)                         │  │  │
│  │  │   • Framework: Flask                                           │  │  │
│  │  │   • Port: 8080                                                 │  │  │
│  │  │   • Endpoints:                                                 │  │  │
│  │  │     - POST /robot/command → /cmd_vel publisher                 │  │  │
│  │  │     - GET /robot/status → health check                         │  │  │
│  │  │     - GET /robot/lidar → /scan subscriber                      │  │  │
│  │  │   • Background thread: rclpy.spin_once() loop (100Hz)          │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                      │  │
│  │  Exposed Ports:                                                      │  │
│  │  • 11345: Gazebo master                                              │  │
│  │  • 8554: RTSP stream                                                 │  │
│  │  • 8080: Bridge HTTP API                                             │  │
│  │                                                                      │  │
│  │  Health Check: ros2 topic list (every 5s)                            │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                            │
│                      ROS2 DDS Network (CycloneDDS)                         │
│                      Domain ID: 42, Multicast enabled                      │
│                                ▲  ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  Container 2: nat-agent                                              │  │
│  │  ════════════════════════════════════════════════════════════════════│  │
│  │  Base Image: nvcr.io/nvidia/base/ubuntu:22.04                        │  │
│  │  Python: 3.11 (NAT compatible)                                       │  │
│  │                                                                      │  │
│  │  Installed Packages:                                                 │  │
│  │  ├─ nvidia-nat[all]==1.0.0 (AI frameworks)                           │  │
│  │  ├─ ros-humble-rclpy (ROS2 client - minimal)                         │  │
│  │  ├─ ros-humble-geometry-msgs, nav-msgs, sensor-msgs                  │  │
│  │  ├─ ros-humble-nav2-msgs (action definitions)                        │  │
│  │  ├─ ros-humble-tf-transformations (quaternion utils)                 │  │
│  │  ├─ ros-humble-rmw-cyclonedds-cpp (DDS middleware)                   │  │
│  │  ├─ ultralytics (YOLOv11n)                                           │  │
│  │  ├─ opencv-python, numpy, torch, torchvision                         │  │
│  │  └─ flask, requests (HTTP client)                                    │  │
│  │                                                                      │  │
│  │  Application Components:                                             │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 1. LLM Mission Parser (Startup Phase - 1x)                     │  │  │
│  │  │   • Model: Llama 3.1 70B via NIM API                           │  │  │
│  │  │   • Input: Natural language prompt                             │  │  │
│  │  │   • Output: Structured mission object                          │  │  │
│  │  │     {                                                          │  │  │
│  │  │       "type": "explore_area" | "count_objects" |               │  │  │
│  │  │               "follow_target" | "patrol_laps",                 │  │  │
│  │  │       "target_class": "person" | "bottle" | null,              │  │  │
│  │  │       "parameters": {...}                                      │  │  │
│  │  │     }                                                          │  │  │
│  │  │   • Execution time: ~3s                                        │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 2. YOLO Object Detector (Continuous - 2Hz cached)              │  │  │
│  │  │   • Model: YOLOv11n (nano variant)                             │  │  │
│  │  │   • Classes: 80 COCO (person, bottle, cup, chair, etc.)        │  │  │
│  │  │   • Input: 640x480 RGB frame from RTSP                         │  │  │
│  │  │   • Output: List[Detection]                                    │  │  │
│  │  │     {                                                          │  │  │
│  │  │       "class": "person",                                       │  │  │
│  │  │       "confidence": 0.87,                                      │  │  │
│  │  │       "bbox": [x1, y1, x2, y2],                                │  │  │
│  │  │       "center": [cx, cy],                                      │  │  │
│  │  │       "distance": 1.2  # from LIDAR fusion                     │  │  │
│  │  │     }                                                          │  │  │
│  │  │   • Inference time: ~50ms (CPU) or ~20ms (GPU)                 │  │  │
│  │  │   • Cache duration: 500ms (reduce overhead)                    │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 3. Mission Controller (State Machine)                          │  │  │
│  │  │   • Tracks mission progress:                                   │  │  │
│  │  │     - count_objects: current_count / target_count              │  │  │
│  │  │     - follow_target: tracking_state, lost_duration             │  │  │
│  │  │     - patrol_laps: current_lap / target_laps                   │  │  │
│  │  │     - explore_area: elapsed_time / duration                    │  │  │
│  │  │   • Generates directives based on state:                       │  │  │
│  │  │     - explore_random, explore_forward                          │  │  │
│  │  │     - track_follow, track_approach, track_backup               │  │  │
│  │  │     - track_search_left, track_search_right, track_search_spin │  │  │
│  │  │     - patrol_arc_left, patrol_arc_right                        │  │  │
│  │  │   • Completion detection: Auto-stop when goal achieved         │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 4. Vision Analyzer (YOLO + LIDAR Fusion)                       │  │  │
│  │  │   • YOLO detections → 2D bounding boxes                        │  │  │
│  │  │   • LIDAR scan → 360° distance map                             │  │  │
│  │  │   • Spatial fusion algorithm:                                  │  │  │
│  │  │     1. Project bbox center to angle: θ = atan2(cy, cx)         │  │  │
│  │  │     2. Query LIDAR at θ ± 10° cone                             │  │  │
│  │  │     3. Distance = median(LIDAR readings in cone)               │  │  │
│  │  │   • Outputs:                                                   │  │  │
│  │  │     - detected_objects: List[Object with distance]             │  │  │
│  │  │     - obstacles: [{zone, distance, angle}]                     │  │  │
│  │  │     - clear_paths: ["forward", "left", "right"]                │  │  │
│  │  │     - safety_score: 0-10                                       │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 5. Navigation Reasoner (Hybrid Decision Logic)                 │  │  │
│  │  │   • Input: mission_directive, vision_analysis, robot_pos       │  │  │
│  │  │   • Decision tree:                                             │  │  │
│  │  │     ┌──────────────────────────────────────────────────────┐   │  │  │
│  │  │     │ Can use Nav2?                                        │   │  │  │
│  │  │     │ ✓ nav2_ready AND robot_pos available AND             │   │  │  │
│  │  │     │   directive in {explore_*, patrol_*, track_approach} │   │  │  │
│  │  │     └──────────┬───────────────────────────────┬───────────┘   │  │  │
│  │  │                │ YES                           │ NO            │  │  │
│  │  │                ▼                               ▼               │  │  │
│  │  │     ┌────────────────────┐       ┌──────────────────────────┐  │  │  │
│  │  │     │ Nav2 Path          │       │ Manual Control           │  │  │  │
│  │  │     │ • Convert directive│       │ • Generate cmd_vel       │  │  │  │
│  │  │     │   to (x,y,θ) goal  │       │ • Direct robot control   │  │  │  │
│  │  │     │ • Send via action  │       │ • LIDAR safety layer     │  │  │  │
│  │  │     │ • Non-blocking     │       │ • Blocking with abort    │  │  │  │
│  │  │     └────────────────────┘       └──────────────────────────┘  │  │  │
│  │  │     └────────────────────────────────────────────────────────┘ │  │  │
│  │  │   • Manual directives (Nav2 bypass):                           │  │  │
│  │  │     - track_follow: Real-time YOLO bbox tracking               │  │  │
│  │  │     - track_backup: Precise reverse distance                   │  │  │
│  │  │     - track_search_spin: 360° rotation in place                │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 6. Robot Controller Interface (ROS2 Native + Bridge)           │  │  │
│  │  │   • Primary mode: Native ROS2 (rclpy)                          │  │  │
│  │  │     - Publisher: /cmd_vel (Twist messages)                     │  │  │
│  │  │     - Subscriber: /scan (LaserScan), /odom (Odometry)          │  │  │
│  │  │     - Action Client: /navigate_to_pose (Nav2 goals)            │  │  │
│  │  │   • Fallback mode: HTTP Bridge                                 │  │  │
│  │  │     - Used if rclpy fails to initialize                        │  │  │
│  │  │     - POST /robot/command, GET /robot/lidar                    │  │  │
│  │  │   • Nav2 Integration:                                          │  │  │
│  │  │     - nav2_interface.py: NavigateToPose action client          │  │  │
│  │  │     - send_goal(x, y, theta): Non-blocking goal submission     │  │  │
│  │  │     - cancel_navigation(): Emergency abort                     │  │  │
│  │  │     - get_state(): Check navigation status                     │  │  │
│  │  │   • Safety monitoring:                                         │  │  │
│  │  │     - 20Hz LIDAR checks during command execution               │  │  │
│  │  │     - Immediate abort if obstacle < 0.3m                       │  │  │
│  │  │     - Progressive speed scaling near obstacles                 │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ 7. LIDAR Safety Monitor (Veto Layer)                           │  │  │
│  │  │   • Priority 0: Pre-execution check                            │  │  │
│  │  │     - Block command if min_distance < 0.3m                     │  │  │
│  │  │     - Force emergency stop                                     │  │  │
│  │  │   • Priority 1: Continuous monitoring (20Hz)                   │  │  │
│  │  │     - During command execution                                 │  │  │
│  │  │     - Abort mid-execution if critical                          │  │  │
│  │  │   • Priority 2: Nav2 safety override                           │  │  │
│  │  │     - Non-blocking check during Nav2 navigation                │  │  │
│  │  │     - Cancel Nav2 goal if danger detected                      │  │  │
│  │  │   • Distance thresholds:                                       │  │  │
│  │  │     - CRITICAL: 0.3m (immediate abort)                         │  │  │
│  │  │     - WARNING: 0.5m (speed reduction)                          │  │  │
│  │  │     - SAFE: 0.8m+ (normal operation)                           │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                      │  │
│  │  Main Control Loop (10Hz - 100ms per iteration):                     │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │ while not mission_completed:                                   │  │  │
│  │  │   1. Get frame from RTSP stream                                │  │  │
│  │  │   2. Check Nav2 safety (if navigating)           [Priority 0]  │  │  │
│  │  │   3. LIDAR safety check (blocking)                [Priority 1] │  │  │
│  │  │   4. Vision analysis (YOLO 2Hz cached)            [Priority 2] │  │  │
│  │  │   5. Mission state update                         [Priority 3] │  │  │
│  │  │   6. Navigation decision (Nav2/Manual)            [Priority 4] │  │  │
│  │  │   7. Execute command (with safety abort)          [Priority 5] │  │  │
│  │  │   8. Sleep 50ms                                                │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                      │  │
│  │  Depends on: ros2-nav2 (waits for healthcheck)                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

### **2. ROS2 Communication Layer**
```
┌─────────────────────────────────────────────────────────────────────────┐
│                     ROS2 DDS Network (CycloneDDS)                       │
│                           Domain ID: 42                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Topics (Publish/Subscribe):                                            │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /cmd_vel (geometry_msgs/Twist)                                   │  │
│  │    Publishers:                                                    │  │
│  │      • NAT Container (manual control)                             │  │
│  │      • Nav2 Local Planner (DWA output)                            │  │
│  │    Subscriber: Gazebo diff_drive controller                       │  │
│  │    Rate: Variable (manual: on-demand, Nav2: 20Hz)                 │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /scan (sensor_msgs/LaserScan)                                    │  │
│  │    Publisher: Gazebo LiDAR plugin (360°, 3.5m max)                │  │
│  │    Subscribers:                                                   │  │
│  │      • NAT Container (safety monitoring)                          │  │
│  │      • Nav2 Costmap (obstacle layer)                              │  │
│  │      • Bridge Server (HTTP endpoint)                              │  │
│  │    Rate: 20Hz                                                     │  │
│  │    Data: 360 range readings, 0.12-3.5m                            │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /odom (nav_msgs/Odometry)                                        │  │
│  │    Publisher: Gazebo ground truth odometry                        │  │
│  │    Subscribers:                                                   │  │
│  │      • NAT Container (robot position)                             │  │
│  │      • Nav2 (localization)                                        │  │
│  │    Rate: 30Hz                                                     │  │
│  │    Data: position (x,y,θ), velocity                               │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /camera/image_raw (sensor_msgs/Image)                            │  │
│  │    Publisher: Gazebo camera plugin                                │  │
│  │    Subscriber: RTSP Publisher (MediaMTX bridge)                   │  │
│  │    Rate: 15fps                                                    │  │
│  │    Format: 640x480, RGB8                                          │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /map (nav_msgs/OccupancyGrid)                                    │  │
│  │    Publisher: Map Server (pre-built) OR SLAM Toolbox              │  │
│  │    Subscriber: Nav2 Global Costmap                                │  │
│  │    Rate: On-demand (static map) or 1Hz (SLAM)                     │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /plan (nav_msgs/Path)                                            │  │
│  │    Publisher: Nav2 Global Planner (A*/Dijkstra output)            │  │
│  │    Subscriber: NAT Container (optional visualization)             │  │
│  │    Rate: On new goal or replan (event-driven)                     │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Actions (Client/Server):                                               │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  /navigate_to_pose (nav2_msgs/action/NavigateToPose)              │  │
│  │    Server: Nav2 BT Navigator                                      │  │
│  │    Client: NAT Container (nav2_interface.py)                      │  │
│  │    Goal: PoseStamped (x, y, θ in map frame)                       │  │
│  │    Feedback: distance_remaining, ETA                              │  │
│  │    Result: SUCCESS (4) or FAILED (5/6)                            │  │
│  │    Flow:                                                          │  │
│  │      1. NAT sends goal (non-blocking)                             │  │
│  │      2. Nav2 plans global path (A*)                               │  │
│  │      3. Nav2 executes local planner (DWA)                         │  │
│  │      4. Nav2 publishes /cmd_vel continuously                      │  │
│  │      5. Nav2 returns result when goal reached or failed           │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

### **3. Navigation Decision Tree (Hybrid Logic)**
```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Mission Directive Generated                          │
│             (from mission_controller based on state)                    │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
              ┌──────────────────────────────────────┐
              │ Can use Nav2 for this directive?     │
              │                                      │
              │ Conditions:                          │
              │ ✓ nav2_ready == True                 │
              │ ✓ robot_pos available (x, y, θ)      │
              │ ✓ Nav2 not currently navigating      │
              │ ✓ Directive in Nav2-compatible set:  │
              │   • explore_*                        │
              │   • patrol_*                         │
              │   • track_approach                   │
              └──────────┬───────────────┬───────────┘
                         │ YES           │ NO
                         ▼               ▼
        ┌────────────────────────────┐  ┌────────────────────────────┐
        │      NAV2 PATH             │  │     MANUAL PATH            │
        └────────────────────────────┘  └────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                           NAV2 PATH                                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 1: Convert directive to (x, y, θ) goal                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   current_pos = (robot.x, robot.y, robot.θ)                             │
│                                                                         │
│   if directive == "explore_random":                                     │
│     angle = random(-π, π)                                               │
│     distance = random(2.0, 3.0)                                         │
│                                                                         │
│   elif directive == "patrol_arc_left":                                  │
│     angle = robot.θ + 0.3 rad                                           │
│     distance = 1.0                                                      │
│                                                                         │
│   elif directive == "track_approach":                                   │
│     angle = robot.θ + 0.0 rad                                           │
│     distance = 0.5                                                      │
│                                                                         │
│   goal_x = robot.x + distance * cos(angle)                              │
│   goal_y = robot.y + distance * sin(angle)                              │
│   goal_θ = angle                                                        │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 2: Send Nav2 goal (non-blocking)                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   success = nav2_interface.send_goal(goal_x, goal_y, goal_θ)            │
│                                                                         │
│   Action client sends NavigateToPose.Goal to Nav2                       │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 3: Nav2 execution flow                                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   a. Global Planner (A*/Dijkstra)                                       │
│      • Searches on costmap                                              │
│      • Finds optimal path avoiding obstacles                            │
│      • Publishes to /plan topic                                         │
│                                                                         │
│   b. Local Planner (DWA)                                                │
│      • Samples velocity trajectories                                    │
│      • Scores by:                                                       │
│        - Path alignment                                                 │
│        - Obstacle clearance                                             │
│        - Goal distance                                                  │
│      • Publishes /cmd_vel @ 20Hz                                        │
│                                                                         │
│   c. Recovery Behaviors (if stuck)                                      │
│      • Rotate 360° to find path                                         │
│      • Back up and retry                                                │
│      • Clear costmap and replan                                         │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 4: NAT Container monitoring                                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   • Continue main loop @ 10Hz                                           │
│   • Check nav2_interface.is_navigating()                                │
│   • Safety override if LIDAR critical                                   │
│   • Cancel Nav2 if mission state changes                                │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 5: Completion                                                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   • Result callback triggered by action server                          │
│   • State updates to SUCCEEDED or FAILED                                │
│   • NAT resumes directive generation                                    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                          MANUAL PATH                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 1: Generate cmd_vel from directive                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   navigation_reasoner.decide_next_action()                              │
│                                                                         │
│   Directive to Command mapping:                                         │
│                                                                         │
│   track_follow:                                                         │
│     • Use YOLO bbox center (cx, cy)                                     │
│     • Angular: align with center                                        │
│     • Linear: approach if distance > 1.5m                               │
│                                                                         │
│   track_backup:                                                         │
│     • Linear: -0.1 m/s                                                  │
│     • Duration: calculate from distance                                 │
│                                                                         │
│   track_search_spin:                                                    │
│     • Angular: 0.5 rad/s                                                │
│     • Duration: 12s (360° rotation)                                     │
│                                                                         │
│   explore_forward:                                                      │
│     • Linear: 0.15 m/s                                                  │
│     • Angular: 0.0 (straight)                                           │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 2: Pre-execution safety check                                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   lidar_data = robot_interface.lidar_data                               │
│   min_dist = min(lidar_data.ranges)                                     │
│                                                                         │
│   if min_dist < 0.3m:                                                   │
│     ABORT → Force stop command                                          │
│     Log: "Pre-execution abort"                                          │
│     return                                                              │
│                                                                         │
│   elif min_dist < 0.5m:                                                 │
│     SCALE DOWN velocity:                                                │
│     scale = (min_dist - 0.3) / 0.2                                      │
│     velocity *= scale * 0.5                                             │
│     Log: "Speed reduction"                                              │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 3: Execute command (blocking with abort)                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   robot_interface.execute_command(cmd)                                  │
│                                                                         │
│   publish_rate = 20 Hz (50ms interval)                                  │
│   iterations = duration * 20                                            │
│                                                                         │
│   for i in range(iterations):                                           │
│     # CRITICAL: Safety check BEFORE publish                             │
│     fresh_lidar = get_latest_lidar()                                    │
│     min_dist = min(fresh_lidar.ranges)                                  │
│                                                                         │
│     if min_dist < 0.3m:                                                 │
│       # IMMEDIATE ABORT                                                 │
│       publish(Twist.zero()) × 5 times                                   │
│       Log: "Emergency abort mid-execution"                              │
│       return False                                                      │
│                                                                         │
│     elif min_dist < 0.5m:                                               │
│       # PROGRESSIVE SCALING                                             │
│       scale = (min_dist - 0.3) / 0.2                                    │
│       scaled_twist = twist * scale * 0.5                                │
│       publish(scaled_twist)                                             │
│                                                                         │
│     else:                                                               │
│       # SAFE: Publish original command                                  │
│       publish(twist)                                                    │
│                                                                         │
│     sleep(50ms)                                                         │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ Step 4: Post-execution                                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   • Return success/failure to main loop                                 │
│   • Log command result                                                  │
│   • Continue to next iteration                                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                             │
                             │
                             ▼
              ┌──────────────────────────────────────┐
              │  Return to main control loop (10Hz)  │
              └──────────────────────────────────────┘
```

---

### **4. Safety Architecture (Multi-Layer Defense)**
```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SAFETY LAYERS (Priority Order)                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Layer 0: Nav2 Costmap (Proactive - 20Hz)                               │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Input: /map (static) + /scan (dynamic)                         │  │
│  │  • Inflation radius: 0.5m around obstacles                        │  │
│  │  • Lethal threshold: Obstacle within robot footprint              │  │
│  │  • Action: Replan path to avoid inflated areas                    │  │
│  │  • Response time: <50ms (planner cycle)                           │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Layer 1: DWA Local Planner (Reactive - 20Hz)                           │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Input: Current velocity + LIDAR scan                           │  │
│  │  • Dynamic Window: Sample forward trajectories                    │  │
│  │  • Collision check: Project trajectories against obstacles        │  │
│  │  • Action: Select safe trajectory with best score                 │  │
│  │  • Fallback: Recovery behaviors if no safe path                   │  │
│  │  • Response time: 50ms (planner update)                           │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Layer 2: NAT LIDAR Veto (Nav2 Override - Non-blocking)                 │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Check frequency: Every main loop iteration (10Hz)              │  │
│  │  • Condition: if nav2_interface.is_navigating()                   │  │
│  │  • Logic:                                                         │  │
│  │    if min(lidar.ranges) < CRITICAL (0.3m):                        │  │
│  │      nav2_interface.cancel_navigation()                           │  │
│  │      Log: "Nav2 aborted - safety override"                        │  │
│  │      Continue loop (non-blocking)                                 │  │
│  │  • Purpose: Abort Nav2 if it fails to detect danger               │  │
│  │  • Response time: 100ms (loop period)                             │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Layer 3: Pre-Execution Check (Manual Control - Blocking)               │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Triggered: Before every manual command                         │  │
│  │  • Logic:                                                         │  │
│  │    fresh_lidar = get_latest_lidar()                               │  │
│  │    min_dist = min(fresh_lidar.ranges)                             │  │
│  │                                                                   │  │
│  │    if min_dist < CRITICAL (0.3m):                                 │  │
│  │      REJECT command                                               │  │
│  │      Force stop: publish Twist.zero()                             │  │
│  │      return to loop                                               │  │
│  │                                                                   │  │
│  │    elif min_dist < WARNING (0.5m):                                │  │
│  │      SCALE velocity: v *= (min_dist - 0.3) / 0.2 * 0.5            │  │
│  │      Log: "Speed reduction applied"                               │  │
│  │  • Purpose: Prevent execution of dangerous commands               │  │
│  │  • Response time: <10ms (instant)                                 │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Layer 4: Mid-Execution Abort (Manual Control - Aggressive)             │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Check frequency: Every publish cycle (20Hz during motion)      │  │
│  │  • Logic inside _send_command():                                  │  │
│  │    for each 50ms interval:                                        │  │
│  │      fresh_lidar = get_latest_lidar()                             │  │
│  │      min_dist = min(fresh_lidar.ranges)                           │  │
│  │                                                                   │  │
│  │      if min_dist < CRITICAL (0.3m):                               │  │
│  │        # EMERGENCY ABORT                                          │  │
│  │        spam_stop_commands() × 5                                   │  │
│  │        Log: "Emergency abort - obstacle {min_dist}m"              │  │
│  │        return False                                               │  │
│  │                                                                   │  │
│  │      elif min_dist < WARNING (0.5m):                              │  │
│  │        # PROGRESSIVE SCALING                                      │  │
│  │        publish(scaled_twist)                                      │  │
│  │                                                                   │  │
│  │      else:                                                        │  │
│  │        publish(original_twist)                                    │  │
│  │  • Purpose: Real-time abort during command execution              │  │
│  │  • Response time: <50ms (single publish cycle)                    │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Layer 5: Exception Handler (Last Resort)                               │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  • Triggered: Any unhandled exception in control loop             │  │
│  │  • Action:                                                        │  │
│  │    try: control_loop()                                            │  │
│  │    except Exception as e:                                         │  │
│  │      Log error                                                    │  │
│  │      spam_stop_commands() × 5                                     │  │
│  │      Set state: ERROR                                             │  │
│  │      Exit gracefully                                              │  │
│  │  • Purpose: Prevent runaway robot on software crash               │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│  Distance Thresholds:                                                   │
│  • CRITICAL: 0.3m - Immediate abort, force stop                         │
│  • WARNING: 0.5m - Speed reduction, cautious progress                   │
│  • SAFE: 0.8m+ - Normal operation, full speed allowed                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
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