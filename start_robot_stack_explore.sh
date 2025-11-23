#!/bin/bash

# Function to run command in new terminal
run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

echo "Starting robot stack..."

# Start Gazebo
run_in_terminal "cd ~ && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

sleep 5

run_in_terminal "cd ~ && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"

sleep 3

run_in_terminal "cd ~ && rviz2"

sleep 3

# Start MediaMTX
run_in_terminal "cd ~ && ./mediamtx"

sleep 2  

# Start RTSP publisher
run_in_terminal "cd ~/turtlebot3_ws/src/custom_controller/custom_controller && python3 rtsp_publisher.py"

sleep 2

# Run ffplay
run_in_terminal "cd ~ && ffplay rtsp://127.0.0.1:8554/robotcam"

echo "All services started!"
echo "You can now run your AI agent in this terminal:"
echo "cd ~/nemo-agent-toolkit && source .venv/bin/activate"
