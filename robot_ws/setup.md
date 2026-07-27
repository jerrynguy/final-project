~/start_robot_stack_explore.sh
~/start_robot_stack_patrol.sh

cd ~/nemo-agent-toolkit/docker
# 1. Start container interactive
./run_hybrid_container.sh

# 2. Trong container - chạy lần đầu
. /workspace/.venv/bin/activate
cd /workspace/mounted_code
uv pip install -e .

# 3. Run NAT
nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore 120 seconds, if you see a person follow them, otherwise stop'

nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and go around 20 times for 60 seconds'

nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore for 60 seconds'

# 4. Sửa code trên host (VSCode/vim)
# 5. Quay lại container - CHỈ CẦN:
uv pip install -e .  # Re-install để reload
nat run --config_file ... --input '...'  # Test lại

# 6. Repeat step 4-5 cho đến khi xong




cd ~/nemo-agent-toolkit/examples/multi_function_agent/gui_controller/backend
python3 main.py

cd ~/nemo-agent-toolkit/examples/multi_function_agent/gui_controller/frontend
npm run dev

http://localhost:5173



ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
rviz2
cd ~
ros2 run nav2_map_server map_saver_cli -f my_map

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml

git add .
git commit -m "update code"
git push origin main

ssh pi@192.168.2.185

