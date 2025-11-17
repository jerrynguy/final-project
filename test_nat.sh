#!/bin/bash
docker run -it --rm \
    --network=host \
    --name nat_container \
    -e ROS_DOMAIN_ID=0 \
    -e RTSP_URL="${RTSP_URL:-rtsp://172.17.0.1:8554/robotcam}" \
    -v ~/nemo-agent-toolkit/examples/multi_function_agent:/workspace/mounted_code:rw \
    -v nat_models:/workspace/persistent_data/models:rw \
    -v nat_ros2:/workspace/persistent_data/ros2_packages:ro \
    nvidia-nat:1.2.1 \
    bash -c ". /workspace/.venv/bin/activate && cd /workspace/mounted_code && uv pip install -e . >/dev/null 2>&1 && nat run --config_file /workspace/mounted_code/src/multi_function_agent/configs/config.yml --input 'Control robot using rtsp://172.17.0.1:8554/robotcam and explore for 60 seconds'"
