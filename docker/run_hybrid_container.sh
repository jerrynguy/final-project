#!/bin/bash

if [ -z "${NVIDIA_API_KEY:-}" ]; then
    echo "Error: NVIDIA_API_KEY is not set." >&2
    echo "Export your NVIDIA API key (from build.nvidia.com) before running this script:" >&2
    echo "    export NVIDIA_API_KEY=<your-key>" >&2
    echo "NGC_API_KEY is optional and defaults to NVIDIA_API_KEY." >&2
    exit 1
fi

docker volume create nat_models 2>/dev/null || true
docker volume create nat_ros2 2>/dev/null || true
# Run container với hybrid mounts
docker run -it --rm \
    --network=host \
    --name nat_container \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_DISTRO=humble \
    -e AMENT_PREFIX_PATH=/opt/ros/humble \
    -e ROS_VERSION=2 \
    -e ROS_PYTHON_VERSION=3 \
    -e RTSP_URL="${RTSP_URL:-rtsp://host.docker.internal:8554/robotcam}" \
    -e NVIDIA_API_KEY="${NVIDIA_API_KEY}" \
    -e NGC_API_KEY="${NGC_API_KEY:-$NVIDIA_API_KEY}" \
    -v ~/nemo-agent-toolkit/examples/multi_function_agent:/workspace/mounted_code:rw \
    -v nat_models:/workspace/persistent_data/models:rw \
    -v nat_ros2:/workspace/persistent_data/ros2_packages:ro \
    nvidia-nat:1.2.1 \
    bash -c ". /workspace/.venv/bin/activate && cd /workspace/mounted_code && uv pip install -e . >/dev/null 2>&1 && exec ${@:-bash}"