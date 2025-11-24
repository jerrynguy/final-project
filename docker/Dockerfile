# SPDX-FileCopyrightText: Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

ARG BASE_IMAGE_URL=nvcr.io/nvidia/base/ubuntu
ARG BASE_IMAGE_TAG=22.04_20240212
ARG NAT_VERSION=1.0.0
ARG PYTHON_VERSION=3.11
ARG UV_VERSION=0.8.3

FROM --platform=$TARGETPLATFORM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv_base
FROM --platform=$TARGETPLATFORM ${BASE_IMAGE_URL}:${BASE_IMAGE_TAG} AS base

ARG NAT_VERSION
ARG PYTHON_VERSION
ARG UV_VERSION

COPY --from=uv_base /uv /uvx /bin/

ENV PYTHONDONTWRITEBYTECODE=1

# Install certificates
RUN export DEBIAN_FRONTEND=noninteractive && \
    export TZ=Etc/UTC && \
    apt-get update && \
    apt upgrade -y && \
    apt-get install --no-install-recommends -y ca-certificates && \
    apt clean && \
    update-ca-certificates

# Set SSL environment variables
ENV REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

# Set working directory
WORKDIR /workspace

# Install the nvidia-nat package
RUN --mount=type=cache,id=uv_cache,target=/root/.cache/uv,sharing=locked \
    uv venv --python ${PYTHON_VERSION} /workspace/.venv && \
    . /workspace/.venv/bin/activate && \
    uv pip install --prerelease=allow "nvidia-nat[crewai, langchain, llama-index, mem0ai, semantic-kernel, zep-cloud] == ${NAT_VERSION}"
    
# =============================================================================
# Install multi_function_agent package with entry points
# =============================================================================
COPY examples/multi_function_agent /workspace/multi_function_agent
RUN --mount=type=cache,id=uv_cache,target=/root/.cache/uv,sharing=locked \
    . /workspace/.venv/bin/activate && \
    cd /workspace/multi_function_agent && \
    uv pip install -e . && \
    uv pip install opencv-python ultralytics torch torchvision av pafy numpy pillow
    
# =============================================================================
# Hybrid Volume Strategy Setup
# =============================================================================
RUN mkdir -p /workspace/persistent_data/models \
    /workspace/persistent_data/ros2_packages \
    /workspace/mounted_code
    
# Install OpenCV dependencies
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx libglib2.0-0 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Pre-download YOLO model to persistent volume
RUN . /workspace/.venv/bin/activate && \
    python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')" && \
    mv ~/.cache/ultralytics/yolo11n.pt /workspace/persistent_data/models/ 2>/dev/null || \
    mv ~/.local/share/Ultralytics/models/yolo11n.pt /workspace/persistent_data/models/ 2>/dev/null || \
    find ~ -name "yolo11n.pt" -exec mv {} /workspace/persistent_data/models/ \; || \
    echo "Model will be downloaded at runtime"
    
# =============================================================================
# ROS2 Humble Integration - System Python 3.10 (for subprocess calls)
# =============================================================================
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
        ros-humble-rclpy \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-nav-msgs \
        ros-humble-nav2-msgs \
        ros-humble-nav2-simple-commander \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-std-msgs \
        ros-humble-action-msgs \
        ros-humble-fastrtps \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rmw-cyclonedds-cpp \
        python3-transforms3d && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy ROS2 packages to persistent volume after installation
RUN mkdir -p /workspace/persistent_data/ros2_packages && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/geometry_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/sensor_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/nav_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/std_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/action_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/nav2_msgs* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/builtin_interfaces* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true && \
    cp -r /opt/ros/humble/lib/python3.10/site-packages/rosidl_* /workspace/persistent_data/ros2_packages/ 2>/dev/null || true

# Install Python transforms library in venv
RUN --mount=type=cache,id=uv_cache,target=/root/.cache/uv,sharing=locked \
    . /workspace/.venv/bin/activate && \
    uv pip install transforms3d

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ROS2 Environment Variables
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add persistent volumes to Python path at runtime
ENV PYTHONPATH="/workspace/mounted_code:/workspace/persistent_data/ros2_packages:${PYTHONPATH}"
# Inject path into venv sitecustomize.py
RUN echo "import sys; sys.path.insert(0, '/workspace/mounted_code'); sys.path.insert(0, '/workspace/persistent_data/ros2_packages')" \
    > /workspace/.venv/lib/python3.11/site-packages/sitecustomize.py
WORKDIR /workspace/mounted_code

# Environment variables for the venv
ENV PATH="/workspace/.venv/bin:$PATH"

# ENTRYPOINT allows bash override for debugging
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["nat"]
