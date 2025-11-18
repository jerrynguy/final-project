#!/bin/bash
set -e

echo "🚀 Initializing persistent volumes for Hybrid Strategy..."

# Create volumes
docker volume create nat_models
docker volume create nat_ros2

# Copy model from image to volume
echo "📦 Copying YOLO model to persistent volume..."
docker run --rm \
    -v nat_models:/target \
    nvidia-nat:1.2.1 \
    sh -c "cp /workspace/persistent_data/models/yolo11n.pt /target/ 2>/dev/null || echo '✓ Model already exists'"

# Copy ROS2 packages from image to volume
echo "📦 Copying ROS2 packages to persistent volume..."
docker run --rm \
    -v nat_ros2:/target \
    nvidia-nat:1.2.1 \
    sh -c "cp -r /workspace/persistent_data/ros2_packages/* /target/ 2>/dev/null || echo '✓ ROS2 packages already exist'"

echo "✅ Volumes initialized!"
echo ""
echo "Volume locations:"
docker volume inspect nat_models -f '  Models: {{.Mountpoint}}'
docker volume inspect nat_ros2 -f '  ROS2: {{.Mountpoint}}'
echo ""
echo "To clean up: docker volume rm nat_models nat_ros2"