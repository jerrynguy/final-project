# pylint: disable=unused-import
# flake8: noqa
import sys
import os
from pathlib import Path

# Ensure mounted code is in path (for hybrid volume strategy)
mounted_code = Path("/workspace/mounted_code")
if mounted_code.exists() and str(mounted_code) not in sys.path:
    sys.path.insert(0, str(mounted_code))

# Ensure persistent ROS2 packages are in path
ros2_packages = Path("/workspace/persistent_data/ros2_packages")
if ros2_packages.exists() and str(ros2_packages) not in sys.path:
    sys.path.insert(0, str(ros2_packages))
    
from nat.cli.register_workflow import register_function
from nat.data_models.function import FunctionBaseConfig
from pydantic import Field

class RobotVisionConfig(FunctionBaseConfig, name="robot_vision_controller"):
    """Configuration for robot vision control system."""
    stream_url: str = Field(default="rtsp://127.0.0.1:8554/robotcam")
    control_mode: str = Field(default="autonomous")
    navigation_goal: str = Field(default="explore")
    safety_level: str = Field(default="high")
    max_speed: float = Field(default=0.5)
    exploration_speed_boost: float = Field(default=1.0, description="Speed multiplier for exploration mode (1.0-2.0)")

@register_function(config_type=RobotVisionConfig)
async def robot_vision_controller_wrapper(config, builder):
    """Lazy-loading wrapper for robot_vision_controller."""
    # Import only when called
    from multi_function_agent._robot_vision_controller.main import _robot_vision_controller
    
    # Call and delegate generator
    async for result in _robot_vision_controller(config, builder):
        yield result