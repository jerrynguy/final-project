# pylint: disable=unused-import
# flake8: noqa

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

@register_function(config_type=RobotVisionConfig)
async def robot_vision_controller_wrapper(config, builder):
    """Lazy-loading wrapper for robot_vision_controller."""
    # Import only when called
    from multi_function_agent.robot_vision_controller.main import robot_vision_controller
    
    # Call and delegate generator
    async with robot_vision_controller(config, builder) as result:
        yield result

# Import other functions
try:
    from multi_function_agent import multi_function_agent_function

    from .robot_vision_controller.main import *
    from .robot_vision_controller.test_integration import *

    from .robot_vision_controller.core.models import *
    from .robot_vision_controller.core.ros2_node import *
    from .robot_vision_controller.core.goal_parser import *
    from .robot_vision_controller.core.query_extractor import *
    from .robot_vision_controller.core.mission_controller import *

    from .robot_vision_controller.utils.ros_interface import *
    from .robot_vision_controller.utils.safety_checks import *
    from .robot_vision_controller.utils.geometry_utils import *
    from .robot_vision_controller.utils.movement_commands import *
    from .robot_vision_controller.utils.log.error_handlers import *
    from .robot_vision_controller.utils.log.output_formatter import *
    from .robot_vision_controller.utils.log.performance_logger import *

    from .robot_vision_controller.perception.lidar_monitor import *
    from .robot_vision_controller.perception.spatial_detector import *
    from .robot_vision_controller.perception.rtsp_stream_handler import *
    from .robot_vision_controller.perception.robot_vision_analyzer import *
    
    from .robot_vision_controller.navigation.nav2_interface import *
    from .robot_vision_controller.navigation.navigation_reasoner import *
    from .robot_vision_controller.navigation.robot_controller_interface import *
    
    from multi_function_agent.robot_vision_controller import main
except ImportError:
    pass
