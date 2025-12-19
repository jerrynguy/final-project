"""
Mission Validator Module
Validates system requirements for different mission types.
"""

import os
import logging
import subprocess
from typing import Tuple
from functools import lru_cache

logger = logging.getLogger(__name__)


class MissionRequirementsError(Exception):
    """Raised when mission requirements are not met."""
    pass


class MissionValidator:
    """
    Validates system requirements for missions.
    Uses caching to avoid repeated subprocess calls.
    """
    
    @staticmethod
    @lru_cache(maxsize=1)
    def check_slam_available() -> bool:
        """
        Check if SLAM Toolbox is available (cached).
        """
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            return 'slam_toolbox' in result.stdout
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    @staticmethod
    def check_map_exists(map_path: str = "~/my_map.yaml") -> Tuple[bool, str]:
        """
        Check if map file exists in multiple locations.
        """
        # Try both container and host paths
        container_map = "/workspace/my_map.yaml"
        host_map = os.path.expanduser(map_path)
        
        if os.path.exists(container_map):
            return True, container_map
        elif os.path.exists(host_map):
            return True, host_map
        else:
            return False, map_path
    
    @staticmethod
    def validate_explore_mission() -> None:
        """
        Validate requirements for explore_area mission.
        """
        logger.info("[MISSION VALIDATION] Explore mission will start SLAM mapping")
        logger.warning("=" * 60)
        logger.warning("[MISSION VALIDATION] Explore mission requires SLAM")
        logger.warning("=" * 60)
        logger.warning("SLAM Toolbox must be running on HOST before starting:")
        logger.warning("")
        logger.warning("  Terminal 1 (HOST):")
        logger.warning("    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True")
        logger.warning("")
        logger.warning("  Wait 3 seconds, then start this mission.")
        logger.warning("=" * 60)

        # Verify slam_toolbox exists on host
        slam_check = input("Is SLAM Toolbox running on host? (y/n): ").lower()
        if slam_check != 'y':
            raise MissionRequirementsError(
                "Explore mission requires SLAM Toolbox running on host.\n"
                "Start: ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
            )
        
        logger.info("[MISSION VALIDATION] ✅ User confirmed SLAM running")
    
    @staticmethod
    def validate_patrol_mission() -> None:
        """
        Validate requirements for patrol_laps mission.
        """
        map_exists, map_path = MissionValidator.check_map_exists()
        
        if not map_exists:
            raise MissionRequirementsError(
                f"Mission 'patrol_laps' requires a pre-built map at {map_path}. "
                f"Map not found. Please run 'explore_area' mission first to create map, "
                f"or create map manually using SLAM:\n"
                f"\n"
                f"HOST Terminal 1:\n"
                f"  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True\n"
                f"\n"
                f"HOST Terminal 2:\n"
                f"  ros2 run turtlebot3_teleop teleop_keyboard\n"
                f"  # Drive around for 60+ seconds\n"
                f"\n"
                f"HOST Terminal 3:\n"
                f"  ros2 run nav2_map_server map_saver_cli -f ~/my_map\n"
                f"\n"
                f"Then restart container with map mounted."
            )
        
        logger.info(f"[MISSION VALIDATION] ✅ Map found: {map_path}")
    
    @staticmethod
    def validate_follow_mission(target_class: str) -> None:
        """
        Validate requirements for follow_target mission.
        """
        if not target_class:
            raise MissionRequirementsError(
                "Mission 'follow_target' requires a target_class. "
                "Example: 'Follow the person' or 'Follow the dog'"
            )
        
        logger.info(f"[MISSION VALIDATION] ✅ Target tracking: {target_class}")
    
    @staticmethod
    def validate_mission(mission_type: str, **kwargs) -> None:
        """
        Validate mission requirements based on type.
        """
        validators = {
            'explore_area': MissionValidator.validate_explore_mission,
            'patrol_laps': MissionValidator.validate_patrol_mission,
            'follow_target': lambda: MissionValidator.validate_follow_mission(
                kwargs.get('target_class')
            )
        }
        
        validator = validators.get(mission_type)
        if validator:
            validator()
        else:
            raise MissionRequirementsError(f"Unknown mission type: {mission_type}")