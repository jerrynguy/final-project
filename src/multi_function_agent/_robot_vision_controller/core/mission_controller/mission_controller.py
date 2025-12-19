"""
Mission Controller Module (Refactored)
Orchestrates mission lifecycle using strategy pattern.
"""

import logging
from typing import Dict

from multi_function_agent._robot_vision_controller.core.parser.goal_parser import (
    Mission, 
    parse_mission_from_prompt,
    MissionParsingError,
    UnsupportedMissionError
)
from multi_function_agent._robot_vision_controller.core.mission_controller.mission_validator.mission_validator import (
    MissionValidator,
    MissionRequirementsError
)
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import (
    BaseMission,
    MissionConfig
)
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.follow_mission import FollowMission
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.patrol_mission import PatrolMission
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.explore_mission import ExploreMission
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.composite_mission import CompositeMission

logger = logging.getLogger(__name__)


class MissionController:
    """
    Mission orchestrator using strategy pattern.
    
    Responsibilities:
    - Validate mission requirements
    - Instantiate appropriate mission class
    - Delegate all operations to mission instance
    """
    
    # Mission type registry
    MISSION_CLASSES = {
        'follow_target': FollowMission,
        'patrol_laps': PatrolMission,
        'explore_area': ExploreMission,
        'composite_mission': CompositeMission 
    }
    
    def __init__(self, mission: Mission):
        """
        Initialize mission controller.
        
        Args:
            mission: Parsed mission object
            
        Raises:
            MissionRequirementsError: If requirements not met
        """
        self.mission = mission
        
        # ✨ CHANGED: Skip validation for composite missions
        # Composite missions validate per-step at execution time
        if mission.type != 'composite_mission':
            self._validate_requirements()
        else:
            logger.info("[VALIDATION] Composite mission - per-step validation at runtime")
        
        # Instantiate appropriate mission class
        self._mission_instance = self._create_mission_instance()
        
        logger.info(f"[MISSION] Controller initialized: {mission.description}")

    def get_current_mission_type(self) -> str:
        """
        Get current effective mission type.
        For composite missions, returns current step type.
        """
        if self.mission.type == 'composite_mission':
            from multi_function_agent._robot_vision_controller.core.mission_controller.missions.composite_mission import CompositeMission
            
            if isinstance(self._mission_instance, CompositeMission):
                current_step_id = self._mission_instance.current_step_id
                current_step = self._mission_instance.steps.get(current_step_id)
                
                if current_step:
                    return current_step.type
        
        return self.mission.type

    def has_explore_step(self) -> bool:
        """
        Check if mission contains explore_area step.
        Determines if SLAM needs to be initialized.
        """
        if self.mission.type == 'explore_area':
            return True
        
        if self.mission.type == 'composite_mission':
            from multi_function_agent._robot_vision_controller.core.mission_controller.missions.composite_mission import CompositeMission
            
            if isinstance(self._mission_instance, CompositeMission):
                return any(
                    step.type == 'explore_area' 
                    for step in self._mission_instance.steps.values()
                )
        
        return False
    
    def _validate_requirements(self) -> None:
        """Validate mission requirements using validator."""
        MissionValidator.validate_mission(
            self.mission.type,
            target_class=getattr(self.mission, 'target_class', None)
        )
    
    def _create_mission_instance(self) -> BaseMission:
        """
        Create mission instance based on type.
        
        Returns:
            BaseMission: Concrete mission instance
            
        Raises:
            ValueError: If mission type unknown
        """
        mission_class = self.MISSION_CLASSES.get(self.mission.type)
        
        if not mission_class:
            raise ValueError(f"Unknown mission type: {self.mission.type}")
        
        # Create config
        config = MissionConfig(
            type=self.mission.type,
            description=self.mission.description,
            parameters=self.mission.parameters,
            target_class=getattr(self.mission, 'target_class', None)
        )
        
        return mission_class(config)
    
    # Delegate all operations to mission instance
    
    def update_state(self, detected_objects=None, robot_pos=None, frame_info=None):
        """Delegate to mission instance."""
        return self._mission_instance.update_state(
            detected_objects, robot_pos, frame_info
        )
    
    def check_completion(self) -> bool:
        """Delegate to mission instance."""
        return self._mission_instance.check_completion()
    
    def get_directive(self) -> str:
        """Delegate to mission instance."""
        return self._mission_instance.get_directive()
    
    def process_frame(
        self,
        detected_objects=None,
        robot_pos=None,
        frame_info=None,
        frame=None,
        vision_analyzer=None,
        full_lidar_scan=None
    ):
        """Delegate to mission instance."""
        return self._mission_instance.process_frame(
            detected_objects,
            robot_pos,
            frame_info,
            frame,
            vision_analyzer,
            full_lidar_scan
        )
    
    @property
    def state(self):
        """Expose mission state."""
        return self._mission_instance.state
    
    # Factory method
    
    @classmethod
    async def from_prompt(cls, user_prompt: str, builder):
        """
        Create MissionController by parsing user prompt.
        
        Args:
            user_prompt: Natural language command
            builder: Workflow builder instance
            
        Returns:
            MissionController: Initialized controller
            
        Raises:
            MissionParsingError: If parsing fails
            UnsupportedMissionError: If mission type not supported
            MissionRequirementsError: If requirements not met
        """
        try:
            mission = await parse_mission_from_prompt(user_prompt, builder)
            logger.info(f"[MISSION] Parsed: {mission.type} - {mission.description}")
            
            return cls(mission)
            
        except (MissionParsingError, UnsupportedMissionError, MissionRequirementsError):
            raise
        
        except Exception as e:
            logger.error(f"Unexpected error creating mission: {e}")
            raise MissionParsingError(f"Failed to create mission: {str(e)}")