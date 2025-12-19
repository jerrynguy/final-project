"""
Base Mission Module
Abstract base class for all mission types.
"""

import time
import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class MissionConfig:
    """Base configuration for all missions."""
    type: str
    description: str
    parameters: Dict
    target_class: Optional[str] = None


class BaseMission(ABC):
    """
    Abstract base class for robot missions.
    
    All missions must implement:
    - _initialize_state(): Setup initial state
    - _update_state(): Process observations
    - _check_completion(): Determine if done
    - _get_directive(): Return navigation command
    """
    
    def __init__(self, config: MissionConfig):
        """
        Initialize mission.
        """
        self.config = config
        self.state = self._create_base_state()
        self.state.update(self._initialize_state())
        self.start_time = time.time()
        
        logger.info(f"[MISSION] Initialized: {config.description}")
    
    def _create_base_state(self) -> Dict:
        """
        Create base state shared by all missions.
        """
        return {
            'status': 'active',
            'progress': 0.0,
            'completed': False
        }
    
    @abstractmethod
    def _initialize_state(self) -> Dict:
        """
        Initialize mission-specific state.
        """
        pass
    
    @abstractmethod
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """
        Update mission state based on observations.
        """
        pass
    
    @abstractmethod
    def _check_completion(self) -> bool:
        """
        Check if mission is completed.
        """
        pass
    
    @abstractmethod
    def _get_directive(self) -> str:
        """
        Get current navigation directive.
        """
        pass
    
    # Public interface (delegates to abstract methods)
    
    def update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Public: Update state."""
        return self._update_state(
            detected_objects, robot_pos, frame_info, 
            frame, vision_analyzer, full_lidar_scan)
    
    def check_completion(self) -> bool:
        """Public: Check completion."""
        if self.state['completed']:
            return True
        
        completed = self._check_completion()
        if completed:
            self.state['completed'] = True
            logger.info(f"[MISSION COMPLETE] {self.config.description}")
        
        return completed
    
    def get_directive(self) -> str:
        """Public: Get directive."""
        return self._get_directive()
    
    def process_frame(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """
        Process frame: update + check + directive in one call.
        """
        state = self.update_state(
            detected_objects, robot_pos, frame_info, 
            frame, vision_analyzer, full_lidar_scan
        )
        completed = self.check_completion()
        directive = self.get_directive()
        
        return {
            'state': state,
            'completed': completed,
            'directive': directive,
            'progress': state.get('progress', 0.0)
        }
    
    def get_elapsed_time(self) -> float:
        """Get mission elapsed time in seconds."""
        return time.time() - self.start_time