"""
Patrol Mission Module
Lap-based patrol mission implementation.
"""

import logging
import os
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.composite_mission import MissionTransitionError
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class PatrolMission(BaseMission):
    """
    Mission: Complete N laps in a pattern.
    """
    
    # Constants
    LAP_COMPLETION_THRESHOLD = 0.5  # Meters - return within this to count lap
    MIN_WAYPOINTS_FOR_LAP = 10  # Minimum waypoints before checking lap completion
    DEFAULT_LAP_COUNT = 5
    
    def _initialize_state(self) -> Dict:
        """Initialize patrol-specific state."""
        return {
            'current_lap': 0,
            'target_laps': self.config.parameters.get('count', self.DEFAULT_LAP_COUNT),
            'start_position': None,
            'lap_waypoints': [],
            'shape': self.config.parameters.get('shape', 'circle')
        }
    
    def _validate_patrol_requirements(self):
        """Validate patrol has map - CHECK MULTIPLE PATHS."""
        
        # FIXED: Check multiple possible paths
        possible_paths = [
            "/workspace/mounted_code/maps/my_map.yaml",
            "/workspace/persistent_data/maps/my_map.yaml",
            "/root/maps/my_map.yaml",
            "/root/my_map.yaml",
            os.path.expanduser("~/my_map.yaml"),
        ]
        
        for map_path in possible_paths:
            if os.path.exists(map_path):
                logger.info(f"[VALIDATION] ✅ Found map: {map_path}")
                return  # Success
        
        # None found - error
        logger.error("[VALIDATION] Map search paths:")
        for path in possible_paths:
            logger.error(f"  ❌ {path}")
        
        raise MissionTransitionError(
            f"Patrol requires map but not found in any location.\n"
            f"Searched: {', '.join(possible_paths[:3])}\n\n"
            f"Fix:\n"
            f"1. Ensure explore mission completed successfully\n"
            f"2. Check SLAM was running during explore\n"
            f"3. Verify map saved to persistent storage"
        )
    
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Update patrol state."""
        if robot_pos:
            self._update_patrol_progress(robot_pos)
        
        # Update progress
        self.state['progress'] = self.state['current_lap'] / self.state['target_laps']
        
        return self.state
    
    def _update_patrol_progress(self, robot_pos: Dict) -> None:
        """Update patrol progress and detect lap completion."""
        # Set start position on first update
        if self.state['start_position'] is None:
            self.state['start_position'] = (robot_pos['x'], robot_pos['y'])
            logger.info(f"[PATROL] Started at {self.state['start_position']}")
            return
        
        # Track waypoints
        self.state['lap_waypoints'].append((robot_pos['x'], robot_pos['y']))
        
        # Check lap completion
        if len(self.state['lap_waypoints']) >= self.MIN_WAYPOINTS_FOR_LAP:
            distance_to_start = self._calculate_distance_to_start(robot_pos)
            
            if distance_to_start < self.LAP_COMPLETION_THRESHOLD:
                self._complete_lap()
    
    def _calculate_distance_to_start(self, robot_pos: Dict) -> float:
        """Calculate distance to start position."""
        return np.sqrt(
            (robot_pos['x'] - self.state['start_position'][0])**2 +
            (robot_pos['y'] - self.state['start_position'][1])**2
        )
    
    def _complete_lap(self) -> None:
        """Mark lap as completed."""
        self.state['current_lap'] += 1
        self.state['lap_waypoints'] = []
        logger.info(
            f"[PATROL] Lap {self.state['current_lap']}/"
            f"{self.state['target_laps']} completed"
        )
    
    def _check_completion(self) -> bool:
        """Check if all laps completed."""
        return self.state['current_lap'] >= self.state['target_laps']
    
    def _get_directive(self) -> str:
        """Get directive based on patrol shape."""
        shape = self.state['shape']
        return f'patrol_{shape}'