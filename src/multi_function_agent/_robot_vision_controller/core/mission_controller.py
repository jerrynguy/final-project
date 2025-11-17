"""
Mission Controller Module
Tracks mission progress and generates navigation directives for robot missions.
"""

import subprocess
import time
import logging
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.goal_parser import (
    Mission, 
    parse_mission_from_prompt,
    MissionParsingError,
    UnsupportedMissionError
)

logger = logging.getLogger(__name__)

class MissionRequirementsError(Exception):
    """Raised when mission requirements are not met."""
    pass

# =============================================================================
# Mission Controller
# =============================================================================

class MissionController:
    """
    Tracks mission progress and generates navigation directives.
    """
    
    def __init__(self, mission: Mission):
        """
        Initialize mission controller.
        """
        self.mission = mission
        self._validate_mission_requirements()
        self.state = self._initialize_state()
        self.start_time = time.time()

    def _validate_mission_requirements(self):
        """
        Validate that system meets mission requirements.
        
        Raises:
            MissionRequirementsError: If requirements not met
        """
        import os
        
        mission_type = self.mission.type
        
        if mission_type == 'explore_area':
            # SLAM requirement - NOW SUPPORTED!
            # Just log that SLAM will be started
            logger.info("[MISSION VALIDATION] Explore mission will start SLAM mapping")
            
            # Check if slam_toolbox is installed
            try:
                result = subprocess.run(
                    ['ros2', 'pkg', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=3.0
                )
                
                if 'slam_toolbox' not in result.stdout:
                    raise MissionRequirementsError(
                        "Mission 'explore_area' requires slam_toolbox package. "
                        "Install: sudo apt install ros-humble-slam-toolbox"
                    )
            except Exception as e:
                logger.warning(f"Could not verify slam_toolbox: {e}")
        
        elif mission_type == 'patrol_laps':
            # Map file requirement
            map_path = os.path.expanduser("~/my_map.yaml")
            
            if not os.path.exists(map_path):
                raise MissionRequirementsError(
                    f"Mission 'patrol_laps' requires a pre-built map at {map_path}. "
                    f"Map not found. Please run 'explore_area' mission first to create map, "
                    f"or create map manually using SLAM:\n"
                    f"1. ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True\n"
                    f"2. ros2 run turtlebot3_teleop teleop_keyboard\n"
                    f"3. ros2 run nav2_map_server map_saver_cli -f ~/my_map"
                )
            
            logger.info(f"[MISSION VALIDATION] Map found: {map_path}")
        
        elif mission_type == 'follow_target':
            # Check target_class provided
            if not self.mission.target_class:
                raise MissionRequirementsError(
                    "Mission 'follow_target' requires a target_class. "
                    "Example: 'Follow the person' or 'Follow the dog'"
                )
            
            logger.info(f"[MISSION VALIDATION] Target tracking: {self.mission.target_class}")
        
        else:
            raise MissionRequirementsError(f"Unknown mission type: {mission_type}")

    # Class method to create from user prompt
    @classmethod
    async def from_prompt(cls, user_prompt: str, builder):
        """
        Create MissionController by parsing user prompt.
        """
        try:
            mission = await parse_mission_from_prompt(user_prompt, builder)
            logger.info(f"[MISSION] Parsed: {mission.type} - {mission.description}")
            
            # Validation happens in __init__()
            return cls(mission)
            
        except (MissionParsingError, UnsupportedMissionError, MissionRequirementsError):
            # Re-raise to be handled by main.py
            raise
        
        except Exception as e:
            logger.error(f"Unexpected error creating mission controller: {e}")
            raise MissionParsingError(f"Failed to create mission: {str(e)}")
            
    def process_frame(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None
    ) -> Dict:
        """
        Process frame and update mission state in one call.
        """
        # Update state
        state = self.update_state(detected_objects, robot_pos, frame_info)
        
        # Check completion
        completed = self.check_completion()
        
        # Get directive
        directive = self.get_directive()
        
        return {
            'state': state,
            'completed': completed,
            'directive': directive,
            'progress': state.get('progress', 0.0)
        }
    
    def _initialize_state(self) -> Dict:
        """
        Initialize state dictionary based on mission type.
        """
        base_state = {
            'status': 'active',
            'progress': 0.0,
            'completed': False
        }
        
        if self.mission.type == 'follow_target':
            return {
                **base_state,
                'target_visible': False,
                'target_position': None,  # {x, y, bbox}
                'last_seen_time': None,
                'last_seen_position': None,
                'predicted_direction': None,  # 'forward', 'left', 'right'
                'lost_duration': 0.0,
                'tracking_duration': 0.0,
                'min_distance': self.mission.parameters.get('min_distance', 1.0),
                'max_distance': self.mission.parameters.get('max_distance', 2.5)
            }
        
        elif self.mission.type == 'patrol_laps':
            return {
                **base_state,
                'current_lap': 0,
                'target_laps': self.mission.parameters.get('count', 5),
                'start_position': None,
                'lap_waypoints': [],  # Track positions to detect lap completion
                'shape': self.mission.parameters.get('shape', 'circle')
            }
        
        else:  # explore_area
            duration_param = self.mission.parameters.get('duration')
            if duration_param is None:
                duration_param = 60.0

            return {
                **base_state,
                'coverage': self.mission.parameters.get('coverage', 'full'),
                'duration': duration_param,
                'areas_visited': set(),
                'slam_enabled': True,
                'map_saved': False,
                'mapping_completed': False
            }
    
    def update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None
    ) -> Dict:
        """
        Update mission state based on new observations.
        """        
        if self.mission.type == 'follow_target':
            return self._update_follow_state(detected_objects, frame_info)
        
        elif self.mission.type == 'patrol_laps':
            return self._update_patrol_state(robot_pos)
        
        else:  # explore_area
            return self._update_explore_state(robot_pos)
    
    def _update_follow_state(self, detected_objects: List[Dict], frame_info: Dict) -> Dict:
        """
        Update target following state (dog-like tracking behavior).
        """
        current_time = time.time()
        
        # Find target in detected objects
        target_found = False
        if detected_objects:
            for obj in detected_objects:
                if obj.get('class') == self.mission.target_class:
                    target_found = True
                    self.state['target_visible'] = True
                    self.state['target_position'] = obj
                    self.state['last_seen_time'] = current_time
                    self.state['last_seen_position'] = obj.get('center', obj.get('bbox'))
                    self.state['lost_duration'] = 0.0
                    self.state['tracking_duration'] += 0.1
                    
                    # Estimate distance from bbox size (rough approximation)
                    bbox = obj.get('bbox', [0, 0, 100, 100])
                    bbox_height = bbox[3] - bbox[1]
                    estimated_distance = max(0.5, 500 / max(bbox_height, 1))  # Inverse relation
                    
                    self.state['target_distance'] = estimated_distance
                    
                    # Predict direction based on bbox position in frame
                    if frame_info:
                        frame_width = frame_info.get('width', 640)
                        center_x = (bbox[0] + bbox[2]) / 2
                        
                        if center_x < frame_width * 0.4:
                            self.state['predicted_direction'] = 'left'
                        elif center_x > frame_width * 0.6:
                            self.state['predicted_direction'] = 'right'
                        else:
                            self.state['predicted_direction'] = 'forward'
                    
                    logger.info(
                        f"[MISSION] Tracking {self.mission.target_class}: "
                        f"distance ~{estimated_distance:.1f}m, "
                        f"direction {self.state['predicted_direction']}"
                    )
                    break
        
        if not target_found:
            self.state['target_visible'] = False
            if self.state['last_seen_time']:
                self.state['lost_duration'] = current_time - self.state['last_seen_time']
                logger.info(
                    f"[MISSION] Target lost for {self.state['lost_duration']:.1f}s, "
                    f"last direction: {self.state['predicted_direction']}"
                )
        
        # Progress based on tracking duration (track for 60s)
        self.state['progress'] = min(1.0, self.state['tracking_duration'] / 60.0)
        
        return self.state
    
    def _update_patrol_state(self, robot_pos: Dict) -> Dict:
        """
        Update patrol laps state.
        """
        if robot_pos and self.state['start_position'] is None:
            self.state['start_position'] = (robot_pos['x'], robot_pos['y'])
            logger.info(f"[MISSION] Patrol started at {self.state['start_position']}")
        
        # Detect lap completion (simple: return to start position within 0.5m)
        if robot_pos and self.state['start_position']:
            distance_to_start = np.sqrt(
                (robot_pos['x'] - self.state['start_position'][0])**2 +
                (robot_pos['y'] - self.state['start_position'][1])**2
            )
            
            if distance_to_start < 0.5 and len(self.state['lap_waypoints']) > 10:
                self.state['current_lap'] += 1
                self.state['lap_waypoints'] = []
                logger.info(
                    f"[MISSION] Lap {self.state['current_lap']}/"
                    f"{self.state['target_laps']} completed"
                )
            else:
                self.state['lap_waypoints'].append((robot_pos['x'], robot_pos['y']))
        
        self.state['progress'] = self.state['current_lap'] / self.state['target_laps']
        
        return self.state
    
    def _update_explore_state(self, robot_pos: Dict) -> Dict:
        """
        Update exploration state.
        """
        elapsed = time.time() - self.start_time
        duration = self.state['duration']
        
        if duration is None or duration == float('inf'):
            self.state['progress'] = 0.0
        else:
            self.state['progress'] = min(1.0, elapsed / duration)

        # Track areas visited (grid-based)
        if robot_pos:
            grid_size = 1.0  # 1m x 1m grid cells
            grid_x = int(robot_pos['x'] / grid_size)
            grid_y = int(robot_pos['y'] / grid_size)
            self.state['areas_visited'].add((grid_x, grid_y))
            
            # Log coverage
            if len(self.state['areas_visited']) % 10 == 0:
                logger.info(f"[EXPLORATION] Covered {len(self.state['areas_visited'])} areas")
            
        return self.state
    
    def check_completion(self) -> bool:
        """
        Check if mission is completed.
        """
        if self.state['completed']:
            return True
        
        if self.mission.type == 'follow_target':
            # Never complete (continuous tracking)
            return False
        
        elif self.mission.type == 'patrol_laps':
            if self.state['current_lap'] >= self.state['target_laps']:
                self.state['completed'] = True
                logger.info(f"[MISSION COMPLETE] Finished {self.state['current_lap']} laps")
                return True
        
        elif self.mission.type == 'explore_area':
            elapsed = time.time() - self.start_time
            duration = self.state['duration']
    
            # FIX: Handle None or inf duration
            if duration is None or duration == float('inf'):
                return False  # Never complete if no duration limit
            
            if elapsed >= self.state['duration']:
                self.state['completed'] = True
                self.state['mapping_complete'] = True
                
                areas_count = len(self.state['areas_visited'])
                logger.info(f"[MISSION COMPLETE] Explored {areas_count} areas in {elapsed:.0f}s")
                return True
        
        return False
    
    def get_directive(self) -> str:
        """
        Generate navigation directive based on current mission state.
        """        
        if self.mission.type == 'follow_target':
            if self.state['target_visible']:
                distance = self.state.get('target_distance', 2.0)
                if distance < self.state['min_distance']:
                    return 'track_backup'  # Too close, backup
                elif distance > self.state['max_distance']:
                    return 'track_approach'  # Too far, approach
                else:
                    return 'track_follow'  # Perfect distance, follow
            else:
                # Target lost - predict behavior
                if self.state['lost_duration'] < 3.0:
                    # Recently lost - continue in predicted direction
                    direction = self.state.get('predicted_direction', 'forward')
                    return f'track_search_{direction}'
                else:
                    # Lost for long - search pattern
                    return 'track_search_spin'
        
        elif self.mission.type == 'patrol_laps':
            shape = self.state['shape']
            return f'patrol_{shape}'
        
        else:  # explore_area
            return 'explore_random'