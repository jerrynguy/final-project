"""
Mission Controller Module
Tracks mission progress and generates navigation directives for robot missions.
"""

import time
import logging
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent.robot_vision_controller.core.goal_parser import Mission, parse_mission_from_prompt

logger = logging.getLogger(__name__)


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
        self.state = self._initialize_state()
        self.start_time = time.time()

    # Class method to create from user prompt
    @classmethod
    async def from_prompt(cls, user_prompt: str, builder):
        """
        Create MissionController by parsing user prompt.
        """
        try:
            mission = await parse_mission_from_prompt(user_prompt, builder)
            logger.info(f"[MISSION] Parsed: {mission.type} - {mission.description}")
            return cls(mission)
        except Exception as e:
            logger.warning(f"Failed to parse mission, using default explore: {e}")
            default_mission = Mission(type='explore_area', description='Default exploration')
            return cls(default_mission)
            # Unified frame processing
            
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
                duration_param = float('inf')

            return {
                **base_state,
                'coverage': self.mission.parameters.get('coverage', 'full'),
                'duration': duration_param,
                'areas_visited': set()
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
                logger.info(f"[MISSION COMPLETE] Explored for {elapsed:.0f}s")
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