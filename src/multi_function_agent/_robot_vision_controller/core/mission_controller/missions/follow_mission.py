"""
Follow Mission Module
Target tracking mission implementation.
"""

import time
import logging
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class FollowMission(BaseMission):
    """
    Mission: Track and follow a moving target (dog-like behavior).
    """
    
    # Constants (extracted from magic numbers)
    LOST_THRESHOLD = 3.0  # Seconds before entering search mode
    MIN_DISTANCE_DEFAULT = 1.0  # Meters
    MAX_DISTANCE_DEFAULT = 2.5  # Meters
    TRACKING_DURATION_TARGET = 60.0  # Seconds for 100% progress
    FRAME_WIDTH_DEFAULT = 640  # Pixels
    
    def _initialize_state(self) -> Dict:
        """Initialize follow-specific state."""
        return {
            'target_visible': False,
            'target_position': None,
            'last_seen_time': None,
            'last_seen_position': None,
            'predicted_direction': None,
            'lost_duration': 0.0,
            'tracking_duration': 0.0,
            'min_distance': self.config.parameters.get('min_distance', self.MIN_DISTANCE_DEFAULT),
            'max_distance': self.config.parameters.get('max_distance', self.MAX_DISTANCE_DEFAULT)
        }
    
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Update target tracking state."""
        current_time = time.time()
        
        # Search for target in detections
        target_found = False
        if detected_objects:
            for obj in detected_objects:
                if obj.get('class') == self.config.target_class:
                    target_found = True
                    self._process_target_detection(obj, current_time, frame_info)
                    break
        
        if not target_found:
            self._process_target_lost(current_time)
        
        # Update progress (based on tracking duration)
        self.state['progress'] = min(
            1.0, 
            self.state['tracking_duration'] / self.TRACKING_DURATION_TARGET
        )
        
        return self.state
    
    def _process_target_detection(
        self,
        obj: Dict,
        current_time: float,
        frame_info: Dict
    ) -> None:
        """Process detected target."""
        self.state['target_visible'] = True
        self.state['target_position'] = obj
        self.state['last_seen_time'] = current_time
        self.state['last_seen_position'] = obj.get('center', obj.get('bbox'))
        self.state['lost_duration'] = 0.0
        self.state['tracking_duration'] += 0.1
        
        # Estimate distance from bbox size
        bbox = obj.get('bbox', [0, 0, 100, 100])
        bbox_height = bbox[3] - bbox[1]
        estimated_distance = max(0.5, 500 / max(bbox_height, 1))
        self.state['target_distance'] = estimated_distance
        
        # Predict direction from bbox position
        if frame_info:
            frame_width = frame_info.get('width', self.FRAME_WIDTH_DEFAULT)
            center_x = (bbox[0] + bbox[2]) / 2
            
            if center_x < frame_width * 0.4:
                self.state['predicted_direction'] = 'left'
            elif center_x > frame_width * 0.6:
                self.state['predicted_direction'] = 'right'
            else:
                self.state['predicted_direction'] = 'forward'
        
        logger.info(
            f"[FOLLOW] Tracking {self.config.target_class}: "
            f"~{estimated_distance:.1f}m, "
            f"{self.state['predicted_direction']}"
        )
    
    def _process_target_lost(self, current_time: float) -> None:
        """Process lost target."""
        self.state['target_visible'] = False
        if self.state['last_seen_time']:
            self.state['lost_duration'] = current_time - self.state['last_seen_time']
            logger.info(
                f"[FOLLOW] Target lost {self.state['lost_duration']:.1f}s, "
                f"last direction: {self.state['predicted_direction']}"
            )
    
    def _check_completion(self) -> bool:
        """Follow missions never complete (continuous tracking)."""
        return False
    
    def _get_directive(self) -> str:
        """Get directive based on tracking state."""
        if self.state['target_visible']:
            return self._get_tracking_directive()
        else:
            return self._get_search_directive()
    
    def _get_tracking_directive(self) -> str:
        """Get directive when target is visible."""
        distance = self.state.get('target_distance', 2.0)
        
        if distance < self.state['min_distance']:
            return 'track_backup'
        elif distance > self.state['max_distance']:
            return 'track_approach'
        else:
            return 'track_follow'
    
    def _get_search_directive(self) -> str:
        """Get directive when target is lost."""
        if self.state['lost_duration'] < self.LOST_THRESHOLD:
            # Recently lost - continue in predicted direction
            direction = self.state.get('predicted_direction', 'forward')
            return f'track_search_{direction}'
        else:
            # Lost for long - search pattern
            return 'track_search_spin'