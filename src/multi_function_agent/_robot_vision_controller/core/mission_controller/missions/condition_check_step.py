"""
Condition Check Step Module
Evaluate conditions for conditional branching in composite missions.
"""

import time
import logging
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import (
    BaseMission, MissionConfig
)

logger = logging.getLogger(__name__)


class ConditionCheckMission(BaseMission):
    """
    Mission: Evaluate condition and determine next step.
    
    Condition types:
    - object_detected: Check if YOLO detected target object
    - timeout: Wait for specified duration
    - distance_traveled: Check if robot moved X meters
    - area_covered: Check if explored X% of area (requires SLAM)
    """
    
    # Constants
    DEFAULT_TIMEOUT = 30.0  # seconds
    CHECK_INTERVAL = 0.5    # Check every 0.5s
    
    def _initialize_state(self) -> Dict:
        """Initialize condition check state."""
        # Extract condition config
        condition_config = self.config.parameters.get('condition', {})
        
        self.condition_type = condition_config.get('type', 'timeout')
        self.condition_params = condition_config
        self.timeout = condition_config.get('timeout', self.DEFAULT_TIMEOUT)
        
        # Branch targets
        self.branch_true = self.config.parameters.get('branch_true', 'mission_complete')
        self.branch_false = self.config.parameters.get('branch_false', 'mission_complete')
        
        # Evaluation state
        self.condition_result = None  # None = pending, True/False = decided
        self.check_start_time = time.time()
        self.last_check_time = 0.0
        
        # Distance tracking (for distance_traveled condition)
        self.start_position = None
        self.distance_traveled = 0.0
        
        logger.info(
            f"[CONDITION] Type: {self.condition_type}, "
            f"timeout: {self.timeout}s, "
            f"branches: True→{self.branch_true}, False→{self.branch_false}"
        )
        
        return {
            'condition_type': self.condition_type,
            'condition_result': None,
            'check_duration': 0.0,
            'timeout': self.timeout,
            'branch_true': self.branch_true,
            'branch_false': self.branch_false
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
        """Evaluate condition periodically."""
        
        current_time = time.time()
        elapsed = current_time - self.check_start_time
        
        # Rate limit checks
        if current_time - self.last_check_time < self.CHECK_INTERVAL:
            return self.state
        
        self.last_check_time = current_time
        
        # Check if already decided
        if self.condition_result is not None:
            return self.state
        
        # Evaluate condition based on type
        if self.condition_type == 'object_detected':
            self.condition_result = self._check_object_detected(
                detected_objects, frame, vision_analyzer,full_lidar_scan
            )
        
        elif self.condition_type == 'timeout':
            # Timeout condition just waits - always returns False
            if elapsed >= self.timeout:
                self.condition_result = False
                logger.info(f"[CONDITION] Timeout after {elapsed:.1f}s")
        
        elif self.condition_type == 'distance_traveled':
            self.condition_result = self._check_distance_traveled(robot_pos)
        
        elif self.condition_type == 'area_covered':
            self.condition_result = self._check_area_covered()
        
        else:
            logger.error(f"Unknown condition type: {self.condition_type}")
            self.condition_result = False
        
        # Update state
        self.state['condition_result'] = self.condition_result
        self.state['check_duration'] = elapsed
        self.state['progress'] = min(1.0, elapsed / self.timeout)
        
        # Log result if decided
        if self.condition_result is not None:
            result_str = "MET" if self.condition_result else "NOT MET"
            next_step = self.branch_true if self.condition_result else self.branch_false
            logger.info(
                f"[CONDITION] {result_str} after {elapsed:.1f}s → "
                f"Next: '{next_step}'"
            )
        
        return self.state
    
    def _check_object_detected(
            self, 
            detected_objects: Optional[List[Dict]],
            frame = None,
            vision_analyzer = None,
            full_lidar_scan = None
        ) -> Optional[bool]:
        """Check if target object detected."""
        target_class = self.condition_params.get('target_class')
        
        if not target_class:
            logger.error("[CONDITION] object_detected requires target_class")
            return False
        
        # Priority 1: Use pre-detected objects from main.py
        if detected_objects:
            detected = any(
                obj.get('class') == target_class 
                for obj in detected_objects    
            )

            if detected:
                logger.info(f"[CONDITION] ✓ Detected target object '{target_class}'")
                return True
            
        # Priority 2: On-demand detection if frame + analyzer if available
        if frame is not None and vision_analyzer is not None:
            logger.info(f"[CONDITION] Running on-demand YOLO for: {target_class}")
            analysis_results = vision_analyzer.analyze_frame(frame, full_lidar_scan=full_lidar_scan)

            detected_now = vision_analyzer.detect_target_objects(
                frame, target_class,
                confident_threshold=0.6,
                full_lidar_scan=full_lidar_scan
            )

            if detected_now:
                logger.info(f"[CONDITION] ✓ Detected target object '{target_class}' (on-demand)")
                return True
        
        return None  # Keep checking
    
    def _check_distance_traveled(self, robot_pos: Optional[Dict]) -> Optional[bool]:
        """Check if robot traveled target distance."""
        target_distance = self.condition_params.get('distance', 5.0)
        
        if not robot_pos:
            return None
        
        # Track start position
        if self.start_position is None:
            self.start_position = (robot_pos['x'], robot_pos['y'])
            logger.info(f"[CONDITION] Distance check start: {self.start_position}")
            return None
        
        # Calculate distance
        dx = robot_pos['x'] - self.start_position[0]
        dy = robot_pos['y'] - self.start_position[1]
        self.distance_traveled = np.sqrt(dx**2 + dy**2)
        
        if self.distance_traveled >= target_distance:
            logger.info(
                f"[CONDITION] ✓ Traveled {self.distance_traveled:.2f}m "
                f"(target: {target_distance}m)"
            )
            return True
        
        return None  # Keep checking
    
    def _check_area_covered(self) -> Optional[bool]:
        """Check if explored target % of area."""
        # TODO: Integrate with SLAM map coverage calculation
        target_coverage = self.condition_params.get('coverage', 0.8)
        
        # Placeholder - would need SLAM integration
        logger.warning("[CONDITION] area_covered not implemented yet")
        return False
    
    def _check_completion(self) -> bool:
        """Condition check completes when result is decided OR timeout."""
        elapsed = time.time() - self.check_start_time
        
        # Complete if result decided
        if self.condition_result is not None:
            return True
        
        # Complete if timeout reached
        if elapsed >= self.timeout:
            self.condition_result = False
            logger.info(f"[CONDITION] Timeout reached ({elapsed:.1f}s)")
            return True
        
        return False
    
    def _get_directive(self) -> str:
        """Directive during condition check (usually stay still or continue exploration)."""
        # Could be 'condition_wait' or 'condition_explore' depending on context
        return 'condition_checking'
    
    def get_next_step_id(self) -> str:
        """Get next step ID based on condition result."""
        if self.condition_result is True:
            return self.branch_true
        else:
            return self.branch_false