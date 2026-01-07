"""
Directional Command Step Module
Standalone mission step for simple directional movements.
"""

import time
import logging
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import (
    BaseMission, MissionConfig
)

logger = logging.getLogger(__name__)


class DirectionalCommandMission(BaseMission):
    """
    Mission: Execute simple directional command (turn left/right, go forward/backward).
    
    Use cases:
    - "Turn left" → rotate left for 2s
    - "Go forward 3 meters" → move forward until 3m traveled
    - "Back up a bit" → move backward for 1s
    """
    
    # Constants
    DEFAULT_TURN_DURATION = 2.0  # seconds
    DEFAULT_MOVE_DURATION = 3.0  # seconds
    DEFAULT_LINEAR_SPEED = 0.3   # m/s
    DEFAULT_ANGULAR_SPEED = 0.6  # rad/s
    
    def _initialize_state(self) -> Dict:
        """Initialize directional command state."""
        # Extract parameters
        self.direction = self.config.parameters.get('direction', 'forward')
        self.distance = self.config.parameters.get('distance')  # meters (optional)
        self.duration = self.config.parameters.get('duration')  # seconds (optional)
        
        # Auto-determine duration if not specified
        if not self.duration:
            if self.direction in ['left', 'right']:
                self.duration = self.DEFAULT_TURN_DURATION
            else:
                self.duration = self.DEFAULT_MOVE_DURATION
        
        # Tracking
        self.start_position = None
        self.last_robot_pos = None
        self.distance_traveled = 0.0
        
        logger.info(
            f"[DIRECTIONAL] Command: {self.direction}, "
            f"duration={self.duration}s, distance={self.distance}m"
        )
        
        return {
            'direction': self.direction,
            'distance_traveled': 0.0,
            'target_distance': self.distance,
            'duration': self.duration
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
        """Update movement progress."""
        
        # Track start position
        if robot_pos and self.start_position is None:
            self.start_position = robot_pos.copy()  # ✅ THAY ĐỔI: Copy dict thay vì tuple
            logger.info(f"[DIRECTIONAL] Start position: {self.start_position}")
        
        # ✅ THÊM: Track current position for rotation check
        if robot_pos:
            self.last_robot_pos = robot_pos.copy()
        
        # Calculate distance traveled (for linear movement)
        if robot_pos and self.start_position:
            dx = robot_pos['x'] - self.start_position['x']
            dy = robot_pos['y'] - self.start_position['y']
            self.distance_traveled = np.sqrt(dx**2 + dy**2)
            
            self.state['distance_traveled'] = self.distance_traveled
        
        # Update progress
        if self.distance:
            # Distance-based progress
            self.state['progress'] = min(1.0, self.distance_traveled / self.distance)
        else:
            # Time-based progress
            elapsed = self.get_elapsed_time()
            self.state['progress'] = min(1.0, elapsed / self.duration)
        
        return self.state

    def _check_completion(self) -> bool:
        """Check if directional command completed."""
        elapsed = self.get_elapsed_time()
        
        # ✅ PRIORITY 1: Check if movement goal achieved
        # (Kiểm tra robot đã di chuyển đủ chưa TRƯỚC KHI check timeout)
        
        if self.direction in ['left', 'right']:
            # Rotation commands: check angle rotated
            if self.start_position and hasattr(self, 'last_robot_pos'):
                start_theta = self.start_position.get('theta', 0) if isinstance(self.start_position, dict) else 0
                current_theta = self.last_robot_pos.get('theta', 0)
                
                # Calculate angle difference (handle wraparound)
                angle_rotated = abs(current_theta - start_theta)
                if angle_rotated > np.pi:
                    angle_rotated = 2 * np.pi - angle_rotated
                
                # Expected rotation: duration * angular_speed
                expected_rotation = self.duration * self.DEFAULT_ANGULAR_SPEED * 0.8  # 80% threshold
                
                if angle_rotated >= expected_rotation:
                    logger.info(
                        f"[DIRECTIONAL] Complete: Rotated {np.degrees(angle_rotated):.1f}° "
                        f"(expected: {np.degrees(expected_rotation):.1f}°)"
                    )
                    return True
        
        elif self.direction in ['forward', 'backward']:
            # Linear commands: check distance traveled
            if self.distance:
                # Distance-based completion
                if self.distance_traveled >= self.distance * 0.9:  # 90% threshold
                    logger.info(
                        f"[DIRECTIONAL] Complete: {self.distance_traveled:.2f}m traveled "
                        f"(target: {self.distance}m)"
                    )
                    return True
            else:
                # Duration-based, but check if moved enough
                expected_distance = self.duration * self.DEFAULT_LINEAR_SPEED * 0.7  # 70% threshold
                if self.distance_traveled >= expected_distance:
                    logger.info(
                        f"[DIRECTIONAL] Complete: {self.distance_traveled:.2f}m traveled "
                        f"(expected: {expected_distance:.2f}m)"
                    )
                    return True
        
        # ✅ PRIORITY 2: Timeout fallback
        # (Chỉ khi robot KHÔNG di chuyển đủ mới dùng timeout)
        if elapsed >= self.duration:
            logger.info(f"[DIRECTIONAL] Complete: {elapsed:.1f}s elapsed (timeout)")
            return True
        
        return False
    
    def _get_directive(self) -> str:
        """Get navigation directive for this command."""
        # Map direction to navigation directive
        directive_map = {
            'forward': 'move_forward',
            'backward': 'move_backward',
            'left': 'turn_left',
            'right': 'turn_right'
        }
        
        directive = directive_map.get(self.direction, 'directional_forward')
        return f'directional_{directive}'