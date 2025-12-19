"""
Movement Commands Module - Enhanced Version
Navigation command generation with mission-aware strategies.
"""

import logging
import random
import numpy as np
from enum import Enum
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)


# Navigation Action Enumeration
class NavigationAction(Enum):
    """Available navigation actions for robot control."""
    STOP = "stop"
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    SLOW_FORWARD = "slow_forward"
    EMERGENCY_STOP = "emergency_stop"


# Navigation Parameters
@dataclass
class NavigationParameters:
    """Navigation speed and threshold parameters."""
    base_speed: float 
    slow_turn_speed: float = 0.3
    normal_turn_speed: float = 0.6
    emergency_turn_speed: float = 1.5
    safe_threshold: int = 7
    caution_threshold: int = 4

# Enhanced Command Factory
class CommandFactory:
    """
    Factory for creating navigation commands with mission awareness.
    
    NEW: Added mission-specific strategies (tracking, patrol, exploration)
    """
    
    def __init__(self, base_speed: float, params: NavigationParameters = None):
        """
        Initialize command factory.
        
        Args:
            base_speed: Base linear velocity in m/s
            params: Optional navigation parameters (uses defaults if None)
        """
        self.base_speed = base_speed
        self.params = params or NavigationParameters(base_speed=base_speed)
        
        # NEW: Exploration boost multiplier (set by reasoner)
        self.exploration_boost = 1.0
    
    def set_exploration_boost(self, boost: float):
        """Set speed boost for exploration missions."""
        self.exploration_boost = max(1.0, min(2.0, boost))
        logger.info(f"[COMMAND FACTORY] Exploration boost: {self.exploration_boost}x")
    
    # Basic Movement Commands    
    def create_forward_command(self, safety_score: int) -> Dict[str, Any]:
        """Create forward movement command with safety-adjusted speed."""
        if safety_score >= self.params.safe_threshold:
            linear_vel = self.base_speed
            confidence = 0.9
        elif safety_score >= self.params.caution_threshold:
            linear_vel = self.base_speed * 0.6
            confidence = 0.7
        else:
            linear_vel = self.base_speed * 0.3
            confidence = 0.5
        
        return {
            'action': NavigationAction.MOVE_FORWARD.value,
            'parameters': {
                'linear_velocity': linear_vel,
                'angular_velocity': 0.0,
                'duration': 1.0
            },
            'confidence': confidence,
            'reason': f'forward_movement_safety_{safety_score}'
        }
    
    def create_slow_forward_command(self, safety_score: int = 5) -> Dict[str, Any]:
        """Create slow forward movement for obstacle approach."""
        return {
            'action': NavigationAction.SLOW_FORWARD.value,
            'parameters': {
                'linear_velocity': self.base_speed * 0.2,
                'angular_velocity': 0.0,
                'duration': 1.0
            },
            'confidence': 0.8,
            'reason': 'obstacles_detected_slow_approach'
        }
    
    def create_turn_and_move_command(self, direction: str, safety_score: int) -> Dict[str, Any]:
        """Create combined turn and forward movement command."""
        # Adjust turn speed based on safety
        angular_vel = (
            self.params.slow_turn_speed
            if safety_score < self.params.caution_threshold
            else self.params.normal_turn_speed
        )
        
        linear_vel = self.base_speed * 0.4
        
        # Set direction
        if direction == 'left':
            angular_vel = abs(angular_vel)
            action = NavigationAction.TURN_LEFT.value
        else:
            angular_vel = -abs(angular_vel)
            action = NavigationAction.TURN_RIGHT.value
        
        return {
            'action': action,
            'parameters': {
                'linear_velocity': linear_vel,
                'angular_velocity': angular_vel,
                'duration': 0.8
            },
            'confidence': 0.8,
            'reason': f'turn_{direction}_and_forward'
        }
    
    def create_turn_around_command(self) -> Dict[str, Any]:
        """Create 180-degree turn command for path reversal."""
        return {
            'action': NavigationAction.ROTATE_RIGHT.value,
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': -self.params.normal_turn_speed,
                'duration': 0.3
            },
            'confidence': 0.9,
            'reason': 'no_clear_path_turn_around'
        }
    
    def create_rotate_command(
        self,
        direction: str,
        duration: float = 0.5,
        speed_multiplier: float = 1.0
    ) -> Dict[str, Any]:
        """Create in-place rotation command."""
        base_angular = self.params.normal_turn_speed * speed_multiplier
        
        if direction == 'left':
            angular_vel = abs(base_angular)
            action = NavigationAction.ROTATE_LEFT.value
        else:
            angular_vel = -abs(base_angular)
            action = NavigationAction.ROTATE_RIGHT.value
        
        return {
            'action': action,
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': angular_vel,
                'duration': duration
            },
            'confidence': 0.7,
            'reason': f'rotate_{direction}'
        }
    
    def create_emergency_stop_command(self, reason: str = "emergency") -> Dict[str, Any]:
        """Create immediate stop command for emergency situations."""
        return {
            'action': NavigationAction.EMERGENCY_STOP.value,
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'confidence': 1.0,
            'reason': f'emergency_stop_{reason}'
        }
    
    # Mission-Specific Commands
    
    # --- TRACKING MISSION COMMANDS ---
    
    def create_track_follow_command(
        self,
        target_direction: Optional[str],
        safety_score: int
    ) -> Dict[str, Any]:
        """Follow target - adjust to match its movement."""
        if target_direction == 'left':
            return self.create_turn_and_move_command('left', safety_score)
        elif target_direction == 'right':
            return self.create_turn_and_move_command('right', safety_score)
        else:  # forward
            return self.create_forward_command(safety_score)
    
    def create_track_approach_command(self) -> Dict[str, Any]:
        """Move closer to target."""
        return {
            'action': 'move_forward',
            'parameters': {
                'linear_velocity': self.base_speed * 0.8,
                'angular_velocity': 0.0,
                'duration': 1.0
            },
            'confidence': 0.8,
            'reason': 'approaching_target'
        }
    
    def create_track_backup_command(self) -> Dict[str, Any]:
        """Back away from target (too close)."""
        return {
            'action': 'move_backward',
            'parameters': {
                'linear_velocity': -self.base_speed * 0.5,
                'angular_velocity': 0.0,
                'duration': 0.8
            },
            'confidence': 0.9,
            'reason': 'target_too_close'
        }
    
    def create_track_search_command(self, search_direction: str) -> Dict[str, Any]:
        """Search for lost target."""
        if search_direction == 'spin':
            return {
                'action': 'rotate_left',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': self.params.slow_turn_speed,
                    'duration': 1.5
                },
                'confidence': 0.7,
                'reason': 'searching_lost_target'
            }
        elif search_direction == 'left':
            return self.create_turn_and_move_command('left', 5)
        elif search_direction == 'right':
            return self.create_turn_and_move_command('right', 5)
        else:  # forward
            return self.create_slow_forward_command(5)
    
    # --- PATROL MISSION COMMANDS ---
    
    def create_patrol_circle_command(self) -> Dict[str, Any]:
        """Circular patrol motion."""
        return {
            'action': 'move_forward',
            'parameters': {
                'linear_velocity': self.base_speed * 0.7,
                'angular_velocity': 0.3,  # Gentle curve
                'duration': 1.0
            },
            'confidence': 0.8,
            'reason': 'circular_patrol'
        }
    
    # --- EXPLORATION MISSION COMMANDS ---
    
    def create_exploration_forward_command(
        self,
        clearance: float,
        safe_distance: float
    ) -> Dict[str, Any]:
        """Forward exploration with random drift."""
        boosted_speed = self.base_speed * self.exploration_boost
        
        return {
            'action': 'move_forward',
            'parameters': {
                'linear_velocity': boosted_speed,
                'angular_velocity': random.uniform(-0.15, 0.15),
                'duration': 2.0
            },
            'confidence': 0.85,
            'reason': 'explore_forward'
        }
    
    def create_exploration_cautious_command(
        self,
        left_clear: float,
        right_clear: float
    ) -> Dict[str, Any]:
        """Cautious forward with bias toward open side."""
        boosted_speed = self.base_speed * self.exploration_boost
        
        # Bias toward more open side
        if left_clear > right_clear + 0.3:
            angular_bias = random.uniform(0.1, 0.3)
        elif right_clear > left_clear + 0.3:
            angular_bias = random.uniform(-0.3, -0.1)
        else:
            angular_bias = random.uniform(-0.2, 0.2)
        
        return {
            'action': 'move_forward',
            'parameters': {
                'linear_velocity': boosted_speed * 0.5,
                'angular_velocity': angular_bias,
                'duration': 1.5
            },
            'confidence': 0.7,
            'reason': 'explore_cautious'
        }
    
    def create_exploration_turn_command(
        self,
        direction: str,
        is_stuck: bool = False
    ) -> Dict[str, Any]:
        """Turn during exploration (toward open space)."""
        boosted_speed = self.base_speed * self.exploration_boost
        
        angular_vel = self.params.normal_turn_speed
        if direction == 'left':
            action = 'rotate_left'
        else:
            action = 'rotate_right'
            angular_vel = -angular_vel
        
        return {
            'action': action,
            'parameters': {
                'linear_velocity': boosted_speed * 0.2,
                'angular_velocity': angular_vel,
                'duration': 1.2 if is_stuck else 0.8
            },
            'confidence': 0.8,
            'reason': f'explore_turn_{direction}'
        }
    
    def create_frontier_approach_command(
        self,
        direction: str,
        distance: float
    ) -> Dict[str, Any]:
        """Move toward detected frontier."""
        boosted_speed = self.base_speed * self.exploration_boost
        
        if direction == 'forward':
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': boosted_speed,
                    'angular_velocity': 0.0,
                    'duration': 2.0
                },
                'confidence': 0.85,
                'reason': 'frontier_forward'
            }
        else:  # left or right turn
            angular_vel = self.params.normal_turn_speed
            if direction == 'right':
                angular_vel = -angular_vel
            
            return {
                'action': f'turn_{direction}',
                'parameters': {
                    'linear_velocity': boosted_speed * 0.4,
                    'angular_velocity': angular_vel,
                    'duration': 1.5
                },
                'confidence': 0.9,
                'reason': f'frontier_{direction}'
            }
    
    # Path-based Navigation
    def create_command_from_clear_paths(
        self,
        clear_paths: List[str],
        recommended_direction: str,
        safety_score: int
    ) -> Dict[str, Any]:
        """
        Generate command based on available clear paths.
        
        MOVED FROM: navigation_reasoner._navigate_using_clear_paths()
        """
        # Handle special case: cautious forward
        if 'cautious_forward' in clear_paths:
            logger.info("[CAUTIOUS] Moving forward slowly in tight space")
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * 0.3,
                    'angular_velocity': 0.0,
                    'duration': 0.5
                },
                'confidence': 0.6,
                'reason': 'cautious_forward_tight_space'
            }
        
        # Path priority selection
        path_priority = ['center', 'left', 'right', 'far_left', 'far_right']
        selected_path = None
        
        for preferred_path in path_priority:
            if preferred_path in clear_paths:
                selected_path = preferred_path
                break
        
        # Fallback to recommended or first available
        if not selected_path:
            if recommended_direction in clear_paths:
                selected_path = recommended_direction
            else:
                selected_path = clear_paths[0] if clear_paths else None
        
        # Generate command
        if selected_path == 'center':
            return self.create_forward_command(safety_score)
        elif selected_path in ['left', 'far_left']:
            return self.create_turn_and_move_command('left', safety_score)
        elif selected_path in ['right', 'far_right']:
            return self.create_turn_and_move_command('right', safety_score)
        else:
            return self.create_forward_command(safety_score)
    
    def create_command_from_obstacles(
        self,
        obstacles: List[Dict]
    ) -> Dict[str, Any]:
        """
        Generate turn command when no clear paths available.
        
        MOVED FROM: navigation_reasoner._handle_no_clear_paths()
        """
        left_obstacles = sum(
            1 for obs in obstacles
            if 'left' in obs.get('position', '').lower()
        )
        
        right_obstacles = sum(
            1 for obs in obstacles
            if 'right' in obs.get('position', '').lower()
        )
        
        # Turn toward side with fewer obstacles
        if left_obstacles < right_obstacles:
            return self.create_rotate_command('left', duration=0.8)
        else:
            return self.create_rotate_command('right', duration=0.5)