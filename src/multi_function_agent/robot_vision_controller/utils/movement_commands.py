"""
Movement Commands Module
Navigation command generation and ROS Twist message conversion.
"""

import logging
from enum import Enum
from typing import Dict, Any
from dataclasses import dataclass
from geometry_msgs.msg import Twist, Vector3

logger = logging.getLogger(__name__)


# =============================================================================
# Navigation Action Enumeration
# =============================================================================

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


# =============================================================================
# Navigation Parameters
# =============================================================================

@dataclass
class NavigationParameters:
    """
    Navigation speed and threshold parameters.
    """
    base_speed: float 
    slow_turn_speed: float = 0.3
    normal_turn_speed: float = 0.6
    emergency_turn_speed: float = 1.5
    safe_threshold: int = 7
    caution_threshold: int = 4


# =============================================================================
# Command Factory
# =============================================================================

class CommandFactory:
    """
    Factory for creating navigation command dictionaries.
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
    
    def create_forward_command(self, safety_score: int) -> Dict[str, Any]:
        """
        Create forward movement command with safety-adjusted speed.
        """
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
        """
        Create slow forward movement for obstacle approach.
        """
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
        """
        Create combined turn and forward movement command.
        """
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
        """
        Create 180-degree turn command for path reversal.
        """
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
        """
        Create in-place rotation command.
        """
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
        """
        Create immediate stop command for emergency situations.
        """
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

# =============================================================================
# ROS Twist Conversion
# =============================================================================

def convert_navigation_to_twist(
    action: str,
    parameters: Dict[str, Any],
    config: Dict[str, Any]
) -> Twist:
    """
    Convert navigation command to ROS Twist message.
    """
    twist = Twist()
    
    # Get velocity limits from config
    max_linear = config.get('max_linear_velocity', 0.22)
    max_angular = config.get('max_angular_velocity', 2.84)
    
    linear_vel = 0.0
    angular_vel = 0.0
    
    # Handle both string and enum action types
    if isinstance(action, str):
        action_value = action
    else:
        action_value = action.value if hasattr(action, 'value') else str(action)
    
    # Map actions to velocities
    if action_value in [NavigationAction.STOP.value, NavigationAction.EMERGENCY_STOP.value]:
        linear_vel = 0.0
        angular_vel = 0.0
    
    elif action_value == NavigationAction.MOVE_FORWARD.value:
        linear_vel = parameters.get('linear_velocity', 0.15)
    
    elif action_value == NavigationAction.MOVE_BACKWARD.value:
        linear_vel = -parameters.get('linear_velocity', 0.15)
    
    elif action_value == NavigationAction.TURN_LEFT.value:
        linear_vel = parameters.get('linear_velocity', 0.15)
        angular_vel = parameters.get('angular_velocity', 1.0)
    
    elif action_value == NavigationAction.TURN_RIGHT.value:
        linear_vel = parameters.get('linear_velocity', 0.15)
        angular_vel = -parameters.get('angular_velocity', 1.0)
    
    elif action_value == NavigationAction.ROTATE_LEFT.value:
        angular_vel = parameters.get('angular_velocity', 1.0)
    
    elif action_value == NavigationAction.ROTATE_RIGHT.value:
        angular_vel = -parameters.get('angular_velocity', 1.0)
    
    elif action_value == NavigationAction.SLOW_FORWARD.value:
        linear_vel = parameters.get('linear_velocity', 0.08)
    
    else:
        logger.warning(f"Unknown action: {action_value}, defaulting to stop")
        linear_vel = 0.0
        angular_vel = 0.0
    
    # Clamp velocities to safe limits
    linear_vel = max(-max_linear, min(max_linear, linear_vel))
    angular_vel = max(-max_angular, min(max_angular, angular_vel))
    
    # Build Twist message
    twist.linear = Vector3(x=linear_vel, y=0.0, z=0.0)
    twist.angular = Vector3(x=0.0, y=0.0, z=angular_vel)
    
    return twist