"""
Safety Checks Module
Velocity validation and safety constraint enforcement for robot navigation.
"""

import math
import logging
from typing import Dict, Any
from dataclasses import dataclass
try:
    from geometry_msgs.msg import Twist
except ImportError:
    from multi_function_agent._robot_vision_controller.utils.ros2_stubs import Twist

logger = logging.getLogger(__name__)


# =============================================================================
# Safety Result Data Structure
# =============================================================================

@dataclass
class SafetyResult:
    """
    Result of safety validation check.
    """
    is_safe: bool
    reason: str = ""
    recommended_action: str = "continue"


# =============================================================================
# Safety Validator
# =============================================================================

class SafetyValidator:
    """
    Validates robot commands against safety constraints.
    """
    
    def __init__(self):
        """Initialize safety validator with default thresholds."""
        # Distance thresholds (meters)
        self.EMERGENCY_DISTANCE = 0.25  # Hardware limit - NEVER touch
        self.CRITICAL_DISTANCE_EXPLORE = 0.4   # Explore mode
        self.CRITICAL_DISTANCE_PATROL = 0.3    # Patrol mode (trust Nav2)
        self.CAUTION_DISTANCE = 0.7     # Start slowing down
        self.AWARE_DISTANCE = 1.2       # Normal operation
        
        # Velocity limits
        self.MAX_SAFE_LINEAR_VEL = 0.6
        self.MAX_SAFE_ANGULAR_VEL = 2.5
        
        # Statistics tracking
        self.total_checks = 0
        self.unsafe_detections = 0

    @property
    def CRITICAL_DISTANCE(self):
        return self.CRITICAL_DISTANCE_EXPLORE  # Default to explore

    @property  
    def WARNING_DISTANCE(self):
        return self.CAUTION_DISTANCE

    @property
    def SAFE_DISTANCE(self):
        return self.AWARE_DISTANCE
    
    def validate_movement_command(self, movement_decision: Dict[str, Any]) -> bool:
        """
        Validate navigation command for safety compliance.
        """
        self.total_checks += 1
        
        try:
            action = movement_decision.get('action', 'stop')
            params = movement_decision.get('parameters', {})
            linear_vel = params.get('linear_velocity', 0.0)
            angular_vel = params.get('angular_velocity', 0.0)
            
            # Validate velocity limits
            if not self._check_velocity_limits(linear_vel, angular_vel):
                self.unsafe_detections += 1
                logger.warning(
                    f"Unsafe velocities: linear={linear_vel}, "
                    f"angular={angular_vel}"
                )
                return False
            
            # Emergency actions and stops are always safe
            if 'emergency' in action or action == 'stop':
                return True
            
            return True
            
        except Exception as e:
            logger.error(f"Safety validation error: {e}")
            self.unsafe_detections += 1
            return False
    
    def _check_velocity_limits(self, linear_vel: float, angular_vel: float) -> bool:
        """
        Check if velocities are within safe limits.
        """
        # Check for NaN/Inf
        if not math.isfinite(linear_vel) or not math.isfinite(angular_vel):
            return False
        
        # Sanity check: reasonable range bounds
        if not (-10.0 < linear_vel < 10.0 and -10.0 < angular_vel < 10.0):
            return False
        
        # Check against maximum safe velocities
        if abs(linear_vel) > self.MAX_SAFE_LINEAR_VEL:
            return False
        
        if abs(angular_vel) > self.MAX_SAFE_ANGULAR_VEL:
            return False
        
        return True
    
    def _get_safe_fallback_analysis(self) -> Dict[str, Any]:
        """
        Generate safe fallback analysis for error conditions.
        """
        return {
            'obstacles': [{
                'type': 'unknown',
                'position': 'ahead',
                'threat_level': 'high'
            }],
            'clear_paths': [],
            'safety_score': 1,
            'recommended_direction': 'stop',
            'vision_instruction': 'stop',
            'immediate_action': 'stop_immediately',
            'processing_time_ms': 1.0,
            'error_fallback': True
        }


# =============================================================================
# Global Safety Validation Functions
# =============================================================================

def validate_robot_command_safety(twist: Twist, config: Dict[str, Any]) -> bool:
    """
    Validate ROS Twist message against configuration limits.
    """
    try:
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        
        # Get limits from config
        max_linear = config.get('max_linear_velocity', 0.62)
        max_angular = config.get('max_angular_velocity', 3.84)
        
        # Check for NaN/Inf
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            logger.error("Command contains invalid values (NaN/Inf)")
            return False
        
        # Check linear velocity limit
        if abs(linear_x) > max_linear:
            logger.warning(
                f"Linear velocity exceeds limit: "
                f"{linear_x:.3f} > {max_linear:.3f}"
            )
            return False
        
        # Check angular velocity limit
        if abs(angular_z) > max_angular:
            logger.warning(
                f"Angular velocity exceeds limit: "
                f"{angular_z:.3f} > {max_angular:.3f}"
            )
            return False
        
        return True
        
    except Exception as e:
        logger.error(f"Safety validation error: {e}")
        return False


def is_velocity_safe(linear_vel: float, angular_vel: float) -> bool:
    """
    Quick check if velocities are safe (conservative limits).
    """
    return (
        math.isfinite(linear_vel) and
        math.isfinite(angular_vel) and
        abs(linear_vel) <= 0.2 and
        abs(angular_vel) <= 1.5 and
        -10.0 < linear_vel < 10.0 and
        -10.0 < angular_vel < 10.0
    )


def get_recommended_speed_multiplier(min_obstacle_distance: float) -> float:
    """
    Calculate speed multiplier based on obstacle proximity.
    """
    if min_obstacle_distance < 0.25:
        return 0.0
    elif min_obstacle_distance < 0.4:
        return 0.3
    elif min_obstacle_distance < 0.6:
        return 0.6
    else:
        return 1.0