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

# GLOBAL SAFETY THRESHOLDS (Single Source of Truth)

# safety_checks.py

class SafetyThresholds:
    """
    🚨 SINGLE SOURCE OF TRUTH - ALL modules import from here!
    Optimized for TurtleBot3 Waffle (width=0.28m)
    """
    
    # ===== HARDWARE PROTECTION =====
    HARDWARE_LIMIT = 0.15  # Physical collision (unchanged)
    
    # ===== CRITICAL ABORT =====
    CRITICAL_ABORT = 0.25  # Emergency stop (unchanged)
    CRITICAL_ABORT_FRONT = 0.22
    CRITICAL_ABORT_SIDE = 0.15
    
    # ===== NAVIGATION ZONES ===== 
    ZONE_0_EMERGENCY = 0.25  
    ZONE_1_PAUSE = 0.40      
    ZONE_2_CAUTION = 0.50    
    ZONE_3_COMFORTABLE = 0.70 
    
    # ===== LEGACY ALIASES (backward compatibility) =====
    WARNING_ZONE = ZONE_1_PAUSE  
    CAUTION_ZONE = ZONE_2_CAUTION  
    SAFE_ZONE = ZONE_3_COMFORTABLE  
    
    # ===== NEW: FRONTIER DETECTION =====
    FRONTIER_MIN_CLEARANCE = 1.0  
    FRONTIER_WALL_THRESHOLD = 0.60  
    FRONTIER_WALL_SAFE_DISTANCE = 0.50  
    
    # ===== ESCAPE SYSTEM =====
    RESUME_SAFE = 0.50  
    ESCAPE_SAFE_THRESHOLD = 0.40  
    
    # ===== BACKUP SAFETY =====
    MIN_SAFE_BACKUP_CLEARANCE = 0.40  
    BACKUP_ABORT_THRESHOLD = 0.25
    BACKUP_CHECK_ARC = 30
    
    # ===== LATERAL ESCAPE =====
    LATERAL_PREFERENCE_THRESHOLD = 0.40  
    
    # ===== VELOCITY LIMITS =====
    MAX_SAFE_LINEAR_VEL = 0.15
    MAX_SAFE_ANGULAR_VEL = 0.6
    
    # ===== DIRECTIONAL ARCS =====
    FRONT_ARC_HALF_ANGLE = 60
    SIDE_ARC_HALF_ANGLE = 90
    REAR_ARC_ANGLE = 120
    OBSTACLE_REJECTION_ARC = 45
    
    # ===== RECOVERY BEHAVIOR =====
    TIGHT_CORNER_THRESHOLD = 0.25  # ← CHANGED: 0.3 → 0.25 (sync with ZONE_0)
    
    # ===== BEHAVIORAL PARAMETERS (CHO ACE ADJUST) ===== 
    # Recovery Timing
    ESCAPE_DURATION = 2.0           # 1.5-3.0 seconds
    ROTATION_DURATION = 1.2         # 0.8-2.0 seconds  
    BACKUP_DURATION = 2.0           # 1.5-3.0 seconds

    # Movement Aggressiveness
    CREEP_SPEED_MULTIPLIER = 0.5    # 0.3-0.7
    TURN_LEFT_AGGRESSIVENESSLY = 0.6       # 0.4-0.8 rad/s
    TURN_RIGHT_AGGRESSIVENESSLY = -0.6      # 0.4-0.8 rad/s

    # Decision Weights
    FRONTIER_PREFERENCE_WEIGHT = 0.8  # 0.5-1.0
    STUCK_PENALTY_MULTIPLIER = 1.5    # 1.0-2.0

    # Stuck Detection  
    STUCK_THRESHOLD = 5               # 3-10 iterations
    STUCK_POSITION_TOLERANCE = 0.05   # 0.03-0.10 meters
    ABORT_SPAM_THRESHOLD = 10         # 5-15 count
    STUCK_BAILOUT_THRESHOLD = 30.0

    @classmethod
    def get_critical_distance_for_direction(cls, angle_deg: float, is_moving_forward: bool) -> float:
        """
        Get critical distance based on obstacle angle and movement direction.
        """
        abs_angle = abs(angle_deg)
        
        if is_moving_forward:
            # Forward movement: strict front, lenient sides
            if abs_angle <= cls.FRONT_ARC_HALF_ANGLE:
                return cls.CRITICAL_ABORT_FRONT  # 0.22m in front ±60°
            else:
                return cls.CRITICAL_ABORT_SIDE   # 0.15m on sides
        else:
            # Turning/stopped: check wider arc
            if abs_angle <= cls.SIDE_ARC_HALF_ANGLE:
                return cls.CRITICAL_ABORT_FRONT  # 0.22m in front ±90°
            else:
                return cls.CRITICAL_ABORT_SIDE   # 0.15m behind

# Safety Result Data Structure
@dataclass
class SafetyResult:
    """
    Result of safety validation check.
    """
    is_safe: bool
    reason: str = ""
    recommended_action: str = "continue"

# Safety Validator
class SafetyValidator:
    """
    Validates robot commands against safety constraints.
    Uses centralized SafetyThresholds.
    """
    
    def __init__(self):
        """Initialize safety validator with centralized thresholds."""
        self.thresholds = SafetyThresholds
        self.total_checks = 0
        self.unsafe_detections = 0

    @property
    def EMERGENCY_DISTANCE(self):
        return self.thresholds.HARDWARE_LIMIT
    
    @property
    def CRITICAL_DISTANCE(self):
        return self.thresholds.CRITICAL_ABORT
    
    @property
    def CRITICAL_DISTANCE_EXPLORE(self):
        return self.thresholds.CRITICAL_ABORT
    
    @property
    def CRITICAL_DISTANCE_PATROL(self):
        return self.thresholds.CRITICAL_ABORT

    @property  
    def WARNING_DISTANCE(self):
        return self.thresholds.CAUTION_ZONE

    @property
    def SAFE_DISTANCE(self):
        return self.thresholds.SAFE_ZONE
    
    @property
    def MAX_SAFE_LINEAR_VEL(self):
        return self.thresholds.MAX_SAFE_LINEAR_VEL
    
    @property
    def MAX_SAFE_ANGULAR_VEL(self):
        return self.thresholds.MAX_SAFE_ANGULAR_VEL
    
    def validate_movement_command(self, movement_decision: Dict[str, Any]) -> bool:
        """Validate navigation command for safety compliance."""
        self.total_checks += 1
        
        try:
            action = movement_decision.get('action', 'stop')
            params = movement_decision.get('parameters', {})
            linear_vel = params.get('linear_velocity', 0.0)
            angular_vel = params.get('angular_velocity', 0.0)
            
            if not self._check_velocity_limits(linear_vel, angular_vel):
                self.unsafe_detections += 1
                logger.warning(
                    f"Unsafe velocities: linear={linear_vel}, "
                    f"angular={angular_vel}"
                )
                return False
            
            if 'emergency' in action or action == 'stop':
                return True
            
            return True
            
        except Exception as e:
            logger.error(f"Safety validation error: {e}")
            self.unsafe_detections += 1
            return False
    
    def _check_velocity_limits(self, linear_vel: float, angular_vel: float) -> bool:
        """Check if velocities are within safe limits."""
        if not math.isfinite(linear_vel) or not math.isfinite(angular_vel):
            return False
        
        if not (-10.0 < linear_vel < 10.0 and -10.0 < angular_vel < 10.0):
            return False
        
        if abs(linear_vel) > self.MAX_SAFE_LINEAR_VEL:
            return False
        
        if abs(angular_vel) > self.MAX_SAFE_ANGULAR_VEL:
            return False
        
        return True
    
    def _get_safe_fallback_analysis(self) -> Dict[str, Any]:
        """Generate safe fallback analysis for error conditions."""
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

# Global Safety Validation Functions

def validate_robot_command_safety(twist: Twist, config: Dict[str, Any]) -> bool:
    """Validate ROS Twist message against configuration limits."""
    try:
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        
        max_linear = config.get('max_linear_velocity', SafetyThresholds.MAX_SAFE_LINEAR_VEL)
        max_angular = config.get('max_angular_velocity', SafetyThresholds.MAX_SAFE_ANGULAR_VEL)
        
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            logger.error("Command contains invalid values (NaN/Inf)")
            return False
        
        if abs(linear_x) > max_linear:
            logger.warning(
                f"Linear velocity exceeds limit: "
                f"{linear_x:.3f} > {max_linear:.3f}"
            )
            return False
        
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
    """Quick check if velocities are safe (conservative limits)."""
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
    ✅ UPDATED: Very aggressive thresholds
    """
    if min_obstacle_distance < SafetyThresholds.CRITICAL_ABORT:  # 0.25m
        return 0.0
    elif min_obstacle_distance < SafetyThresholds.WARNING_ZONE:  # 0.40m
        return 0.4
    elif min_obstacle_distance < SafetyThresholds.CAUTION_ZONE:  # 0.50m
        return 0.7
    else:  # >= 0.80m
        return 1.0