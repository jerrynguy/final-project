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

class SafetyThresholds:
    """
    🚨 SINGLE SOURCE OF TRUTH - ALL modules MUST import from here!
    
    Design philosophy:
    - Conservative abort (0.22m) protects hardware
    - Generous escape threshold (0.35m) accounts for sensor noise + robot width
    - Tight rejection arc (45°) maximizes escape options for differential drive
    """
    
    # ===== HARDWARE & CRITICAL ABORT =====
    HARDWARE_LIMIT = 0.12           # Physical collision distance (never breach)
    CRITICAL_ABORT = 0.22           # ⬆️ Emergency backup trigger (was 0.20)
    CRITICAL_ABORT_FRONT = 0.22     # Frontal critical (±45° arc)
    CRITICAL_ABORT_SIDE = 0.15      # Side obstacle (±90°-180° arc)
    RESUME_SAFE = 0.80              # ⬆️ Hysteresis resume 
    
    # ===== NAVIGATION ZONES (for NavigationReasoner) =====
    ZONE_1_CRITICAL = 0.30          # Zone 1: <0.4m → rotate/backup only
    ZONE_2_MEDIUM = 0.80            # Zone 2: 0.4-0.8m → slow + aggressive steer
    ZONE_3_FAR = 1.50               # Zone 3: >0.8m → normal speed + frontier guidance
    
    # ===== LEGACY ALIASES (backward compatibility) =====
    WARNING_ZONE = ZONE_1_CRITICAL  # 0.40m
    CAUTION_ZONE = ZONE_2_MEDIUM    # 0.80m
    SAFE_ZONE = ZONE_3_FAR          # 1.50m
    
    # ===== ESCAPE SYSTEM =====
    ESCAPE_SAFE_THRESHOLD = 0.35    # ⬆️ Min clearance for escape sector (was 0.30)
                                     # Rationale: Robot width ~0.3m + sensor noise margin
    OBSTACLE_REJECTION_ARC = 45     # ⬇️ ±45° rejection arc (was 60°)
                                     # Rationale: Differential drive can rotate in-place
    
    # ===== VELOCITY LIMITS =====
    MAX_SAFE_LINEAR_VEL = 0.6
    MAX_SAFE_ANGULAR_VEL = 2.5
    
    # ===== DIRECTIONAL ARC DEFINITIONS =====
    FRONT_ARC_HALF_ANGLE = 60       # ±60° = 120° frontal cone
    SIDE_ARC_HALF_ANGLE = 90        # ±90° = 180° side awareness
    
    # ===== REAR ARC SAFETY =====
    REAR_ARC_ANGLE = 120            # Rear arc starts at ±120°
    MIN_SAFE_BACKUP_CLEARANCE = 0.25  # Min rear distance to allow backup
    BACKUP_ABORT_THRESHOLD = 0.20   # Emergency stop during backup
    
    # ===== RECOVERY BEHAVIOR =====
    TIGHT_CORNER_THRESHOLD = 0.3    # If both L/R < 0.3m → rotate-only
    
    @classmethod
    def get_critical_distance_for_direction(cls, angle_deg: float, is_moving_forward: bool) -> float:
        """
        Get critical distance based on obstacle angle and movement direction.
        """
        abs_angle = abs(angle_deg)
        
        if is_moving_forward:
            # Forward movement: strict front, lenient sides
            if abs_angle <= cls.FRONT_ARC_HALF_ANGLE:
                return cls.CRITICAL_ABORT_FRONT  # 0.20m in front ±60°
            else:
                return cls.CRITICAL_ABORT_SIDE   # 0.15m on sides (only emergency)
        else:
            # Turning/stopped: check wider arc
            if abs_angle <= cls.SIDE_ARC_HALF_ANGLE:
                return cls.CRITICAL_ABORT_FRONT  # 0.20m in front ±90°
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
        # Reference centralized thresholds instead of local copies
        self.thresholds = SafetyThresholds
        
        # Statistics tracking
        self.total_checks = 0
        self.unsafe_detections = 0

    # CHANGED: Properties now reference SafetyThresholds
    @property
    def EMERGENCY_DISTANCE(self):
        return self.thresholds.HARDWARE_LIMIT
    
    @property
    def CRITICAL_DISTANCE(self):
        return self.thresholds.CRITICAL_ABORT
    
    @property
    def CRITICAL_DISTANCE_EXPLORE(self):
        # REMOVED: No mode-specific overrides
        return self.thresholds.CRITICAL_ABORT
    
    @property
    def CRITICAL_DISTANCE_PATROL(self):
        # REMOVED: No mode-specific overrides
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

# Global Safety Validation Functions

def validate_robot_command_safety(twist: Twist, config: Dict[str, Any]) -> bool:
    """
    Validate ROS Twist message against configuration limits.
    """
    try:
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        
        # CHANGED: Use centralized thresholds
        max_linear = config.get('max_linear_velocity', SafetyThresholds.MAX_SAFE_LINEAR_VEL)
        max_angular = config.get('max_angular_velocity', SafetyThresholds.MAX_SAFE_ANGULAR_VEL)
        
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
    CHANGED: Use centralized thresholds
    """
    if min_obstacle_distance < SafetyThresholds.CRITICAL_ABORT:
        return 0.0
    elif min_obstacle_distance < SafetyThresholds.WARNING_ZONE:
        return 0.3
    elif min_obstacle_distance < SafetyThresholds.CAUTION_ZONE:
        return 0.6
    else:
        return 1.0