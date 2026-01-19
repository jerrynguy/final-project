"""
Navigation Reasoner Module (4-Zone Simplified)
High-level decision making based on 4 primary zones: FRONT, LEFT, RIGHT, BACK.
"""

import time
import logging
from typing import Dict, List, Optional

import numpy as np

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyValidator, SafetyThresholds
from multi_function_agent._robot_vision_controller.perception.detector.frontier_detector import FrontierDetector
from multi_function_agent._robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor

logger = logging.getLogger(__name__)


class NavigationReasoner:
    """
    Simplified 4-zone navigation decision maker.
    
    Zones:
    - FRONT: ±45° (forward movement)
    - LEFT: 45-135° (left side awareness)
    - RIGHT: -45 to -135° (right side awareness)
    - BACK: >135° (backup safety)
    
    Thresholds:
    - Safe zone: >0.80m
    - Warning zone: 0.50-0.80m
    - Critical zone: <0.50m
    """
    
    def __init__(
        self, 
        safety_level: str = "high", 
        max_speed: float = 0.5,
        safety_monitor: Optional[LidarSafetyMonitor] = None
    ):
        """Initialize navigation reasoner."""
        self.safety_level = safety_level
        self.max_speed = max_speed
        self.safety_validator = SafetyValidator()
        self.safety_monitor = safety_monitor
        
        # Safety level speed multipliers
        self.speed_multipliers = {
            "high": 0.3,
            "medium": 0.6,
            "low": 1.0
        }
        
        self.base_speed = max_speed * self.speed_multipliers.get(safety_level, 0.5)
        self.exploration_boost = 1.0
        
        # Frontier detection
        self.frontier_detector = FrontierDetector()
        self.use_frontier_detection = True

        # NEW: Stuck detection
        self.last_position = None
        self.position_history = []
        self.stuck_counter = 0
        self.STUCK_THRESHOLD = 5  # 5 iterations without movement
        
        logger.info(
            f"[NAVIGATION] Initialized 4-zone system:\n"
            f"  - Safe zone: >{SafetyThresholds.ZONE_3_COMFORTABLE:.2f}m\n"
            f"  - Warning zone: {SafetyThresholds.ZONE_2_CAUTION:.2f}-{SafetyThresholds.ZONE_3_COMFORTABLE:.2f}m\n"
            f"  - Critical zone: <{SafetyThresholds.ZONE_2_CAUTION:.2f}m"
        )
    
    def set_exploration_boost(self, boost: float):
        """Set speed boost factor for exploration missions."""
        self.exploration_boost = max(1.0, min(2.0, boost))
        logger.info(f"[NAVIGATION] Exploration speed boost set to: {self.exploration_boost}x")
    
    def _analyze_4_zones(self, lidar_data) -> Dict[str, float]:
        """
        Analyze LIDAR into 4 primary zones.
        
        Zone definitions:
        - FRONT: ±45° (90° cone)
        - LEFT: 45-135° (90° arc)
        - RIGHT: -45 to -135° (90° arc)
        - BACK: >135° or <-135° (90° arc)
        
        Returns:
            {
                'front': minimum distance in front zone,
                'left': minimum distance in left zone,
                'right': minimum distance in right zone,
                'back': minimum distance in back zone
            }
        """
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            zones = {
                'front': float('inf'),
                'left': float('inf'),
                'right': float('inf'),
                'back': float('inf')
            }
            
            for i, distance in enumerate(ranges):
                if np.isnan(distance) or np.isinf(distance):
                    continue
                
                # Calculate angle
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle_rad)
                
                # Normalize to [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                # Classify into zones
                if -45 <= angle_deg <= 45:
                    zones['front'] = min(zones['front'], distance)
                elif 45 < angle_deg <= 135:
                    zones['left'] = min(zones['left'], distance)
                elif -135 <= angle_deg < -45:
                    zones['right'] = min(zones['right'], distance)
                else:  # >135° or <-135°
                    zones['back'] = min(zones['back'], distance)
            
            return zones
            
        except Exception as e:
            logger.error(f"[4-ZONE] Analysis failed: {e}")
            return {
                'front': float('inf'),
                'left': float('inf'),
                'right': float('inf'),
                'back': float('inf')
            }
    
    def _navigate_safe(self, zones: Dict[str, float], robot_pos: Dict) -> Dict:
        """
        Safe zone (all zones > 0.80m) → Forward or explore.
        """
        # Try frontier guidance first
        if self.use_frontier_detection and robot_pos:
            best_frontier = self.frontier_detector.get_best_frontier(robot_pos)
            
            if best_frontier:
                frontier_dir = self.frontier_detector.get_frontier_direction(best_frontier)
                
                if frontier_dir == 'forward':
                    angular = 0.0
                elif frontier_dir == 'left':
                    angular = 0.4
                elif frontier_dir == 'right':
                    angular = -0.4
                else:
                    angular = 0.0
                
                logger.info(
                    f"[SAFE ZONE] Following frontier {best_frontier.distance:.1f}m "
                    f"at {best_frontier.angle:.0f}° → {frontier_dir}"
                )
                
                return {
                    'action': 'explore_forward',
                    'parameters': {
                        'linear_velocity': self.base_speed * self.exploration_boost,
                        'angular_velocity': angular,
                        'duration': 2.0
                    },
                    'confidence': 0.85,
                    'reason': 'safe_zone_frontier'
                }
        
        # No frontier → Forward based on front clearance
        if zones['front'] > SafetyThresholds.FRONTIER_MIN_CLEARANCE:
            logger.info(f"[SAFE ZONE] Front clear ({zones['front']:.2f}m), moving forward")
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * self.exploration_boost,
                    'angular_velocity': 0.0,
                    'duration': 2.0
                },
                'confidence': 0.8,
                'reason': 'safe_zone_forward'
            }
        else:
            # Front < 1.0m but still safe → Cautious forward
            logger.info(f"[SAFE ZONE] Front OK ({zones['front']:.2f}m), cautious forward")
            return {
                'action': 'slow_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * 0.7,
                    'angular_velocity': 0.0,
                    'duration': 1.5
                },
                'confidence': 0.75,
                'reason': 'safe_zone_cautious'
            }
    
    def _navigate_warning(self, zones: Dict[str, float]) -> Dict:
        """
        Warning zone (0.50-0.80m in some zone) → Steer away.
        """
        # Find closest zone
        closest_zone = min(zones, key=zones.get)
        closest_dist = zones[closest_zone]
        
        logger.info(
            f"[WARNING ZONE] Closest: {closest_zone} at {closest_dist:.2f}m"
        )
        
        # Speed scales with distance (0.50m→60%, 0.80m→100%)
        speed_factor = (closest_dist - SafetyThresholds.ZONE_2_CAUTION) / \
                       (SafetyThresholds.ZONE_3_COMFORTABLE - SafetyThresholds.ZONE_2_CAUTION)
        speed_factor = max(0.6, min(1.0, speed_factor))
        
        # Decision: Steer away from closest zone
        if closest_zone == 'front':
            # Front close → Check which side is better
            if zones['left'] > zones['right']:
                logger.info(f"[WARNING] Front close, steering LEFT (left:{zones['left']:.2f}m > right:{zones['right']:.2f}m)")
                return {
                    'action': 'slow_and_steer',
                    'parameters': {
                        'linear_velocity': self.base_speed * speed_factor,
                        'angular_velocity': 0.4,
                        'duration': 1.5
                    },
                    'confidence': 0.75,
                    'reason': 'warning_steer_left'
                }
            else:
                logger.info(f"[WARNING] Front close, steering RIGHT (right:{zones['right']:.2f}m > left:{zones['left']:.2f}m)")
                return {
                    'action': 'slow_and_steer',
                    'parameters': {
                        'linear_velocity': self.base_speed * speed_factor,
                        'angular_velocity': -0.4,
                        'duration': 1.5
                    },
                    'confidence': 0.75,
                    'reason': 'warning_steer_right'
                }
        
        elif closest_zone == 'left':
            # Left close → Steer right
            logger.info(f"[WARNING] Left close ({closest_dist:.2f}m), steering RIGHT")
            return {
                'action': 'slow_and_steer',
                'parameters': {
                    'linear_velocity': self.base_speed * speed_factor,
                    'angular_velocity': -0.4,
                    'duration': 1.5
                },
                'confidence': 0.75,
                'reason': 'warning_steer_right'
            }
        
        elif closest_zone == 'right':
            # Right close → Steer left
            logger.info(f"[WARNING] Right close ({closest_dist:.2f}m), steering LEFT")
            return {
                'action': 'slow_and_steer',
                'parameters': {
                    'linear_velocity': self.base_speed * speed_factor,
                    'angular_velocity': 0.4,
                    'duration': 1.5
                },
                'confidence': 0.75,
                'reason': 'warning_steer_left'
            }
        
        else:  # Back close (rare in warning)
            # Just move forward away from back
            logger.info(f"[WARNING] Back close ({closest_dist:.2f}m), moving forward")
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * 0.6,
                    'angular_velocity': 0.0,
                    'duration': 1.0
                },
                'confidence': 0.7,
                'reason': 'warning_back_escape'
            }

    def _navigate_critical(self, zones: Dict[str, float], lidar_data) -> Dict:
        """
        Critical zone (<0.50m) → Turn toward best clearance.
        
        SIMPLE STRATEGY: 
        1. Find direction with most space
        2. Rotate toward it
        3. If already facing it, creep forward
        """
        # Find best and worst zones
        best_zone = max(zones, key=zones.get)
        worst_zone = min(zones, key=zones.get)
        best_dist = zones[best_zone]
        worst_dist = zones[worst_zone]
        
        logger.warning(
            f"[CRITICAL] Worst: {worst_zone}={worst_dist:.2f}m, "
            f"Best: {best_zone}={best_dist:.2f}m"
        )

        # TRUE DEADLOCK: All directions < 0.30m (too tight to turn)
        if best_dist < SafetyThresholds.ZONE_0_EMERGENCY:
            logger.error(f"[TRUE DEADLOCK] All zones < {SafetyThresholds.ZONE_0_EMERGENCY}m")
            
            # Last resort: Try tiny backup
            rear_check = self.safety_monitor.check_rear_clearance(lidar_data)
            if rear_check and rear_check > SafetyThresholds.ZONE_1_PAUSE:
                logger.warning("[DESPERATION] Attempting micro-backup")
                return {
                    'action': 'backup_slow',
                    'parameters': {
                        'linear_velocity': -0.10,
                        'angular_velocity': 0.0,
                        'duration': 1.0
                    },
                    'confidence': 0.5,
                    'reason': 'desperation_backup'
                }
            
            return self._stop_command()

        # RULE 1: If front is BEST and > 0.40m → TRY FORWARD
        if best_zone == 'front' and best_dist > SafetyThresholds.ZONE_2_CAUTION:
            # Adaptive speed based on clearance
            if best_dist > 0.80:
                # Plenty of room → Use higher speed
                linear_vel = 0.22 
            elif best_dist > 0.60:
                # Good clearance → Medium speed
                linear_vel = 0.18  
            else:
                # Moderate clearance → Conservative
                linear_vel = 0.15  # Original
            logger.warning(f"[CRITICAL] Front is BEST ({best_dist:.2f}m), attempting CREEP")
            return {
                'action': 'creep_forward',
                'parameters': {
                    'linear_velocity': linear_vel,  # Slow but FORWARD
                    'angular_velocity': 0.0,
                    'duration': 1.5
                },
                'confidence': 0.75,
                'reason': 'critical_creep_best_front'
            }

        #o RULE 2: If front > 0.35m (even if not best) → TRY FORWARD ANYWAY
        # This breaks rotation loops
        if zones['front'] > SafetyThresholds.ZONE_2_CAUTION * 0.9 and best_dist - zones['front'] < SafetyThresholds.ZONE_1_PAUSE:
            # ✅ NEW: Adaptive speed
            if zones['front'] > 0.60:
                linear_vel = 0.18  # ← CHANGED from 0.12
            elif zones['front'] > 0.45:
                linear_vel = 0.15  # ← NEW tier
            else:
                linear_vel = 0.12  # Original (tight space)

            # Front is "good enough" (within 0.30m of best)
            logger.warning(
                f"[CRITICAL] Front acceptable ({zones['front']:.2f}m), "
                f"forcing FORWARD at {linear_vel:.2f} m/s to break rotation loop"
            )
            return {
                'action': 'creep_forward',
                'parameters': {
                    'linear_velocity': linear_vel,
                    'angular_velocity': 0.0,
                    'duration': 1.2
                },
                'confidence': 0.70,
                'reason': 'critical_break_rotation_loop'
            }
        
        # STRATEGY: Turn toward best clearance
        if best_zone == 'left':
            logger.warning(f"[CRITICAL] Rotating toward BEST clearance: LEFT ({best_dist:.2f}m)")
            return {
                'action': 'rotate_left',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.4,  # Slower rotation in tight space
                    'duration': 1.2
                },
                'confidence': 0.8,
                'reason': 'critical_turn_to_best'
            }
        
        elif best_zone == 'right':
            logger.warning(f"[CRITICAL] Rotating toward BEST clearance: RIGHT ({best_dist:.2f}m)")
            return {
                'action': 'rotate_right',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': -0.4,
                    'duration': 1.2
                },
                'confidence': 0.8,
                'reason': 'critical_turn_to_best'
            }
        
        else:  # best_zone == 'back'
            # Turn 180° - pick left or right based on which is more clear
            if zones['left'] > zones['right']:
                logger.warning(f"[CRITICAL] Best is BACK, turning LEFT first")
                return self._rotate_left()
            else:
                logger.warning(f"[CRITICAL] Best is BACK, turning RIGHT first")
                return self._rotate_right()
            
    # ===== Helper Functions =====
    
    def _rotate_left(self) -> Dict:
        """Pure left rotation."""
        return {
            'action': 'rotate_left',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.6,
                'duration': 1.5
            },
            'confidence': 0.8,
            'reason': 'critical_rotate_left'
        }
    
    def _rotate_right(self) -> Dict:
        """Pure right rotation."""
        return {
            'action': 'rotate_right',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': -0.6,
                'duration': 1.5
            },
            'confidence': 0.8,
            'reason': 'critical_rotate_right'
        }
    
    def _creep_forward(self) -> Dict:
        """Slow creep forward in tight space."""
        return {
            'action': 'creep_forward',
            'parameters': {
                'linear_velocity': 0.10,
                'angular_velocity': 0.0,
                'duration': 1.0
            },
            'confidence': 0.7,
            'reason': 'critical_creep_forward'
        }
    
    def _stop_command(self) -> Dict:
        """Emergency stop command."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'confidence': 1.0,
            'reason': 'emergency_stop'
        }
    
    def _check_if_stuck(self, robot_pos: Dict) -> bool:
        """
        Detect if robot stuck in same position.
        
        Returns:
            True if stuck (no movement for STUCK_THRESHOLD iterations)
        """
        if robot_pos is None:
            return False
        
        current_pos = (robot_pos['x'], robot_pos['y'])
        
        # Track position history
        self.position_history.append(current_pos)
        if len(self.position_history) > self.STUCK_THRESHOLD:
            self.position_history.pop(0)
        
        # Check if all recent positions are same (within 5cm)
        if len(self.position_history) >= self.STUCK_THRESHOLD:
            first_pos = self.position_history[0]
            
            all_same = all(
                abs(pos[0] - first_pos[0]) < 0.05 and 
                abs(pos[1] - first_pos[1]) < 0.05
                for pos in self.position_history
            )
            
            if all_same:
                self.stuck_counter += 1
               
                # Only trigger if stuck for LONG time
                # Allow 10 iterations of rotation before declaring stuck
                if self.stuck_counter >= 10:  # ← CHANGED: was immediate
                    logger.warning(
                        f"[STUCK #{self.stuck_counter}] No movement at "
                        f"({first_pos[0]:.2f}, {first_pos[1]:.2f})"
                    )
                    return True
                else:
                    logger.debug(
                        f"[POSITION STABLE #{self.stuck_counter}] "
                        f"(might be rotating, waiting...)"
                    )
                    return False

        # Reset counter if moving
        self.stuck_counter = 0
        return False
    
    def decide_next_action(
        self,
        vision_analysis: Dict,
        robot_pos: Dict = None,
        spatial_detector = None,
        lidar_data = None,
        mission_directive: str = None
    ) -> Dict:
        """
        Main decision-making function using 4-zone analysis.
        
        Flow:
        1. Analyze LIDAR into 4 zones (FRONT, LEFT, RIGHT, BACK)
        2. Find minimum distance across all zones
        3. Choose action based on zone clearances
        """
        
        # STEP 1: Validate LIDAR data
        if lidar_data is None:
            logger.warning("[NAV] No LIDAR data available")
            return self._stop_command()
        
        if self.safety_monitor is None:
            logger.error("[NAV] No safety monitor reference")
            return self._stop_command()
        
        # PRIORITY: Check mission directive FIRST
        if mission_directive == 'directional_stop':
            logger.info("[MISSION] Stop directive - halting all movement")
            return {
                'action': 'stop',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.0,
                    'duration': 0.1
                },
                'confidence': 1.0,
                'reason': 'mission_stop_directive'
            }
    
        # Check if stuck BEFORE choosing action
        if robot_pos and self._check_if_stuck(robot_pos):
            logger.error("[STUCK] Robot not moving, forcing aggressive turn")
            
            # Force random large rotation
            import random
            direction = random.choice(['left', 'right'])
            
            return {
                'action': f'rotate_{direction}',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.6 if direction == 'left' else -0.6,
                    'duration': 2.5  # Longer turn
                },
                'confidence': 0.9,
                'reason': 'stuck_recovery_aggressive_turn'
            }
        
        # STEP 2: Analyze into 4 zones
        zones = self._analyze_4_zones(lidar_data)
        min_dist = min(zones.values())
        
        logger.debug(
            f"[NAV] Zones - Front:{zones['front']:.2f}m, "
            f"Left:{zones['left']:.2f}m, Right:{zones['right']:.2f}m, "
            f"Back:{zones['back']:.2f}m | Min:{min_dist:.2f}m"
        )
        
        # STEP 3: Decision based on minimum distance
        if min_dist > SafetyThresholds.ZONE_3_COMFORTABLE:  # >0.80m
            return self._navigate_safe(zones, robot_pos)
        
        elif min_dist > SafetyThresholds.ZONE_2_CAUTION:  # 0.50-0.80m
            return self._navigate_warning(zones)
        
        else:  # <0.50m
            return self._navigate_critical(zones, lidar_data)