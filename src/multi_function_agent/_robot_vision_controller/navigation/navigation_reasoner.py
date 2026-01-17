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
        if zones['front'] > 1.0:
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
        Critical zone (<0.50m in some zone) → Rotate or backup.
        """
        closest_zone = min(zones, key=zones.get)
        closest_dist = zones[closest_zone]
        
        logger.warning(
            f"[CRITICAL ZONE] {closest_zone.upper()} blocked at {closest_dist:.2f}m"
        )
        
        # OPTION 1: Front blocked
        if closest_zone == 'front':
            if zones['front'] < 0.40:
                # Very close (< 0.40m) → Must rotate
                if zones['left'] > zones['right']:
                    logger.warning(f"[CRITICAL] Front blocked, rotating LEFT (left:{zones['left']:.2f}m > right:{zones['right']:.2f}m)")
                    return self._rotate_left()
                else:
                    logger.warning(f"[CRITICAL] Front blocked, rotating RIGHT (right:{zones['right']:.2f}m > left:{zones['left']:.2f}m)")
                    return self._rotate_right()
            else:
                # 0.40-0.50m → Can creep if sides clear
                if zones['left'] > 0.70 and zones['right'] > 0.70:
                    logger.info(f"[CRITICAL] Front tight but sides clear, creeping forward")
                    return self._creep_forward()
                else:
                    # Sides also tight → Rotate to better side
                    if zones['left'] > zones['right']:
                        logger.warning(f"[CRITICAL] Sides tight, rotating LEFT")
                        return self._rotate_left()
                    else:
                        logger.warning(f"[CRITICAL] Sides tight, rotating RIGHT")
                        return self._rotate_right()
        
        # OPTION 2: Left blocked → Rotate right
        elif closest_zone == 'left':
            logger.warning(f"[CRITICAL] Left blocked ({closest_dist:.2f}m), rotating RIGHT")
            return self._rotate_right()
        
        # OPTION 3: Right blocked → Rotate left
        elif closest_zone == 'right':
            logger.warning(f"[CRITICAL] Right blocked ({closest_dist:.2f}m), rotating LEFT")
            return self._rotate_left()
        
        # OPTION 4: Back blocked → Move forward if possible
        else:
            if zones['front'] > 0.70:
                logger.warning(f"[CRITICAL] Back blocked, escaping FORWARD (front:{zones['front']:.2f}m)")
                return {
                    'action': 'move_forward',
                    'parameters': {
                        'linear_velocity': 0.2,
                        'angular_velocity': 0.0,
                        'duration': 1.0
                    },
                    'confidence': 0.7,
                    'reason': 'critical_back_escape'
                }
            else:
                # Front also blocked → Try backup if rear clear
                rear_clearance = self.safety_monitor.check_rear_clearance(lidar_data)
                
                if rear_clearance and rear_clearance > 0.40:
                    logger.warning(f"[CRITICAL] All front blocked, BACKING UP (rear:{rear_clearance:.2f}m)")
                    return {
                        'action': 'backup_slow',
                        'parameters': {
                            'linear_velocity': -0.20,
                            'angular_velocity': 0.0,
                            'duration': 2.0
                        },
                        'confidence': 0.7,
                        'reason': 'critical_backup'
                    }
                else:
                    # Stuck
                    logger.error(f"[DEADLOCK] All directions blocked")
                    return self._stop_command()
    
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