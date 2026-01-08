"""
Navigation Reasoner Module
High-level decision making for robot navigation based on vision analysis.
"""

import time
import logging
from typing import Dict, List, Optional

import numpy as np

from multi_function_agent._robot_vision_controller.utils.movement_commands import (
    CommandFactory,
    NavigationAction,
    NavigationParameters
)
from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyValidator, SafetyThresholds
from multi_function_agent._robot_vision_controller.perception.detector.frontier_detector import FrontierDetector

logger = logging.getLogger(__name__)


# Navigation Reasoner
class NavigationReasoner:
    """
    High-level navigation decision maker for autonomous robot control.
    """
    
    def __init__(self, safety_level: str = "high", max_speed: float = 0.5):
        """
        Initialize navigation reasoner.
        
        Args:
            safety_level: Safety configuration (high/medium/low)
            max_speed: Maximum robot speed in m/s
        """
        self.safety_level = safety_level
        self.max_speed = max_speed
        self.nav_params = NavigationParameters(base_speed=max_speed)
        self.safety_validator = SafetyValidator()
        self.command_factory = CommandFactory(base_speed=max_speed, params=self.nav_params)
        
        # Safety level speed multipliers
        self.speed_multipliers = {
            "high": 0.3,
            "medium": 0.6,
            "low": 1.0
        }
        
        self.base_speed = max_speed * self.speed_multipliers.get(safety_level, 0.5)

        self.exploration_boost = 1.0  # Will be set from config via setter
        
        # Decision tracking
        self._last_decision = None
        self._decision_history = []
        self._start_time = time.time()
        self._startup_grace_period = 5.0
        self._last_decision_time = 0
        
        # Navigation configuration
        self.use_map_validation = True
        self.path_lookahead_distance = 1.0
        
        self._setup_decision_parameters()

        self. frontier_detector = FrontierDetector()    
        self.use_frontier_detection = True

    def _get_navigation_zone(
        self, 
        front_clear: float,
        left_clear: float,
        right_clear: float
    ) -> int:
        """
        Determine navigation zone using centralized thresholds.
        
        Zones:
        - Zone 1 (CRITICAL): Front < ZONE_1_CRITICAL (0.40m)
        - Zone 2 (MEDIUM): Front < ZONE_2_MEDIUM (0.80m) OR tight corridor
        - Zone 3 (FAR): Front >= ZONE_2_MEDIUM (0.80m)
        """
        
        # RULE 1: Front < 0.4m → CRITICAL
        if front_clear < SafetyThresholds.ZONE_1_CRITICAL:
            return 1
        
        # RULE 2: Front < 0.8m → MEDIUM
        if front_clear < SafetyThresholds.ZONE_2_MEDIUM:
            return 2
        
        # RULE 3: Tight corridor detection (both sides < 0.5m)
        corridor_threshold = 0.5
        if left_clear < corridor_threshold and right_clear < corridor_threshold:
            logger.warning(
                f"[ZONE CHECK] Tight corridor: L={left_clear:.2f} R={right_clear:.2f} "
                f"→ Zone 2 (despite Front={front_clear:.2f})"
            )
            return 2
        
        # RULE 4: All clear → FAR
        return 3

    def _decide_avoidance_direction(
        self, 
        clearances: Dict,
        current_angular: float = 0.0
    ) -> tuple[str, float]:
        """
        Decide which direction to steer for obstacle avoidance.
        """
        left = clearances.get('left', 0)
        right = clearances.get('right', 0)
        front = clearances.get('front', 0)
        
        # Clear difference threshold
        CLEAR_DIFF = 0.15  # 15cm difference to choose side
        
        # Left is clearly better
        if left > right + CLEAR_DIFF:
            angular = 0.5  # Moderate left turn
            direction = "left"
        
        # Right is clearly better
        elif right > left + CLEAR_DIFF:
            angular = -0.5  # Moderate right turn
            direction = "right"
        
        # Similar clearance - maintain current direction if any
        else:
            if abs(current_angular) > 0.1:
                # Continue current turn
                angular = current_angular
                direction = "continue"
            else:
                # Choose slightly better side
                angular = 0.3 if left > right else -0.3
                direction = "slight"
        
        return direction, angular


    def _create_zone2_command(
        self, 
        clearances: Dict,
        front_clear: float
    ) -> Dict:
        """
        Create command for Zone 2 (MEDIUM distance: 0.3-0.7m).
        
        Strategy: Slow down + aggressive steering to avoid obstacle.
        NO BACKUP - just turn and go around.
        """
        direction, angular = self._decide_avoidance_direction(clearances)
        
        # Adaptive speed: closer = slower
        # front_clear: 0.7m → speed 0.6x
        # front_clear: 0.5m → speed 0.4x
        # front_clear: 0.3m → speed 0.2x
        speed_factor = (front_clear - 0.3) / 0.4  # Linear scale 0.0-1.0
        speed_factor = max(0.2, min(0.6, speed_factor))
        
        linear_vel = self.base_speed * self.exploration_boost * speed_factor
        
        logger.info(
            f"[ZONE 2 - AVOIDANCE] Front: {front_clear:.2f}m → "
            f"Speed: {speed_factor:.1%}, Steer {direction} ({angular:.2f})"
        )
        
        return {
            'action': 'avoidance_steering',
            'parameters': {
                'linear_velocity': linear_vel,
                'angular_velocity': angular,
                'duration': 1.0
            },
            'confidence': 0.8,
            'reason': f'zone2_avoid_{direction}'
        }


    def _create_zone1_command(self, clearances: Dict) -> Dict:
        """
        Create command for Zone 1 (CRITICAL: <0.3m).
        
        Strategy:
        1. Check if sides are clear (>0.25m) → Rotate in place
        2. If all blocked → Gentle backup + rotate
        """
        left = clearances.get('left', 0)
        right = clearances.get('right', 0)
        front = clearances.get('front', 0)
        
        # Check if sides are clear enough for in-place rotation
        if left > 0.25 or right > 0.25:
            # Can rotate in place toward clearer side
            if left > right:
                angular = 0.7
                direction = "left"
            else:
                angular = -0.7
                direction = "right"
            
            logger.warning(
                f"[ZONE 1 - ROTATE] Front blocked {front:.2f}m → "
                f"Rotate {direction} (L:{left:.2f} R:{right:.2f})"
            )
            
            return {
                'action': 'critical_rotate',
                'parameters': {
                    'linear_velocity': 0.0,  # Pure rotation
                    'angular_velocity': angular,
                    'duration': 0.6
                },
                'confidence': 0.9,
                'reason': f'zone1_rotate_{direction}'
            }
        
        # All sides blocked - need gentle backup
        else:
            # Backup toward clearer side
            angular = 0.5 if left > right else -0.5
            
            logger.error(
                f"[ZONE 1 - BACKUP] All blocked (F:{front:.2f} L:{left:.2f} R:{right:.2f}) "
                f"→ Gentle backup"
            )
            
            return {
                'action': 'critical_backup',
                'parameters': {
                    'linear_velocity': -0.15,  # Very slow backup
                    'angular_velocity': angular,
                    'duration': 0.5  # Short backup
                },
                'confidence': 0.7,
                'reason': 'zone1_backup_blocked'
            }
        
    def _execute_directional_command(
        self,
        directive: str,
        vision_analysis: Dict
    ) -> Dict:
        """
        Execute directional commands from composite missions.
        """
        # Extract direction from directive
        # Format: 'directional_move_forward' → 'forward'
        parts = directive.split('_', 1)

        if len(parts) < 2:
            logger.error(f"Invalid directional directive: {directive}")
            return self.command_factory.create_forward_command(5)
        
        full_command = parts[1]  # 'turn_right'
        
        if 'forward' in full_command:
            command_type = 'forward'
        elif 'backward' in full_command:
            command_type = 'backward'
        elif 'left' in full_command:
            command_type = 'left'
        elif 'right' in full_command:
            command_type = 'right'
        else:
            logger.error(f"Unknown command: {full_command}")
            return self.command_factory.create_forward_command(5)
        
        logger.info(f"[DIRECTIONAL] Parsed: {directive} → {command_type}")
        
        safety_score = vision_analysis.get('safety_score', 5)
        
        # Generate appropriate command
        if command_type == 'forward':
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * 0.5,  # Moderate speed
                    'angular_velocity': 0.0,
                    'duration': 2.0
                },
                'confidence': 0.8,
                'reason': 'directional_forward'
            }
        
        elif command_type == 'backward':
            return {
                'action': 'move_backward',
                'parameters': {
                    'linear_velocity': -self.base_speed * 0.3,  # Slow backup
                    'angular_velocity': 0.0,
                    'duration': 1.5
                },
                'confidence': 0.8,
                'reason': 'directional_backward'
            }
        
        elif command_type in ['left', 'right']:
            angular_vel = 0.6 if command_type == 'left' else -0.6
            
            return {
                'action': f'rotate_{command_type}',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': angular_vel,
                    'duration': 2.0
                },
                'confidence': 0.9,
                'reason': f'directional_turn_{command_type}'
            }
        
        else:
            logger.error(f"Unknown directional command: {command_type}")
            return self.command_factory.create_forward_command(safety_score)

    def set_exploration_boost(self, boost: float):
        """
        Set speed boost factor for exploration missions.
        """
        self.exploration_boost = max(1.0, min(2.0, boost))
        logger.info(f"[NAVIGATION] Exploration speed boost set to: {self.exploration_boost}x")
    
    def _setup_decision_parameters(self):
        """
        Initialize decision-making parameters from validators and nav params.
        """
        self.EMERGENCY_SAFETY_THRESHOLD = 2
        self.CAUTION_SAFETY_THRESHOLD = self.nav_params.caution_threshold
        self.SAFE_THRESHOLD = self.nav_params.safe_threshold
        
        self.CRITICAL_DISTANCE = SafetyThresholds.CRITICAL_ABORT
        self.WARNING_DISTANCE = SafetyThresholds.WARNING_ZONE
        self.SAFE_DISTANCE = SafetyThresholds.SAFE_ZONE
        
        self.SLOW_TURN_SPEED = self.nav_params.slow_turn_speed
        self.NORMAL_TURN_SPEED = self.nav_params.normal_turn_speed
        self.EMERGENCY_TURN_SPEED = self.nav_params.emergency_turn_speed
        
        self.MIN_DECISION_INTERVAL = 0.05
    
    def _navigate_using_clear_paths(
        self,
        clear_paths: List[str],
        recommended_direction: str,
        safety_score: int
    ) -> Dict[str, any]:
        """
        Navigate using available clear paths.
        """
        # Handle special case: cautious forward (from adaptive threshold)
        if 'cautious_forward' in clear_paths:
            logger.info("[CAUTIOUS] Moving forward slowly in tight space")
            return {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': self.base_speed * 0.3,  # Very slow
                    'angular_velocity': 0.0,
                    'duration': 0.5
                },
                'confidence': 0.6,
                'reason': 'cautious_forward_tight_space'
            }
        
        # Original logic continues...
        path_priority = ['center', 'left', 'right', 'far_left', 'far_right']
        
        selected_path = None
        
        # Select path based on priority
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
        
        # Generate command based on selected path
        if selected_path == 'center':
            return self.command_factory.create_forward_command(safety_score)
        
        elif selected_path in ['left', 'far_left']:
            return self.command_factory.create_turn_and_move_command('left', safety_score)
        
        elif selected_path in ['right', 'far_right']:
            return self.command_factory.create_turn_and_move_command('right', safety_score)
        
        else:
            return self.command_factory.create_forward_command(safety_score)
    
    def _validate_and_smooth_decision(
        self,
        decision: Dict[str, any],
        analysis: Dict[str, any]
    ) -> Dict[str, any]:
        """
        Validate and clamp decision parameters to safe ranges.
        """
        params = decision.get('parameters', {})
        
        # Clamp linear velocity
        linear_vel = params.get('linear_velocity', 0.0)
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        params['linear_velocity'] = linear_vel
        
        # Clamp angular velocity
        angular_vel = params.get('angular_velocity', 0.0)
        max_angular = self.safety_validator.MAX_SAFE_ANGULAR_VEL
        angular_vel = max(-max_angular, min(max_angular, angular_vel))
        params['angular_velocity'] = angular_vel
        
        # Clamp duration
        duration = params.get('duration', 1.0)
        duration = max(0.5, min(3.0, duration))
        params['duration'] = duration
        
        decision['parameters'] = params
        
        return decision
    
    def _get_emergency_stop_decision(self, reason: str) -> Dict[str, any]:
        """
        Generate emergency stop command.
        """
        return self.command_factory.create_emergency_stop_command(reason)
    
    def _handle_no_clear_paths(self, obstacles: List[Dict]) -> Dict[str, any]:
        """
        Handle situation when no clear paths are available.
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
            return {
                'action': NavigationAction.ROTATE_LEFT.value,
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': self.NORMAL_TURN_SPEED,
                    'duration': 0.8
                },
                'confidence': 0.7,
                'reason': 'no_clear_paths_turn_left'
            }
        else:
            return {
                'action': NavigationAction.ROTATE_RIGHT.value,
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': -self.NORMAL_TURN_SPEED,
                    'duration': 0.5
                },
                'confidence': 0.7,
                'reason': 'no_clear_paths_turn_right'
            }
    
    def _repeat_last_decision(self) -> Dict[str, any]:
        """
        Repeat the last decision to avoid rapid command changes.
        """
        if self._last_decision:
            repeated_decision = self._last_decision.copy()
            repeated_decision['repeated'] = True
            return repeated_decision
        else:
            return self.command_factory.create_forward_command(5)
    
    def _update_decision_history(self, decision: Dict[str, any]):
        """
        Update decision history for stuck detection.
        """
        self._last_decision = decision
        
        self._decision_history.append({
            'action': decision.get('action'),
            'timestamp': time.time(),
            'confidence': decision.get('confidence', 0.5),
            'reason': decision.get('reason', '')
        })
        
        # Keep only last 10 decisions
        if len(self._decision_history) > 10:
            self._decision_history = self._decision_history[-10:]
    
    def _determine_immediate_action(
        self,
        spatial: Dict,
        vision: Dict,
        goal: str
    ) -> str:
        """
        Determine immediate action based on spatial analysis.
        """
        if spatial['safety_score'] < 3:
            return 'stop_immediately'
        
        if not spatial['clear_paths']:
            return 'turn_around'
        
        nearby_obstacles = [
            obs for obs in spatial['obstacles']
            if obs.get('distance_estimate', 1.0) < self.CRITICAL_DISTANCE
        ]
        
        if nearby_obstacles:
            return 'slow_forward'
        
        direction = spatial['recommended_direction']
        
        if direction == 'left':
            return 'turn_left_and_forward'
        elif direction == 'right':
            return 'turn_right_and_forward'
        else:
            return 'move_forward'
    
    def _execute_mission_directive(
        self,
        directive: str,
        vision_analysis: Dict, 
        robot_pos: Optional[Dict] = None
    ) -> Dict[str, any]:
        """
        Execute specific mission directive with appropriate navigation behavior.
        """
        logger.info(f"[MISSION DIRECTIVE] Executing: {directive}")
        
        safety_score = vision_analysis.get('safety_score', 5)
        clear_paths = vision_analysis.get('clear_paths', [])
        
        # ===== TRACKING DIRECTIVES =====
        if directive == 'track_follow':
            # Follow target - match its movement
            target_info = vision_analysis.get('target_tracking')
            if target_info and target_info.get('direction'):
                direction = target_info['direction']
                if direction == 'left':
                    return self.command_factory.create_turn_and_move_command('left', safety_score)
                elif direction == 'right':
                    return self.command_factory.create_turn_and_move_command('right', safety_score)
            return self.command_factory.create_forward_command(safety_score)
        
        elif directive == 'track_approach':
            # Move closer to target
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
        
        elif directive == 'track_backup':
            # Back away from target (too close)
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
        
        elif directive.startswith('track_search_'):
            # Search for lost target
            search_direction = directive.split('_')[-1]  # forward, left, right, spin
            
            if search_direction == 'spin':
                return {
                    'action': 'rotate_left',
                    'parameters': {
                        'linear_velocity': 0.0,
                        'angular_velocity': self.SLOW_TURN_SPEED,
                        'duration': 1.5
                    },
                    'confidence': 0.7,
                    'reason': 'searching_lost_target'
                }
            elif search_direction == 'left':
                return self.command_factory.create_turn_and_move_command('left', safety_score)
            elif search_direction == 'right':
                return self.command_factory.create_turn_and_move_command('right', safety_score)
            else:  # forward
                return self.command_factory.create_slow_forward_command(safety_score)
        
        # ===== PATROL DIRECTIVES =====
        elif directive.startswith('patrol_'):
            patrol_shape = directive.split('_')[-1]  # circle, square, corridor
            
            if patrol_shape == 'circle':
                # Circular motion: constant forward + slight turn
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
            else:
                # Default: forward with obstacle avoidance
                return self._make_navigation_decision(vision_analysis)
        
        # ===== EXPLORATION DIRECTIVES =====
        elif directive.startswith('explore_'):
            return self._frontier_aware_exploration(vision_analysis, robot_pos)
        
        else:  # Default or unknown directive
            # Standard exploration behavior
            return self._make_navigation_decision(vision_analysis)

    def _frontier_aware_exploration(
        self,
        vision_analysis: Dict,
        robot_pos: Dict
    ) -> Dict[str, any]:
        """
        Perform exploration with 3-zone reactive navigation.
        
        New strategy:
        - Zone 3 (>0.7m): Normal forward + slight steering
        - Zone 2 (0.3-0.7m): Slow + aggressive steering (NO BACKUP)
        - Zone 1 (<0.3m): Rotate/backup decision
        """
        clearances = vision_analysis.get('clearances', {})
        obstacles = vision_analysis.get('obstacles', [])
        
        front_clear = clearances.get('forward', 999)
        left_clear = clearances.get('left', 999)
        right_clear = clearances.get('right', 999)
        
        boosted_speed = self.base_speed * self.exploration_boost

        # Detect tight space condition
        min_clearance = min(front_clear, left_clear, right_clear)
        is_tight_space = min_clearance < 1.0  # Any side < 1.0m = tight
        
        if is_tight_space:
            logger.info(
                f"[TIGHT SPACE] Min clearance {min_clearance:.2f}m < 1.0m "
                f"→ Disabling frontier guidance"
            )

        # Determine navigation zone
        zone = self._get_navigation_zone(
            front_clear=clearances.get('forward', 999),
            left_clear=clearances.get('left', 999),
            right_clear=clearances.get('right', 999)
        )
        
        logger.debug(
            f"[NAV ZONE {zone}] F:{front_clear:.2f} L:{left_clear:.2f} R:{right_clear:.2f}"
        )
        
        # ZONE 1: CRITICAL (<0.3m) - Rotate or backup
        if zone == 1:
            return self._create_zone1_command(clearances)

        # ZONE 2: MEDIUM (0.3-0.7m) - Slow down + aggressive steering
        elif zone == 2:
            return self._create_zone2_command(clearances, front_clear)

        # ZONE 3: FAR (>0.7m) - Normal forward with frontier guidance
        else:
            # Try to get frontier direction
            best_frontier = None
            if self.use_frontier_detection and robot_pos:
                best_frontier = self.frontier_detector.get_best_frontier(robot_pos)
            
            # Frontier-guided navigation
            if best_frontier:
                frontier_direction = self.frontier_detector.get_frontier_direction(
                    best_frontier
                )
                
                if frontier_direction == 'forward':
                    angular = 0.0
                elif frontier_direction == 'left':
                    angular = 0.4
                elif frontier_direction == 'right':
                    angular = -0.4
                else:
                    angular = 0.0
                
                logger.info(
                    f"[ZONE 3 - FRONTIER] Target {best_frontier.distance:.1f}m "
                    f"at {best_frontier.angle:.0f}° → {frontier_direction}"
                )
            
            # No frontier - use clearance-based steering
            else:
                direction, angular = self._decide_avoidance_direction(clearances)
                
                # Log reason for not using frontier
                if is_tight_space:
                    logger.info(
                        f"[ZONE 3 - CLEARANCE] Tight space - using clearance-based "
                        f"steering {direction} (angular: {angular:.2f})"
                    )
                else:
                    logger.info(
                        f"[ZONE 3 - CLEARANCE] No valid frontier - steering {direction} "
                        f"(angular: {angular:.2f})"
                    )
            
            return {
                'action': 'explore_forward',
                'parameters': {
                    'linear_velocity': boosted_speed,
                    'angular_velocity': angular,
                    'duration': 2.0
                },
                'confidence': 0.85,
                'reason': 'zone3_explore_forward'
            }
            
    def _make_navigation_decision(
        self,
        analysis: Dict[str, any]
    ) -> Dict[str, any]:
        """
        Make standard navigation decision based on vision analysis.
        """
        safety_score = analysis.get('safety_score', 5)
        clear_paths = analysis.get('clear_paths', ['center'])
        recommended_direction = analysis.get('recommended_direction', 'center')
        immediate_action = analysis.get('immediate_action', 'move_forward')
        obstacles = analysis.get('obstacles', [])
        
        # Handle immediate actions from vision
        if immediate_action == 'turn_around':
            return self.command_factory.create_turn_around_command()
        elif immediate_action == 'slow_forward':
            return self.command_factory.create_slow_forward_command(safety_score)
        elif immediate_action == 'turn_left_and_forward':
            return self.command_factory.create_turn_and_move_command('left', safety_score)
        elif immediate_action == 'turn_right_and_forward':
            return self.command_factory.create_turn_and_move_command('right', safety_score)
        elif immediate_action == 'move_forward':
            return self.command_factory.create_forward_command(safety_score)
        
        # Navigate using clear paths
        if clear_paths:
            return self._navigate_using_clear_paths(
                clear_paths,
                recommended_direction,
                safety_score
            )
        
        # No clear paths available
        return self._handle_no_clear_paths(obstacles)
    

    def decide_next_action(
        self,
        vision_analysis: Dict,
        robot_pos: Dict = None,
        spatial_detector = None,
        lidar_override: Dict = None,
        mission_directive: str = None
    ) -> Dict:
        """
        Main decision-making function for navigation.
        """
        current_time = time.time()
        
        # Priority 1: Check LiDAR safety override FIRST
        if lidar_override and lidar_override.get('veto', False):
            logger.warning(f"[LIDAR OVERRIDE] {lidar_override.get('reason')}")
            return lidar_override['command']
        
        # Priority 2: Rate limiting
        if current_time - self._last_decision_time < self.MIN_DECISION_INTERVAL:
            return self._repeat_last_decision()
        
        start_time = time.time()
        
        try:
            # Priority 3: Execute mission directive or standard navigation
            if mission_directive:
                if mission_directive.startswith('directional_'):
                    return self._execute_directional_command(
                        mission_directive, 
                        vision_analysis
                    )
                
                decision = self._execute_mission_directive(
                    mission_directive, 
                    vision_analysis,
                    robot_pos
                )
            else:
                decision = self._make_navigation_decision(vision_analysis)
            
            # Validate and smooth decision
            final_decision = self._validate_and_smooth_decision(
                decision,
                vision_analysis
            )
            
            # Update history
            self._update_decision_history(final_decision)
            
            # Track decision time
            decision_time = (time.time() - start_time) * 1000
            final_decision['decision_time_ms'] = decision_time
            
            self._last_decision_time = current_time
            
            return final_decision
            
        except Exception as e:
            logger.error(f"Decision making failed: {e}")
            return self._get_emergency_stop_decision("decision_error")