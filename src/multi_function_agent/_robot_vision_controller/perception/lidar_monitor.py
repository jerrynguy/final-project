"""
LiDAR Safety Monitor Module (Smart Directional + Smart Recovery)
Critical abort with intelligent recovery toward open space.
"""

import time
import logging
import numpy as np
from typing import Dict, Optional, List, Tuple

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds

logger = logging.getLogger(__name__)


class LidarSafetyMonitor:
    """
    Smart directional safety monitor with intelligent recovery.
    
    Features:
    - Direction-aware obstacle detection
    - Smart recovery that turns toward open space
    - Hysteresis to prevent oscillation
    """
    
    def __init__(self):
        """Initialize monitor with centralized thresholds."""
        self.thresholds = SafetyThresholds
        
        self.abort_count = 0
        
        # Hysteresis state tracking
        self.last_abort_time = 0.0
        self.cooldown_duration = 3.0
        
        # Track current movement for smart abort
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
    
    def update_movement_state(self, linear_vel: float, angular_vel: float):
        """Update current movement direction for smart abort logic."""
        self.last_linear_velocity = linear_vel
        self.last_angular_velocity = angular_vel
    
    def _is_moving_forward(self) -> bool:
        """Check if robot is primarily moving forward (not turning)."""
        return (abs(self.last_linear_velocity) > 0.1 and 
                abs(self.last_angular_velocity) < 0.3)

    def _should_abort_for_obstacle(self, angle_deg: float, distance: float) -> bool:
        """
        Smart abort decision based on obstacle angle and movement direction.
        """
        is_forward = self._is_moving_forward()
        
        # More conservative abort thresholds
        if is_forward:
            # Moving forward: only abort if obstacle directly in path
            if abs(angle_deg) <= 45:  # Front arc
                critical_threshold = 0.22  # INCREASED from 0.20m
            else:  # Side/rear
                critical_threshold = 0.15  # Keep strict for sides
        else:
            # Turning/stopped: check wider arc
            if abs(angle_deg) <= 90:
                critical_threshold = 0.20
            else:
                critical_threshold = 0.15
        
        should_abort = distance < critical_threshold
        
        if should_abort:
            logger.debug(
                f"[ABORT CHECK] angle={angle_deg:.1f}°, dist={distance:.3f}m, "
                f"threshold={critical_threshold:.3f}m, forward={is_forward} → ABORT"
            )
        
        return should_abort
    
    # Command Generators     
    def _emergency_stop(self) -> Dict:
        """Generate emergency stop command."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'reason': 'emergency_stop'
        }
    
    def _pause_command(self) -> Dict:
        """Generate brief pause command during cooldown."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.3
            },
            'reason': 'cooldown_pause'
        }
    
    def check_critical_abort(
            self,
            lidar_data,
            robot_pos: Optional[Dict] = None
        ) -> Dict:
        """
        Check for critical distance with smart directional logic and recovery.
        """
        if lidar_data is None:
            logger.error("[CRITICAL] No LIDAR data - ABORT")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0,
                'state': 'no_lidar'
            }
        
        try:
            # Get obstacles with angles
            obstacles_with_angles = self._get_obstacles_with_angles(lidar_data)
            current_time = time.time()
            
            # Calculate time since last abort
            time_since_abort = current_time - self.last_abort_time
            in_cooldown = time_since_abort < self.cooldown_duration
            
            # STATE 1: Check if need to ABORT (smart directional check)
            if not in_cooldown:
                # Find closest obstacle that should trigger abort
                abort_obstacle = None
                min_abort_distance = float('inf')
                
                for angle_deg, distance in obstacles_with_angles:
                    if self._should_abort_for_obstacle(angle_deg, distance):
                        if distance < min_abort_distance:
                            min_abort_distance = distance
                            abort_obstacle = (angle_deg, distance)
                
                if abort_obstacle:
                    angle, distance = abort_obstacle
                    
                    # ABORT - start cooldown timer
                    self.last_abort_time = current_time
                    self.abort_count += 1
                    
                    logger.error(
                        f"[ABORT #{self.abort_count}] Obstacle at {angle:.1f}° "
                        f"({distance:.3f}m) → SMART RECOVERY"
                    )
                    
                    # Smart recovery with clearance analysis
                    clearances = self._analyze_clearances(obstacles_with_angles)

                    # Check if this is a dead-end situation
                    rear_clear = self._get_rear_clearance(obstacles_with_angles)
                    is_dead_end = self._is_dead_end(clearances, rear_clear)

                    # Extend cooldown for dead-end to prevent oscillation
                    if is_dead_end:
                        self.cooldown_duration = 3.0  # Longer cooldown (was 2.0s)
                        logger.warning("[DEAD-END] Extended cooldown to 3.0s")
                    else:
                        self.cooldown_duration = 2.0  # Normal cooldown
                    
                    return {
                        'abort': True,
                        'command': self._smart_recovery_command(
                            angle, 
                            clearances, 
                            obstacles_with_angles,
                            robot_pos
                        ),
                        'min_distance': distance,
                        'obstacle_angle': angle,
                        'clearances': clearances,
                        'state': 'abort'
                    }
            
            # STATE 2: In COOLDOWN after abort
            if in_cooldown:
                min_distance = self._get_min_distance(lidar_data)
                
                if min_distance > self.thresholds.RESUME_SAFE:
                    logger.info(
                        f"[RESUME] Clearance: {min_distance:.3f}m "
                        f"(after {time_since_abort:.1f}s cooldown) → NORMAL"
                    )
                    self.last_abort_time = 0.0
                    
                    return {
                        'abort': False,
                        'command': None,
                        'min_distance': min_distance,
                        'state': 'normal'
                    }
                
                else:
                    logger.debug(
                        f"[COOLDOWN] {time_since_abort:.1f}s elapsed - "
                        f"Distance: {min_distance:.3f}m (need > {self.thresholds.RESUME_SAFE}m)"
                    )
                    
                    return {
                        'abort': False,
                        'command': self._pause_command(),
                        'min_distance': min_distance,
                        'state': 'cooldown'
                    }

            # STATE 3: NORMAL operation
            min_distance = self._get_min_distance(lidar_data)
            return {
                'abort': False,
                'command': None,
                'min_distance': min_distance,
                'state': 'normal'
            }
            
        except Exception as e:
            logger.error(f"Safety check failed: {e}")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0,
                'state': 'error'
            }
    
    # Helper Methods    
    def _get_obstacles_with_angles(self, lidar_data) -> List[Tuple[float, float]]:
        """Extract obstacles with their angles from LIDAR scan."""
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            obstacles = []
            
            for i, distance in enumerate(ranges):
                if np.isnan(distance) or np.isinf(distance):
                    continue
                
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle_rad)
                
                # Normalize to [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                obstacles.append((angle_deg, distance))
            
            obstacles.sort(key=lambda x: x[1])
            
            return obstacles
            
        except Exception as e:
            logger.error(f"Failed to extract obstacles with angles: {e}")
            return []
    
    def _get_min_distance(self, lidar_data) -> float:
        """Extract minimum distance from LIDAR scan (360° coverage)."""
        try:
            ranges = lidar_data.ranges
            
            valid_ranges = [
                r for r in ranges 
                if not (np.isnan(r) or np.isinf(r))
            ]
            
            if not valid_ranges:
                logger.warning("No valid LIDAR ranges")
                return float('inf')
            
            return min(valid_ranges)
            
        except Exception as e:
            logger.error(f"Failed to get min distance: {e}")
            return float('inf')
    
    def get_min_distance(self, lidar_data) -> float:
        """Public interface for minimum distance."""
        return self._get_min_distance(lidar_data)
    
    def _analyze_clearances(self, obstacles_with_angles: List[Tuple[float, float]]) -> Dict:
        """
        Analyze clearance distances in each direction.
        """
        clearances = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }
        
        for angle_deg, distance in obstacles_with_angles:
            abs_angle = abs(angle_deg)
            
            # Front: ±45°
            if abs_angle <= 45:
                clearances['front'] = min(clearances['front'], distance)
            
            # Left: 45-135°
            elif 45 < angle_deg <= 135:
                clearances['left'] = min(clearances['left'], distance)
            
            # Right: -135 to -45°
            elif -135 <= angle_deg < -45:
                clearances['right'] = min(clearances['right'], distance)
        
        # Cap at max sensor range for readability
        for key in clearances:
            if clearances[key] == float('inf'):
                clearances[key] = 3.5  # Max LiDAR range
        
        return clearances

    def _get_rear_clearance(self, obstacles_with_angles: List[Tuple[float, float]]) -> float:
        """
        Get minimum clearance in rear arc for safe backup validation.
        
        Rear arc definition: |angle| > 120° (back hemisphere)
        """
        rear_distances = [
            distance for angle_deg, distance in obstacles_with_angles
            if abs(angle_deg) > 120  # Rear hemisphere: 120-180° and -120 to -180°
        ]
        
        if not rear_distances:
            return float('inf')
        
        min_rear = min(rear_distances)
        
        logger.debug(f"[REAR CHECK] Min rear clearance: {min_rear:.3f}m")
        return min_rear

    def _get_rear_clearance_by_direction(
        self, 
        obstacles_with_angles: List[Tuple[float, float]]
    ) -> Dict[str, float]:
        """
        Get rear clearance in 3 zones for intelligent backup.
        
        Zones:
        - rear_left: 120° to 180° (back-left quadrant)
        - rear_center: 150° to 180° AND -180° to -150° (straight back)
        - rear_right: -180° to -120° (back-right quadrant)
        
        Returns:
            Dict with min distance per zone. inf = no obstacle.
        """
        rear_zones = {
            'rear_left': [],
            'rear_center': [],
            'rear_right': []
        }
        
        for angle_deg, distance in obstacles_with_angles:
            # Rear left: 120° to 150°
            if 120 < angle_deg <= 150:
                rear_zones['rear_left'].append(distance)
            
            # Rear center: straight back (150° to 180° and -180° to -150°)
            elif 150 < angle_deg <= 180 or -180 <= angle_deg < -150:
                rear_zones['rear_center'].append(distance)
            
            # Rear right: -150° to -120°
            elif -150 <= angle_deg < -120:
                rear_zones['rear_right'].append(distance)
        
        # Calculate min distance per zone
        clearances = {}
        for zone, distances in rear_zones.items():
            if distances:
                clearances[zone] = min(distances)
            else:
                clearances[zone] = float('inf')  # No obstacle detected
        
        logger.debug(
            f"[REAR ZONES] Left:{clearances['rear_left']:.2f}m, "
            f"Center:{clearances['rear_center']:.2f}m, "
            f"Right:{clearances['rear_right']:.2f}m"
        )
        
        return clearances

    def _is_dead_end(self, clearances: Dict, rear_clear: float) -> bool:
        """
        Detect if robot is trapped in dead-end corner.
        
        Dead-end criteria: ALL sides blocked within critical distance.
        """
        DEAD_END_THRESHOLD = 0.18  # Stricter than normal abort (0.20m)
        
        front = clearances.get('front', float('inf'))
        left = clearances.get('left', float('inf'))
        right = clearances.get('right', float('inf'))
        
        # All 4 sides blocked
        all_sides_blocked = (
            front < DEAD_END_THRESHOLD and
            left < DEAD_END_THRESHOLD and
            right < DEAD_END_THRESHOLD and
            rear_clear < DEAD_END_THRESHOLD
        )
        
        if all_sides_blocked:
            logger.error(
                f"[DEAD-END DETECTED] F:{front:.2f} L:{left:.2f} "
                f"R:{right:.2f} Rear:{rear_clear:.2f} - ALL < {DEAD_END_THRESHOLD}m"
            )
            return True
        
        return False
    
    # Command Generators
    def _smart_recovery_command(
        self, 
        obstacle_angle: float, 
        clearances: Dict,
        obstacles_with_angles: List[Tuple[float, float]],
        robot_pos: Optional[Dict] = None
    ) -> Dict:
        """
        Generate smart recovery command with rear collision prevention.
        """
        left_clear = clearances.get('left', 0)
        right_clear = clearances.get('right', 0)
        
        # Check rear clearance for backup safety
        rear_clear = self._get_rear_clearance(obstacles_with_angles)
        
        # Detect tight corners (both sides blocked)
        is_tight_corner = left_clear < 0.3 and right_clear < 0.3
        
        # STRATEGY 1: ROTATE-ONLY if rear blocked or tight corner
        if rear_clear < 0.25 or is_tight_corner:
            # Determine rotation direction (toward more open side)
            if left_clear > right_clear + 0.1:
                angular = 0.8  # Strong left rotation
                direction = "left"
            elif right_clear > left_clear + 0.1:
                angular = -0.8  # Strong right rotation
                direction = "right"
            else:
                # Equal clearance: rotate away from obstacle
                angular = 0.8 if obstacle_angle > 0 else -0.8
                direction = "away_from_obstacle"
            
            logger.warning(
                f"[ROTATE-ONLY RECOVERY] Rear: {rear_clear:.2f}m, "
                f"L/R: {left_clear:.2f}/{right_clear:.2f}m → Rotate {direction}"
            )
            
            return {
                'action': 'rotate_recovery',
                'parameters': {
                    'linear_velocity': 0.0,  # NO BACKUP - rotate only
                    'angular_velocity': angular,
                    'duration': 0.8  # Quick rotation
                },
                'reason': f'rotate_only_{direction}_rear_blocked'
            }
        
        # STRATEGY 2: INTELLIGENT BACKUP with rear+front analysis
        else:
            # Get directional rear clearances
            rear_zones = self._get_rear_clearance_by_direction(obstacles_with_angles)
            
            rear_left = rear_zones.get('rear_left', 0)
            rear_center = rear_zones.get('rear_center', 0)
            rear_right = rear_zones.get('rear_right', 0)
            
            # Determine best backup direction using weighted scoring
            backup_scores = {
                'center': rear_center * 1.2,  # Prefer straight backup (1.2x weight)
                'left': rear_left * 1.0,
                'right': rear_right * 1.0
            }
            
            # Find best direction
            best_direction = max(backup_scores.items(), key=lambda x: x[1])
            backup_direction = best_direction[0]
            backup_clearance = rear_zones.get(f'rear_{backup_direction}', 0)
            
            # Safety check: minimum clearance required
            MIN_BACKUP_CLEARANCE = 0.25
            if backup_clearance < MIN_BACKUP_CLEARANCE:
                # Can't backup safely in any direction → force rotate-only
                logger.warning(
                    f"[BACKUP ABORT] All rear zones blocked "
                    f"(best: {backup_clearance:.2f}m < {MIN_BACKUP_CLEARANCE}m)"
                )
                
                # Fall back to rotate-only strategy
                if left_clear > right_clear + 0.1:
                    angular = 0.8
                    direction = "left"
                else:
                    angular = -0.8
                    direction = "right"
                
                return {
                    'action': 'forced_rotate_recovery',
                    'parameters': {
                        'linear_velocity': 0.0,
                        'angular_velocity': angular,
                        'duration': 1.0
                    },
                    'reason': f'backup_blocked_rotate_{direction}'
                }
            
            # Determine angular velocity based on backup direction
            if backup_direction == 'center':
                # Straight backup - add slight bias away from front obstacle
                if obstacle_angle > 10:  # Obstacle on left
                    angular = -0.2  # Slight right turn during backup
                    bias = "right"
                elif obstacle_angle < -10:  # Obstacle on right
                    angular = 0.2  # Slight left turn during backup
                    bias = "left"
                else:
                    angular = 0.0  # Pure straight backup
                    bias = "straight"
                
                logger.info(
                    f"[SMART BACKUP] Center backup with {bias} bias "
                    f"(clearance: {backup_clearance:.2f}m)"
                )
            
            elif backup_direction == 'left':
                angular = 0.5  # Moderate left turn while backing
                logger.info(
                    f"[SMART BACKUP] Left backup "
                    f"(rear-left: {backup_clearance:.2f}m)"
                )
            
            else:  # right
                angular = -0.5  # Moderate right turn while backing
                logger.info(
                    f"[SMART BACKUP] Right backup "
                    f"(rear-right: {backup_clearance:.2f}m)"
                )
            
            # Adaptive duration based on available clearance
            # Formula: duration = min(0.6s, clearance / 0.35)
            # Examples: 
            #   - 0.7m clearance → 0.6s (capped)
            #   - 0.4m clearance → 0.57s
            #   - 0.3m clearance → 0.43s
            safe_duration = min(0.6, backup_clearance / 0.35)
            
            return {
                'action': 'smart_backup_recovery',
                'parameters': {
                    'linear_velocity': -0.20,
                    'angular_velocity': angular,
                    'duration': safe_duration
                },
                'reason': f'smart_backup_{backup_direction}',
                'rear_clearance': backup_clearance,
                'backup_direction': backup_direction
            }
    
    # Statistics & Debugging    
    def get_stats(self) -> Dict:
        """Get safety monitor statistics."""
        return {
            'total_aborts': self.abort_count,
            'in_cooldown': time.time() - self.last_abort_time < self.cooldown_duration,
            'cooldown_remaining': max(
                0.0, 
                self.cooldown_duration - (time.time() - self.last_abort_time)
            ),
            'movement_state': 'forward' if self._is_moving_forward() else 'turning'
        }
    
    def reset_stats(self):
        """Reset abort counter (for testing)."""
        self.abort_count = 0
        self.last_abort_time = 0.0
        logger.info("[STATS RESET] Abort counter cleared")