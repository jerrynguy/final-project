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
        self.cooldown_duration = 2.0
        
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
        """Smart abort decision based on obstacle angle and movement direction."""
        is_forward = self._is_moving_forward()
        
        critical_threshold = self.thresholds.get_critical_distance_for_direction(
            angle_deg, is_forward
        )
        
        should_abort = distance < critical_threshold
        
        if should_abort:
            logger.debug(
                f"[ABORT CHECK] angle={angle_deg:.1f}°, dist={distance:.3f}m, "
                f"threshold={critical_threshold:.3f}m, forward={is_forward} → ABORT"
            )
        
        return should_abort
    
    # =========================================================================
    # Command Generators (MOVED UP to fix "method not found" error)
    # =========================================================================
    
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
    
    def check_critical_abort(self, lidar_data) -> Dict:
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
            
            # =====================================================================
            # STATE 1: Check if need to ABORT (smart directional check)
            # =====================================================================
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
                    
                    # NEW ABORT - start cooldown timer
                    self.last_abort_time = current_time
                    self.abort_count += 1
                    
                    logger.error(
                        f"[ABORT #{self.abort_count}] Obstacle at {angle:.1f}° "
                        f"({distance:.3f}m) → SMART RECOVERY"
                    )
                    
                    # CHANGED: Smart recovery with clearance analysis
                    clearances = self._analyze_clearances(obstacles_with_angles)
                    
                    return {
                        'abort': True,
                        'command': self._smart_recovery_command(
                            angle, 
                            clearances, 
                            obstacles_with_angles
                        ),
                        'min_distance': distance,
                        'obstacle_angle': angle,
                        'clearances': clearances,
                        'state': 'abort'
                    }
            
            # =====================================================================
            # STATE 2: In COOLDOWN after abort
            # =====================================================================
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
            
            # =====================================================================
            # STATE 3: NORMAL operation
            # =====================================================================
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
    
    # =========================================================================
    # Helper Methods
    # =========================================================================
    
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
        
        Args:
            obstacles_with_angles: List of (angle_deg, distance) tuples
            
        Returns:
            float: Minimum rear distance (inf if no rear obstacles)
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
    
    # =========================================================================
    # Command Generators
    # =========================================================================
    def _smart_recovery_command(
        self, 
        obstacle_angle: float, 
        clearances: Dict,
        obstacles_with_angles: List[Tuple[float, float]]  # CHANGED: Thêm param
    ) -> Dict:
        """
        Generate smart recovery command with rear collision prevention.
        
        Strategy:
        1. Check rear clearance first
        2. If rear < 0.25m → Rotate-only recovery (NO backup)
        3. If rear >= 0.25m → Safe backup with adaptive duration
        4. Turn toward more open side
        
        Args:
            obstacle_angle: Angle of obstacle that triggered abort
            clearances: Left/right/front clearance dict
            obstacles_with_angles: Full obstacle list for rear checking
            
        Returns:
            dict: Recovery command with safe parameters
        """
        left_clear = clearances.get('left', 0)
        right_clear = clearances.get('right', 0)
        
        # ADDED: Check rear clearance for backup safety
        rear_clear = self._get_rear_clearance(obstacles_with_angles)
        
        # ADDED: Detect tight corners (both sides blocked)
        is_tight_corner = left_clear < 0.3 and right_clear < 0.3
        
        # =======================================================================
        # STRATEGY 1: ROTATE-ONLY if rear blocked or tight corner
        # =======================================================================
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
        
        # =======================================================================
        # STRATEGY 2: SAFE BACKUP with adaptive duration
        # =======================================================================
        
        # Determine turn direction (toward more open side)
        if left_clear > right_clear + 0.2:
            angular = 0.6  # Moderate left turn while backing
            direction = "left"
        elif right_clear > left_clear + 0.2:
            angular = -0.6  # Moderate right turn while backing
            direction = "right"
        else:
            # Equal clearance: turn away from obstacle angle
            angular = 0.7 if obstacle_angle > 0 else -0.7
            direction = "away_from_obstacle"
        
        # CHANGED: Adaptive backup duration based on rear clearance
        # Formula: duration = min(0.6s, rear_clearance / 0.35)
        # Examples: 
        #   - rear 0.7m → 0.6s (capped)
        #   - rear 0.4m → 0.57s
        #   - rear 0.3m → 0.43s
        safe_duration = min(0.6, rear_clear / 0.35)
        
        logger.info(
            f"[SAFE BACKUP] Rear: {rear_clear:.2f}m, L/R: {left_clear:.2f}/{right_clear:.2f}m "
            f"→ Backup {safe_duration:.2f}s + turn {direction}"
        )
        
        return {
            'action': 'safe_backup_recovery',
            'parameters': {
                'linear_velocity': -0.20,  # CHANGED: Slower backup (was -0.25)
                'angular_velocity': angular,
                'duration': safe_duration  # CHANGED: Adaptive (was fixed 1.2s)
            },
            'reason': f'safe_backup_{direction}',
            'rear_clearance': rear_clear  # ADDED: For monitoring
        }
    
    # =========================================================================
    # Statistics & Debugging
    # =========================================================================
    
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