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
                        'command': self._smart_recovery_command(angle, clearances),
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
        Analyze clearance in left/right/front directions.
        
        Returns:
            dict: {'front': distance, 'left': distance, 'right': distance}
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
    
    # =========================================================================
    # Command Generators
    # =========================================================================
    def _smart_recovery_command(self, obstacle_angle: float, clearances: Dict) -> Dict:
        """
        Generate smart recovery command that turns toward open space.
        
        Strategy:
        1. Check if robot is in tight corner (both sides blocked)
        2. If yes: straight backup only (no rotation)
        3. If no: turn toward more open side while backing up
        """
        left_clear = clearances.get('left', 0)
        right_clear = clearances.get('right', 0)
        
        # Phát hiện góc chật - cần ít nhất 30cm một bên để xoay an toàn
        MIN_SAFE_CLEARANCE = 0.30  # 30cm minimum to safely turn
        
        if left_clear < MIN_SAFE_CLEARANCE and right_clear < MIN_SAFE_CLEARANCE:
            # CORNER SITUATION: Both sides blocked - DON'T rotate!
            logger.warning(
                f"[TIGHT CORNER DETECTED] Left: {left_clear:.2f}m, Right: {right_clear:.2f}m "
                f"→ STRAIGHT BACKUP ONLY (no rotation)"
            )
            return {
                'action': 'minimal_recovery',
                'parameters': {
                    'linear_velocity': -0.15,  # Gentle straight backup
                    'angular_velocity': 0.0,   # NO rotation in tight space
                    'duration': 0.8            # Shorter duration
                },
                'reason': 'tight_corner_minimal_recovery'
            }
        
        # CHANGED: Có đủ không gian để xoay - dùng logic thông minh
        clearance_diff = left_clear - right_clear
        
        # Decision: Turn toward more open side
        if clearance_diff > 0.2:
            angular = 0.7  # Strong left turn
            direction = "left"
        elif clearance_diff < -0.2:
            angular = -0.7  # Strong right turn
            direction = "right"
        else:
            # Equal clearance: turn away from obstacle angle
            angular = 0.8 if obstacle_angle > 0 else -0.8
            direction = "away_from_obstacle"
        
        logger.info(
            f"[SMART RECOVERY] Clearances - Left: {left_clear:.2f}m, Right: {right_clear:.2f}m "
            f"→ Turn {direction} (angular: {angular:.2f})"
        )
        
        return {
            'action': 'smart_recovery',
            'parameters': {
                'linear_velocity': -0.20,  # CHANGED: Reduced from -0.25 (gentler)
                'angular_velocity': angular,  # Turn toward open space
                'duration': 1.0  # CHANGED: Reduced from 1.2 (less rotation)
            },
            'reason': f'smart_recovery_{direction}'
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