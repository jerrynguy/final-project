"""
LiDAR Safety Monitor Module (REDESIGNED v2)
Single critical abort layer with hysteresis to prevent oscillation.
"""

import time
import logging
import numpy as np
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class LidarSafetyMonitor:
    """
    Simplified critical-only safety monitor with hysteresis.
    
    Hysteresis prevents oscillation:
    - ABORT at 0.12m (critical threshold)
    - RESUME at 0.25m (hysteresis gap)
    - Cooldown period after abort (2.0s)
    
    This prevents back/forward loop when approaching obstacles.
    """
    
    # =========================================================================
    # Thresholds (with hysteresis)
    # =========================================================================
    CRITICAL_DISTANCE = 0.2  # 20cm - Hardware protection (abort)
    RESUME_DISTANCE = 0.35    # 35cm - Resume threshold (hysteresis gap)
    
    def __init__(self):
        """Initialize monitor with state tracking."""
        self.abort_count = 0
        
        # Hysteresis state tracking
        self.last_abort_time = 0.0      # Timestamp of last abort
        self.cooldown_duration = 2.0    # Cooldown period (seconds)
    
    def check_critical_abort(self, lidar_data) -> Dict:
        """
        Check for critical distance with hysteresis to prevent oscillation.
        
        State Machine:
        1. NORMAL: Can move freely, check for obstacles
        2. ABORT: Obstacle < 0.12m → Emergency backup
        3. COOLDOWN: After abort, wait until distance > 0.25m OR timeout
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
            # Get minimum distance from 360° LIDAR scan
            min_distance = self._get_min_distance(lidar_data)
            current_time = time.time()
            
            # Calculate time since last abort
            time_since_abort = current_time - self.last_abort_time
            in_cooldown = time_since_abort < self.cooldown_duration
            
            # =====================================================================
            # STATE 1: Check if need to ABORT (critical distance reached)
            # =====================================================================
            if min_distance < self.CRITICAL_DISTANCE and not in_cooldown:
                # NEW ABORT - start cooldown timer
                self.last_abort_time = current_time
                self.abort_count += 1
                
                logger.error(
                    f"[ABORT #{self.abort_count}] Distance: {min_distance:.3f}m "
                    f"→ EMERGENCY BACKUP"
                )
                
                return {
                    'abort': True,
                    'command': self._emergency_backup(),
                    'min_distance': min_distance,
                    'state': 'abort'
                }
            
            # =====================================================================
            # STATE 2: In COOLDOWN after abort
            # =====================================================================
            elif in_cooldown:
                # Check if safe to resume (distance > hysteresis threshold)
                if min_distance > self.RESUME_DISTANCE:
                    # Sufficient clearance - clear cooldown and resume
                    logger.info(
                        f"[RESUME] Clearance: {min_distance:.3f}m "
                        f"(after {time_since_abort:.1f}s cooldown) → NORMAL"
                    )
                    self.last_abort_time = 0.0  # Reset cooldown timer
                    
                    return {
                        'abort': False,
                        'command': None,
                        'min_distance': min_distance,
                        'state': 'normal'
                    }
                
                else:
                    # Still too close - stay in cooldown (pause movement)
                    logger.debug(
                        f"[COOLDOWN] {time_since_abort:.1f}s elapsed - "
                        f"Distance: {min_distance:.3f}m (need > {self.RESUME_DISTANCE}m)"
                    )
                    
                    return {
                        'abort': False,  # Don't abort again
                        'command': self._pause_command(),  # Stop briefly
                        'min_distance': min_distance,
                        'state': 'cooldown'
                    }
            
            # =====================================================================
            # STATE 3: NORMAL operation (safe to proceed)
            # =====================================================================
            else:
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
    
    def _get_min_distance(self, lidar_data) -> float:
        """
        Extract minimum distance from LIDAR scan (360° coverage).
        
        Args:
            lidar_data: LaserScan message
            
        Returns:
            float: Minimum valid distance in meters
        """
        try:
            ranges = lidar_data.ranges
            
            # Filter out invalid readings (NaN, Inf)
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
    
    # =========================================================================
    # Command Generators
    # =========================================================================
    
    def _emergency_backup(self) -> Dict:
        """
        Generate emergency backup command.
        
        Returns:
            dict: Navigation command to back away from obstacle
        """
        return {
            'action': 'move_backward',
            'parameters': {
                'linear_velocity': -0.3,   # Back up at 0.3 m/s
                'angular_velocity': 0.0,   # Straight back
                'duration': 0.8            # For 0.8 seconds (~24cm)
            },
            'reason': 'critical_abort_backup'
        }
    
    def _emergency_stop(self) -> Dict:
        """
        Generate emergency stop command.
        
        Returns:
            dict: Navigation command to stop immediately
        """
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
        """
        Generate brief pause command during cooldown.
        
        Returns:
            dict: Navigation command to pause briefly
        """
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.3  # Short pause
            },
            'reason': 'cooldown_pause'
        }
    
    # =========================================================================
    # Statistics & Debugging
    # =========================================================================
    
    def get_stats(self) -> Dict:
        """
        Get safety monitor statistics.
        
        Returns:
            dict: Statistics about abort events
        """
        return {
            'total_aborts': self.abort_count,
            'in_cooldown': time.time() - self.last_abort_time < self.cooldown_duration,
            'cooldown_remaining': max(
                0.0, 
                self.cooldown_duration - (time.time() - self.last_abort_time)
            )
        }
    
    def reset_stats(self):
        """Reset abort counter (for testing)."""
        self.abort_count = 0
        self.last_abort_time = 0.0
        logger.info("[STATS RESET] Abort counter cleared")