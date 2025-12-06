"""
LiDAR Safety Monitor Module (REDESIGNED)
Single critical abort layer - Nav2 handles everything else.
"""

import logging
import numpy as np
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class LidarSafetyMonitor:
    """
    Simplified critical-only safety monitor.
    Philosophy: Trust Nav2, only intervene at hardware limits.
    """
    
    # SINGLE THRESHOLD - hardware protection only
    CRITICAL_DISTANCE = 0.12  # 12cm - absolute minimum
    
    def __init__(self):
        self.abort_count = 0
    
    def check_critical_abort(self, lidar_data) -> Dict:
        """
        Check ONLY for immediate collision (< 12cm).
        
        Returns:
            {'abort': bool, 'command': dict, 'min_distance': float}
        """
        if lidar_data is None:
            logger.error("[CRITICAL] No LIDAR - ABORT")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0
            }
        
        try:
            ranges = lidar_data.ranges
            
            # Find minimum distance (all 360°)
            valid_ranges = [r for r in ranges if not (np.isnan(r) or np.isinf(r))]
            
            if not valid_ranges:
                return {'abort': False, 'command': None, 'min_distance': 999.0}
            
            min_distance = min(valid_ranges)
            
            # ONLY abort if critically close
            if min_distance < self.CRITICAL_DISTANCE:
                self.abort_count += 1
                logger.error(f"[CRITICAL ABORT #{self.abort_count}] Obstacle at {min_distance:.3f}m")
                
                return {
                    'abort': True,
                    'command': self._emergency_backup(),
                    'min_distance': min_distance
                }
            
            return {'abort': False, 'command': None, 'min_distance': min_distance}
            
        except Exception as e:
            logger.error(f"Safety check failed: {e}")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0
            }
    
    def get_min_distance(self, lidar_data) -> float:
        """Utility: Get minimum distance from LIDAR."""
        if lidar_data is None:
            return float('inf')
        
        try:
            ranges = lidar_data.ranges
            valid_ranges = [r for r in ranges if not (np.isnan(r) or np.isinf(r))]
            return min(valid_ranges) if valid_ranges else float('inf')
        except:
            return float('inf')
    
    def _emergency_backup(self) -> Dict:
        """Emergency backup when critically close."""
        return {
            'action': 'move_backward',
            'parameters': {
                'linear_velocity': -0.3,
                'angular_velocity': 0.0,
                'duration': 0.8
            },
            'reason': 'critical_abort'
        }
    
    def _emergency_stop(self) -> Dict:
        """Emergency stop."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'reason': 'emergency_stop'
        }