"""
LiDAR Safety Monitor Module (Simplified)
Provides obstacle detection and clearance analysis - NO decision making.
"""

import logging
import numpy as np
from typing import Dict, Optional, List, Tuple

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds

logger = logging.getLogger(__name__)


class LidarSafetyMonitor:
    """
    Simplified safety monitor - Sensor reader ONLY.
    
    Responsibilities:
    - Read LIDAR data
    - Provide 360° clearance analysis
    - Check rear clearance for backup safety
    
    NOT responsible for:
    - Decision making (moved to navigation_reasoner)
    - Escape planning (removed)
    - State management (removed)
    """
    
    def __init__(self):
        """Initialize monitor."""
        self.thresholds = SafetyThresholds
        
        # Simple tracking (no state machine)
        self._last_min_dist = None
        self._movement_type = None
    
    def get_min_distance(self, lidar_data) -> float:
        """
        Extract minimum distance from LIDAR scan.
        
        Returns:
            Minimum valid distance in meters, or inf if no valid readings.
        """
        try:
            ranges = lidar_data.ranges
            valid_ranges = [
                r for r in ranges 
                if not (np.isnan(r) or np.isinf(r))
            ]
            
            if not valid_ranges:
                logger.warning("[LIDAR] No valid ranges")
                return float('inf')
            
            return min(valid_ranges)
            
        except Exception as e:
            logger.error(f"[LIDAR] Failed to get min distance: {e}")
            return float('inf')
    
    def _get_obstacles_with_angles(self, lidar_data) -> List[Tuple[float, float]]:
        """
        Extract obstacles with their angles from LIDAR scan.
        
        Returns:
            List of (angle_deg, distance) tuples, sorted by distance.
        """
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            obstacles = []
            
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
                
                obstacles.append((angle_deg, distance))
            
            # Sort by distance (closest first)
            obstacles.sort(key=lambda x: x[1])
            return obstacles
            
        except Exception as e:
            logger.error(f"[LIDAR] Failed to extract obstacles: {e}")
            return []
    
    def _analyze_360_clearances(
        self, 
        obstacles: List[Tuple[float, float]]
    ) -> Dict[int, float]:
        """
        Analyze clearance for 12 sectors (30° each).
        
        Returns:
            {0: 1.4, 30: 1.2, 60: 1.5, ...} - clearance in meters per sector
            
        NO SCORING - just raw clearance values.
        """
        SECTOR_SIZE = 15  # degrees
        NUM_SECTORS = 24
        MAX_LIDAR_RANGE = 3.5
        
        # Initialize all sectors with max range
        sector_clearances = {
            sector * SECTOR_SIZE: MAX_LIDAR_RANGE
            for sector in range(NUM_SECTORS)
        }
        
        # Map obstacles to sectors (take minimum distance per sector)
        for angle_deg, distance in obstacles:
            # Normalize angle to [0, 360)
            normalized_angle = angle_deg % 360
            
            # Find corresponding sector (0, 30, 60, ..., 330)
            sector_index = int(normalized_angle // SECTOR_SIZE)
            sector_angle = sector_index * SECTOR_SIZE
            
            # Update sector with minimum distance
            current_min = sector_clearances[sector_angle]
            sector_clearances[sector_angle] = min(current_min, distance)
        
        return sector_clearances
    
    def check_rear_clearance(self, lidar_data) -> Optional[float]:
        """
        Check rear hemisphere clearance for safe backup.
        
        Returns:
            Minimum distance in ±120° arc behind robot, or None if no data.
            
        Simple measurement - NO complex validation.
        """
        if lidar_data is None:
            return None
        
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            rear_distances = []
            
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
                
                # Rear arc: |angle| > 120°
                if abs(angle_deg) > 120:
                    rear_distances.append(distance)
            
            if not rear_distances:
                return None
            
            min_rear = min(rear_distances)
            
            logger.debug(f"[REAR CHECK] Minimum rear clearance: {min_rear:.3f}m")
            return min_rear
            
        except Exception as e:
            logger.error(f"[REAR CHECK] Failed: {e}")
            return None
    
    def get_stats(self) -> Dict:
        """
        Get simple statistics.
        
        Returns:
            {
                'last_min_distance': float or None,
                'movement_state': str
            }
        """
        return {
            'last_min_distance': self._last_min_dist,
            'movement_state': self._movement_type or 'unknown'
        }
    
    def reset_stats(self):
        """Reset statistics."""
        self._last_min_dist = None
        self._movement_type = None
        logger.info("[LIDAR] Stats reset")

    def get_obstacle_info(self, lidar_data) -> Dict:
        """
        Get complete obstacle information from LIDAR.
        
        Returns:
            {
                'min_distance': float,
                'clearances': Dict[int, float],  # 360° sectors (0°, 30°, 60°, ...)
                'obstacle_detected': bool,
                'is_critical': bool
            }
        """
        if lidar_data is None:
            logger.warning("[LIDAR] No data available")
            return {
                'min_distance': float('inf'),
                'clearances': {},
                'obstacle_detected': False,
                'is_critical': False
            }
        
        try:
            # Get minimum distance
            min_dist = self.get_min_distance(lidar_data)
            
            # Extract obstacles with angles
            obstacles = self._get_obstacles_with_angles(lidar_data)
            
            # Analyze 360° clearances
            clearances = self._analyze_360_clearances(obstacles)
            
            # Update tracking
            self._last_min_dist = min_dist
            
            return {
                'min_distance': min_dist,
                'clearances': clearances,
                'obstacle_detected': min_dist < SafetyThresholds.WARNING_ZONE,
                'is_critical': min_dist < SafetyThresholds.CRITICAL_ABORT
            }
            
        except Exception as e:
            logger.error(f"[LIDAR] Failed to get obstacle info: {e}")
            return {
                'min_distance': float('inf'),
                'clearances': {},
                'obstacle_detected': False,
                'is_critical': False
            }