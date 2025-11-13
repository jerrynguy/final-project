"""
Spatial Detector Module
LiDAR-based obstacle detection and spatial analysis for safe navigation.
"""

import cv2
import time
import logging
import numpy as np
from enum import Enum
from ultralytics import YOLO
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyValidator

try:
    from sensor_msgs.msg import LaserScan
    LIDAR_AVAILABLE = True
except ImportError:
    LIDAR_AVAILABLE = False
    class LaserScan:
        pass

logger = logging.getLogger(__name__)


# =============================================================================
# Object Type Enumeration
# =============================================================================

class ObjectType(Enum):
    """Types of detected objects in environment."""
    OBSTACLE = "obstacle"
    WALL = "wall"
    PERSON = "person"
    UNKNOWN = "unknown"


# =============================================================================
# Spatial Detector
# =============================================================================

class SpatialDetector:
    """
    LiDAR-based spatial analysis for obstacle detection and path planning.
    """
    
    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        """
        Initialize spatial detector.
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        self.safety_validator = SafetyValidator()
        
        # Frame processing configuration
        self.frame_skip = 1
        self._frame_counter = 0
        self._last_result = None
        
        # LiDAR configuration
        self.use_lidar = True
        self.lidar_max_range = 3.5
        self.lidar_min_range = 0.12
        self.lidar_critical_distance = self.safety_validator.CRITICAL_DISTANCE
        self.lidar_warning_distance = self.safety_validator.WARNING_DISTANCE
        
        # Navigation thresholds
        self.safe_distance_threshold = self.safety_validator.SAFE_DISTANCE

    def get_full_lidar_scan(self, scan_msg: LaserScan) -> Dict[int, float]:
        """
        Get complete 360° LiDAR scan indexed by angle.
        """
        if not LIDAR_AVAILABLE or scan_msg is None:
            return {}
        
        angle_scan = {}
        
        try:
            angle_min = scan_msg.angle_min
            angle_increment = scan_msg.angle_increment
            ranges = scan_msg.ranges
            
            for i, distance in enumerate(ranges):
                # Filter invalid readings
                if np.isnan(distance) or np.isinf(distance):
                    continue
                
                if distance < self.lidar_min_range or distance > self.lidar_max_range:
                    continue
                
                # Calculate angle in degrees
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle_rad)
                
                # Normalize to [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                # Round to integer for indexing
                angle_key = int(round(angle_deg))
                
                # Keep closest distance if multiple readings at same angle
                if angle_key not in angle_scan or distance < angle_scan[angle_key]:
                    angle_scan[angle_key] = distance
            
            logger.debug(f"[LIDAR FULL] Captured {len(angle_scan)} angle readings")
            return angle_scan
            
        except Exception as e:
            logger.error(f"Full LiDAR scan failed: {e}")
            return {}


    def query_distance_at_angle(
        self, 
        angle_scan: Dict[int, float], 
        target_angle: float,
        tolerance: int = 5
    ) -> Optional[float]:
        """
        Query LiDAR distance at specific angle with tolerance.
        """
        target_int = int(round(target_angle))
        
        # Direct lookup first
        if target_int in angle_scan:
            return angle_scan[target_int]
        
        # Search within tolerance
        for offset in range(1, tolerance + 1):
            if target_int + offset in angle_scan:
                return angle_scan[target_int + offset]
            if target_int - offset in angle_scan:
                return angle_scan[target_int - offset]
        
        return None

    def process_lidar_scan(
        self, 
        scan_msg: LaserScan,
        return_full_scan: bool = True
    ) -> tuple[List[Dict], Dict[int, float]]:
        """
        Process LiDAR scan into obstacles + full 360° map.
        """
        if not LIDAR_AVAILABLE or scan_msg is None:
            return [], {}
        
        # Get full 360° scan first
        full_scan = self.get_full_lidar_scan(scan_msg) if return_full_scan else {}
        
        obstacles = []
        
        try:
            # Group by zones for navigation obstacles
            zone_obstacles = {
                'front': [],
                'left': [],
                'right': [],
                'back': []
            }
            
            for angle_deg, distance in full_scan.items():
                # Classify into zones
                if -45 <= angle_deg <= 45:
                    zone = 'front'
                elif 45 < angle_deg <= 135:
                    zone = 'left'
                elif -135 <= angle_deg < -45:
                    zone = 'right'
                else:
                    zone = 'back'
                
                zone_obstacles[zone].append({
                    'distance': distance,
                    'angle': angle_deg
                })
            
            # Create obstacle for each zone (use closest point)
            for zone, obs_list in zone_obstacles.items():
                if not obs_list:
                    continue
                
                closest = min(obs_list, key=lambda x: x['distance'])
                distance = closest['distance']
                angle = closest['angle']
                
                # Determine threat level
                if distance < self.lidar_critical_distance:
                    threat = 'high'
                elif distance < self.lidar_warning_distance:
                    threat = 'medium'
                else:
                    threat = 'low'
                
                obstacle = {
                    'type': 'obstacle',
                    'position': zone,
                    'size': 'lidar_point',
                    'distance_estimate': distance,
                    'angle': angle,
                    'threat_level': threat,
                    'source': 'lidar'
                }
                obstacles.append(obstacle)
            
            if obstacles:
                logger.info(f"[LIDAR] Detected {len(obstacles)} zone obstacles")
                for obs in obstacles:
                    logger.info(
                        f"  - {obs['position']}: {obs['distance_estimate']:.2f}m "
                        f"({obs['threat_level']}) at {obs['angle']:.1f}°"
                    )
            
            return obstacles, full_scan
            
        except Exception as e:
            logger.error(f"LIDAR processing failed: {e}")
            return [], {}
    
    def _calculate_clearances_from_lidar(self, lidar_obstacles: List[Dict]) -> Dict[str, float]:
        """
        Calculate clearance distances in each direction.
        """
        clearances = {
            'forward': self.lidar_max_range,
            'left': self.lidar_max_range,
            'right': self.lidar_max_range
        }
        
        for obs in lidar_obstacles:
            position = obs.get('position', '')
            distance = obs.get('distance_estimate', 999)
            
            # Update minimum clearance for each direction
            if position == 'front':
                clearances['forward'] = min(clearances['forward'], distance)
            elif position == 'left':
                clearances['left'] = min(clearances['left'], distance)
            elif position == 'right':
                clearances['right'] = min(clearances['right'], distance)
        
        logger.debug(
            f"[CLEARANCE] forward={clearances['forward']:.2f}m, "
            f"left={clearances['left']:.2f}m, right={clearances['right']:.2f}m"
        )
        
        return clearances
    
    def _calculate_safety_score_from_obstacles(self, obstacles: List[Dict]) -> int:
        """
        Calculate safety score based on obstacle threat levels.
        """
        if not obstacles:
            return 10
        
        # Count threats by level
        high_threats = sum(1 for obs in obstacles if obs.get('threat_level') == 'high')
        medium_threats = sum(1 for obs in obstacles if obs.get('threat_level') == 'medium')
        low_threats = sum(1 for obs in obstacles if obs.get('threat_level') == 'low')
        
        # Calculate score (higher threats = lower score)
        base_score = 10
        base_score -= high_threats * 3
        base_score -= medium_threats * 2
        base_score -= low_threats * 1
        
        # Clamp to [0, 10]
        final_score = max(0, min(10, base_score))
        
        logger.debug(
            f"[SAFETY] Score={final_score}/10 "
            f"(high={high_threats}, med={medium_threats}, low={low_threats})"
        )
        
        return final_score
    
    def _get_safe_fallback(self) -> Dict[str, any]:
        """
        Generate safe fallback result when analysis fails.
        """
        logger.warning("[FALLBACK] Using safe fallback result")
        return {
            'obstacles': [{
                'type': 'obstacle',
                'position': 'front',
                'size': 'unknown',
                'distance_estimate': self.lidar_critical_distance,
                'threat_level': "high",
                'source': 'fallback'
            }],
            'clear_paths': [],
            'safety_score': 1,
            'recommended_direction': 'stop',
            'spatial_confidence': 0.3,
            'lidar_used': False,
            'map_info': {},
            'exploration_frontiers': [],
            'clearances': {
                'forward': self.lidar_critical_distance,
                'left': self.lidar_critical_distance,
                'right': self.lidar_critical_distance
            }
        }
    
    def _fast_spatial_analysis(
        self,
        frame: np.ndarray,
        lidar_scan=None,
    ) -> Dict[str, any]:
        """
        Execute fast spatial analysis using LiDAR data.
        """
        try:
            logger.info("=" * 60)
            logger.info("[SPATIAL ANALYSIS START]")
            
            # Process LiDAR scan
            lidar_obstacles = []
            full_lidar_scan = {}
            if lidar_scan is not None:
                lidar_obstacles, full_lidar_scan = self.process_lidar_scan(lidar_scan, return_full_scan=True)
                logger.info(f"[DEBUG] Full LiDAR scan: {len(full_lidar_scan)} angles")

            else:
                logger.warning("[DEBUG] No LIDAR data available!")
            
            # Calculate clearances in each direction
            clearances = self._calculate_clearances_from_lidar(lidar_obstacles)
            logger.info(f"[DEBUG] Clearances: {clearances}")
            
            # ADAPTIVE THRESHOLD: Use lower threshold if all clearances are moderate
            max_clearance = max(clearances.values()) if clearances else 0
            logger.info(f"[DEBUG] Max clearance: {max_clearance:.2f}m")
            
            if max_clearance < self.safe_distance_threshold:
                adaptive_threshold = self.lidar_critical_distance * 1.5  # 0.45m
                logger.warning(f"[ADAPTIVE] Using relaxed threshold {adaptive_threshold:.2f}m (max_clearance={max_clearance:.2f}m < safe={self.safe_distance_threshold:.2f}m)")
            else:
                adaptive_threshold = self.safe_distance_threshold  # 1.0m
                logger.info(f"[ADAPTIVE] Using normal threshold {adaptive_threshold:.2f}m")
            
            # Nav2 handles direction finding - just use default
            safe_directions = []  # Not used anymore
            recommended_direction = 'forward'  # Default - Nav2 decides
            logger.info(f"[DEBUG] Nav2 will handle direction planning")
            
            # Calculate overall safety score
            safety_score = self._calculate_safety_score_from_obstacles(lidar_obstacles)
            logger.info(f"[DEBUG] Safety score: {safety_score}/10")
            
            # Provide fallback clear_paths if empty
            if not safe_directions:
                logger.warning("[DEBUG] No safe directions found, adding 'cautious_forward' fallback")
                final_clear_paths = ['cautious_forward']
            else:
                final_clear_paths = safe_directions
            
            logger.info(f"[DEBUG] Final clear_paths: {final_clear_paths}")
            
            # Build result
            result = {
                'obstacles': lidar_obstacles,
                'clear_paths': final_clear_paths,
                'safety_score': safety_score,
                'recommended_direction': recommended_direction,
                'spatial_confidence': 0.9,
                'lidar_used': len(lidar_obstacles) > 0,
                'clearances': clearances,
                'full_lidar_scan': full_lidar_scan
            }
            
            logger.info("[SPATIAL ANALYSIS END]")
            logger.info("=" * 60)
            
            return result
            
        except Exception as e:
            logger.error(f"Spatial analysis failed: {e}", exc_info=True)
            return self._get_safe_fallback()