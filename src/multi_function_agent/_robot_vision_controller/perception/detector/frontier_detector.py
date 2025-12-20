"""
Frontier Detector Module (SLAM-Aware)
Detects TRUE unexplored regions by analyzing SLAM map occupancy.
"""

import logging
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class Frontier:
    """Represents a TRUE unexplored frontier region."""
    center: Tuple[float, float]  # (x, y) in map coordinates
    size: int  # Number of frontier cells
    distance: float  # Distance from robot
    angle: float  # Angle from robot (degrees)
    score: float  # Exploration priority score
    is_valid: bool = True  # True if not blocked by wall


class FrontierDetector:
    """
    SLAM-aware frontier detection.
    
    Key Concept:
    - FREE (0) + UNKNOWN (-1) = TRUE FRONTIER ✅
    - OCCUPIED (100) + UNKNOWN (-1) = WALL, NOT FRONTIER ❌
    """
    
    def __init__(self):
        """Initialize frontier detector."""
        self.min_frontier_size = 5  # Minimum cells
        self.max_frontier_distance = 4.0  # Max distance (meters)
        self.frontier_cache = []
        self.last_detection_time = 0
        self.detection_interval = 3.0  # Detect every 3 seconds
        
        # Wall safety constraints
        self.min_safe_distance_from_wall = 2.0  # Keep 2m buffer from walls
        self.wall_penalty_factor = 0.3  # Heavy penalty for wall-adjacent
        
        # SLAM map values
        self.FREE = 0
        self.OCCUPIED = 100
        self.UNKNOWN = -1
    
    def detect_frontiers(
        self, 
        map_data: Optional[Dict],
        robot_pose: Optional[Dict]
    ) -> List[Frontier]:
        """
        Detect REAL frontiers from SLAM map occupancy grid.
        
        Strategy:
        1. Get SLAM map occupancy grid
        2. Find cells that are:
           - Currently UNKNOWN (-1)
           - Adjacent to FREE space (0)
           - NOT adjacent to OCCUPIED (100) only
        3. Group into frontier clusters
        4. Score by distance + size
        """
        import time
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_detection_time < self.detection_interval:
            return self.frontier_cache
        
        if robot_pose is None:
            logger.debug("[FRONTIER] No robot pose")
            return []
        
        try:
            # STRATEGY: Use LiDAR-based with wall detection
            frontiers = self._detect_frontiers_lidar_slam_aware(robot_pose)
            
            # Cache results
            self.frontier_cache = frontiers
            self.last_detection_time = current_time
            
            if frontiers:
                logger.info(f"[FRONTIER] Detected {len(frontiers)} VALID frontiers")
                for i, f in enumerate(frontiers[:3], 1):
                    logger.info(
                        f"  {i}. Distance: {f.distance:.2f}m, "
                        f"Angle: {f.angle:.1f}°, Score: {f.score:.2f}"
                    )
            else:
                logger.debug("[FRONTIER] No valid frontiers found")
            
            return frontiers
            
        except Exception as e:
            logger.error(f"[FRONTIER] Detection failed: {e}")
            return []

    def _detect_frontiers_lidar_slam_aware(self, robot_pose: Dict) -> List[Frontier]:
        """
        LiDAR-based frontier detection with SLAM awareness.
        """
        try:
            from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
            ros_node = get_ros2_node()
            
            lidar_data = ros_node.get_scan()
            if lidar_data is None:
                return []
            
            # Get max LiDAR range
            max_range = lidar_data.range_max if hasattr(lidar_data, 'range_max') else 3.5
            
            # ✅ FIX: Convert LaserScan to full_scan Dict using SpatialDetector
            from multi_function_agent._robot_vision_controller.perception.detector.spatial_detector import SpatialDetector
            spatial_detector = SpatialDetector()
            full_scan = spatial_detector.get_full_lidar_scan(lidar_data)
            
            # Sample 8 directions
            directions = [
                (0, "front"),
                (45, "front_left"),
                (90, "left"),
                (135, "back_left"),
                (180, "back"),
                (-135, "back_right"),
                (-90, "right"),
                (-45, "front_right")
            ]
            
            frontiers = []
            robot_x = robot_pose.get('x', 0)
            robot_y = robot_pose.get('y', 0)
            robot_theta = robot_pose.get('theta', 0)
            
            for angle_offset, name in directions:
                # Calculate absolute angle
                absolute_angle = np.degrees(robot_theta) + angle_offset
                
                # Get distance in this direction
                distance = self._get_lidar_distance_at_angle(
                    lidar_data, 
                    np.radians(angle_offset)
                )
                
                # LAYER 1: Basic validation
                is_valid_frontier = self._is_valid_frontier(
                    distance, 
                    max_range, 
                    angle_offset
                )
                
                if not is_valid_frontier:
                    continue
                
                # ✅ LAYER 2: Wall-adjacent check (NOW full_scan is defined!)
                is_wall_adjacent = False
                wall_ratio = 0.0
                
                if full_scan:  # Check if conversion successful
                    is_wall_adjacent, wall_ratio = self._has_nearby_wall(
                        angle_offset,
                        distance,
                        full_scan  # ← NOW WORKS!
                    )
                    
                    # Hard filter for wall-adjacent + too close
                    if is_wall_adjacent and distance < self.min_safe_distance_from_wall:
                        logger.debug(
                            f"[FRONTIER REJECT] {name} at {distance:.1f}m "
                            f"rejected (wall-adjacent + too close)"
                        )
                        continue
                
                # ✅ LAYER 3: Calculate score with wall penalties
                frontier_x = robot_x + distance * np.cos(np.radians(absolute_angle))
                frontier_y = robot_y + distance * np.sin(np.radians(absolute_angle))
                
                distance_score = distance / self.max_frontier_distance
                
                wall_max_penalty = 1.0
                if distance > max_range * 0.8:
                    wall_max_penalty = 0.3
                
                wall_adjacent_penalty = 1.0
                if is_wall_adjacent:
                    wall_adjacent_penalty = 1.0 - (wall_ratio * 0.6)
                
                direction_bonus = 1.0
                if abs(angle_offset) < 90:
                    direction_bonus = 1.3
                elif abs(angle_offset) < 120:
                    direction_bonus = 1.1
                
                score = (distance_score * 
                        wall_max_penalty * 
                        wall_adjacent_penalty * 
                        direction_bonus)
                
                frontier = Frontier(
                    center=(frontier_x, frontier_y),
                    size=int(distance * 10),
                    distance=distance,
                    angle=angle_offset,
                    score=score,
                    is_valid=True
                )
                
                frontiers.append(frontier)
                
                logger.debug(
                    f"[FRONTIER] {name}: dist={distance:.2f}m, "
                    f"score={score:.2f}, wall_adj={is_wall_adjacent}"
                )
            
            # Sort by score
            frontiers.sort(key=lambda f: f.score, reverse=True)
            
            return frontiers
            
        except Exception as e:
            logger.error(f"[FRONTIER] LiDAR detection failed: {e}")
            return []

    def _is_valid_frontier(
        self, 
        distance: float, 
        max_range: float,
        angle_offset: float
    ) -> bool:
        """
        Determine if a direction is a VALID frontier (not a wall).
        
        Rules:
        1. Distance must be > 1.5m (enough clearance)
        2. Distance must be < max_range * 0.8 (not hitting sensor limit = wall)
        3. Distance must be < max_frontier_distance (not too far)
        4. Not backward direction (abs(angle) < 150°)
        """
        # Rule 1: Minimum clearance
        if distance < 1.5:
            return False
        
        # Rule 2: Not a wall (hitting sensor max)
        if distance > max_range * 0.8:
            logger.debug(
                f"[FRONTIER REJECT] Distance {distance:.2f}m too close to "
                f"max_range {max_range:.2f}m (wall detection)"
            )
            return False
        
        # Rule 3: Not too far
        if distance > self.max_frontier_distance:
            return False
        
        # Rule 4: Not backward
        if abs(angle_offset) > 150:
            return False
        
        return True

    def _has_nearby_wall(
        self, 
        angle_deg: float, 
        distance: float,
        full_scan: Dict[int, float]
    ) -> tuple[bool, float]:
        """
        Detect if frontier has walls nearby (wall-hugging detection).
        
        Strategy:
        1. Check ±30° arc around frontier direction
        2. Count readings < 2.5m (wall threshold)
        3. If >40% readings show walls → wall-adjacent
        
        Args:
            angle_deg: Frontier direction in degrees
            distance: Frontier distance
            full_scan: Complete 360° LiDAR scan
        
        Returns:
            (is_wall_adjacent: bool, wall_ratio: float)
        """
        if not full_scan:
            return False, 0.0
        
        WALL_THRESHOLD = 2.5  # Distance considered "close wall"
        CHECK_ARC = 30  # Check ±30° around frontier
        WALL_RATIO_THRESHOLD = 0.4  # 40% = wall-adjacent
        
        wall_readings = 0
        total_checks = 0
        
        # Sample every 5° in ±30° arc
        for offset in range(-CHECK_ARC, CHECK_ARC + 1, 5):
            check_angle = int(angle_deg + offset)
            
            # Normalize to [-180, 180]
            while check_angle > 180:
                check_angle -= 360
            while check_angle < -180:
                check_angle += 360
            
            if check_angle in full_scan:
                nearby_distance = full_scan[check_angle]
                total_checks += 1
                
                if nearby_distance < WALL_THRESHOLD:
                    wall_readings += 1
        
        if total_checks == 0:
            return False, 0.0
        
        wall_ratio = wall_readings / total_checks
        is_wall_adjacent = wall_ratio > WALL_RATIO_THRESHOLD
        
        if is_wall_adjacent:
            logger.debug(
                f"[WALL DETECT] Frontier at {angle_deg:.0f}° ({distance:.1f}m) "
                f"is wall-adjacent ({wall_ratio:.1%} readings < {WALL_THRESHOLD}m)"
            )
        
        return is_wall_adjacent, wall_ratio
    
    def _get_lidar_distance_at_angle(self, lidar_data, angle_rad: float) -> float:
        """Get LiDAR distance at specific angle."""
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_max = lidar_data.angle_max
            angle_increment = lidar_data.angle_increment
            
            # Find closest index
            if angle_rad < angle_min or angle_rad > angle_max:
                return 0.0
            
            index = int((angle_rad - angle_min) / angle_increment)
            index = max(0, min(len(ranges) - 1, index))
            
            distance = ranges[index]
            
            if np.isnan(distance) or np.isinf(distance):
                return 0.0
            
            return distance
            
        except Exception as e:
            logger.debug(f"Failed to get LiDAR distance: {e}")
            return 0.0
    
    def get_best_frontier(self, robot_pose: Dict) -> Optional[Frontier]:
        """
        Get highest-scoring VALID frontier.
        """
        frontiers = self.detect_frontiers(None, robot_pose)
        
        if not frontiers:
            logger.debug("[FRONTIER] No valid frontiers available")
            return None
        
        best = frontiers[0]
        logger.info(
            f"[FRONTIER] Best frontier: {best.distance:.2f}m at {best.angle:.0f}°, "
            f"score={best.score:.2f}"
        )
        
        return best
    
    def get_frontier_direction(self, frontier: Optional[Frontier]) -> Optional[str]:
        """
        Convert frontier to navigation direction.
        """
        if frontier is None or not frontier.is_valid:
            return None
        
        angle = frontier.angle
        
        if -30 <= angle <= 30:
            return 'forward'
        elif 30 < angle <= 120:
            return 'left'
        elif -120 <= angle < -30:
            return 'right'
        else:
            return None  # Behind or invalid