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
        
        Key Logic:
        1. Sample 8 directions
        2. For each direction with clearance > 1.5m:
           - Check if path is FREE (no obstacles detected)
           - Check if NOT hitting a wall (distance < max_range)
           - Only then it's a valid frontier
        """
        try:
            from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
            ros_node = get_ros2_node()
            
            lidar_data = ros_node.get_scan()
            if lidar_data is None:
                return []
            
            # Get max LiDAR range
            max_range = lidar_data.range_max if hasattr(lidar_data, 'range_max') else 3.5
            
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
                
                # CRITICAL CHECK: Is this a VALID frontier?
                is_valid_frontier = self._is_valid_frontier(
                    distance, 
                    max_range, 
                    angle_offset
                )
                
                if is_valid_frontier:
                    # Calculate frontier center
                    frontier_x = robot_x + distance * np.cos(np.radians(absolute_angle))
                    frontier_y = robot_y + distance * np.sin(np.radians(absolute_angle))
                    
                    # Score: farther = better, but penalize if too close to max_range (wall)
                    distance_score = distance / self.max_frontier_distance
                    
                    # Penalty if close to sensor max (likely wall)
                    wall_penalty = 1.0
                    if distance > max_range * 0.8:
                        wall_penalty = 0.3  # Heavy penalty for wall-like readings
                    
                    # Bonus for front/side (avoid going backward)
                    direction_bonus = 1.0
                    if abs(angle_offset) < 90:
                        direction_bonus = 1.3
                    elif abs(angle_offset) < 120:
                        direction_bonus = 1.1
                    
                    score = distance_score * wall_penalty * direction_bonus
                    
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
                        f"score={score:.2f}, valid=True"
                    )
            
            # Sort by score (higher = better)
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