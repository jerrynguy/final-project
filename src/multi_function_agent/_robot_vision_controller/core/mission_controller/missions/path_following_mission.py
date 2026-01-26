"""
Path Following Mission Module
Execute precise parametric path using Nav2.
"""

import time
import logging
import numpy as np
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import (
    BaseMission, MissionConfig
)

logger = logging.getLogger(__name__)

# Import path generator from same directory
try:
    from .parametric_path_generator import ParametricPathGenerator, Waypoint
except ImportError:
    # Fallback for direct import
    from multi_function_agent._robot_vision_controller.core.mission_controller.missions.parametric_path_generator import ParametricPathGenerator, Waypoint


class PathFollowingMission(BaseMission):
    """
    Mission: Follow parametric geometric path using Nav2.
    
    Features:
    - Eclipse, spiral, figure-8, circle paths
    - Precise waypoint tracking
    - Progress monitoring
    - Path completion detection
    """
    
    # Constants
    WAYPOINT_REACHED_THRESHOLD = 0.15  # Meters
    NAV2_GOAL_TIMEOUT = 30.0  # Seconds
    DEFAULT_RESOLUTION = 100  # Waypoints per curve
    
    def _initialize_state(self) -> Dict:
        """Initialize path following state."""
        # Extract parameters
        path_type = self.config.parameters.get('path_type', 'circle')
        
        # Generate path
        self.path_generator = ParametricPathGenerator(
            resolution=self.config.parameters.get('resolution', self.DEFAULT_RESOLUTION)
        )
        
        self.waypoints = self._generate_path(path_type)
        
        if not self.waypoints:
            raise ValueError(f"Failed to generate path: {path_type}")
        
        logger.info(
            f"[PATH MISSION] Generated {path_type} with "
            f"{len(self.waypoints)} waypoints"
        )
        
        return {
            'path_type': path_type,
            'total_waypoints': len(self.waypoints),
            'current_waypoint_index': 0,
            'waypoints_reached': 0,
            'nav2_goal_sent': False,
            'nav2_goal_time': None,
            'path_started': False,
            'start_position_set': False
        }
    
    def _generate_path(self, path_type: str) -> List[Waypoint]:
        """
        Generate waypoints based on path type.
        
        Returns:
            List of waypoints or empty list if invalid
        """
        params = self.config.parameters
        
        # Center position (will be set to robot's start position)
        center = (
            params.get('center_x', 0.0),
            params.get('center_y', 0.0)
        )
        
        try:
            if path_type == 'eclipse':
                waypoints = self.path_generator.generate_eclipse(
                    center=center,
                    semi_major=params.get('semi_major', 2.0),
                    semi_minor=params.get('semi_minor', 1.0),
                    rotation=params.get('rotation', 0.0)
                )
            
            elif path_type == 'spiral':
                waypoints = self.path_generator.generate_spiral(
                    center=center,
                    a=params.get('initial_radius', 0.5),
                    b=params.get('growth_rate', 0.1),
                    turns=params.get('turns', 3)
                )
            
            elif path_type == 'figure8':
                waypoints = self.path_generator.generate_figure8(
                    center=center,
                    width=params.get('width', 2.0),
                    height=params.get('height', 1.5)
                )
            
            elif path_type == 'circle':
                waypoints = self.path_generator.generate_circle(
                    center=center,
                    radius=params.get('radius', 1.5)
                )
            
            else:
                logger.error(f"[PATH MISSION] Unknown path type: {path_type}")
                return []
            
            # Validate
            if self.path_generator.validate_path(waypoints):
                return waypoints
            else:
                logger.error("[PATH MISSION] Path validation failed")
                return []
        
        except Exception as e:
            logger.error(f"[PATH MISSION] Path generation failed: {e}")
            return []
    
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Update path following progress."""
        
        # Set start position on first update
        if robot_pos and not self.state['start_position_set']:
            self._offset_path_to_start(robot_pos)
            self.state['start_position_set'] = True
            self.state['path_started'] = True
            logger.info(
                f"[PATH MISSION] Path centered at robot position: "
                f"({robot_pos['x']:.2f}, {robot_pos['y']:.2f})"
            )
        
        # Check waypoint progress
        if robot_pos and self.state['path_started']:
            self._check_waypoint_reached(robot_pos)
        
        # Update progress
        if self.state['total_waypoints'] > 0:
            self.state['progress'] = (
                self.state['waypoints_reached'] / 
                self.state['total_waypoints']
            )
        
        return self.state
    
    def _offset_path_to_start(self, robot_pos: Dict):
        """
        Offset entire path so it starts at robot's current position.
        
        Args:
            robot_pos: Current robot position {'x', 'y', 'theta'}
        """
        if not self.waypoints:
            return
        
        # Calculate offset
        first_wp = self.waypoints[0]
        offset_x = robot_pos['x'] - first_wp.x
        offset_y = robot_pos['y'] - first_wp.y
        
        # Apply offset to all waypoints
        for wp in self.waypoints:
            wp.x += offset_x
            wp.y += offset_y
        
        logger.info(
            f"[PATH MISSION] Path offset: ({offset_x:.2f}, {offset_y:.2f})"
        )
    
    def _check_waypoint_reached(self, robot_pos: Dict):
        """
        Check if current waypoint reached.
        
        Args:
            robot_pos: Current robot position
        """
        current_idx = self.state['current_waypoint_index']
        
        if current_idx >= len(self.waypoints):
            return  # Path complete
        
        current_wp = self.waypoints[current_idx]
        
        # Calculate distance to waypoint
        dx = robot_pos['x'] - current_wp.x
        dy = robot_pos['y'] - current_wp.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Check if reached
        if distance < self.WAYPOINT_REACHED_THRESHOLD:
            self.state['waypoints_reached'] += 1
            self.state['current_waypoint_index'] += 1
            self.state['nav2_goal_sent'] = False  # Ready for next waypoint
            
            logger.info(
                f"[PATH MISSION] Waypoint {current_idx + 1}/"
                f"{len(self.waypoints)} reached"
            )
    
    def _check_completion(self) -> bool:
        """Check if entire path completed."""
        return self.state['current_waypoint_index'] >= len(self.waypoints)
    
    def _get_directive(self) -> str:
        """
        Get navigation directive.
        
        Returns:
            'path_follow' to trigger Nav2 goal sending
        """
        return 'path_follow'
    
    def get_next_waypoint(self) -> Optional[Dict]:
        """
        Get next waypoint for Nav2.
        
        Returns:
            Dict with {'x', 'y', 'theta'} or None if complete
        """
        current_idx = self.state['current_waypoint_index']
        
        if current_idx >= len(self.waypoints):
            return None
        
        wp = self.waypoints[current_idx]
        
        return {
            'x': wp.x,
            'y': wp.y,
            'theta': wp.theta,
            'index': wp.index
        }
    
    def should_send_nav2_goal(self) -> bool:
        """
        Check if should send new Nav2 goal.
        
        Returns:
            True if ready to send next goal
        """
        # Don't send if already sent and waiting
        if self.state['nav2_goal_sent']:
            # Check timeout
            if self.state['nav2_goal_time']:
                elapsed = time.time() - self.state['nav2_goal_time']
                if elapsed > self.NAV2_GOAL_TIMEOUT:
                    logger.warning(
                        f"[PATH MISSION] Nav2 goal timeout after {elapsed:.0f}s, "
                        f"skipping waypoint {self.state['current_waypoint_index']}"
                    )
                    # Skip this waypoint
                    self.state['current_waypoint_index'] += 1
                    self.state['nav2_goal_sent'] = False
                    return True
            return False
        
        # Send if path started and not complete
        return (
            self.state['path_started'] and 
            self.state['current_waypoint_index'] < len(self.waypoints)
        )
    
    def mark_nav2_goal_sent(self):
        """Mark that Nav2 goal has been sent."""
        self.state['nav2_goal_sent'] = True
        self.state['nav2_goal_time'] = time.time()
    
    def get_path_visualization_data(self) -> Dict:
        """
        Export path data for visualization.
        
        Returns:
            Dict with waypoints, current position, progress
        """
        return {
            'waypoints': self.path_generator.export_to_dict(self.waypoints),
            'current_index': self.state['current_waypoint_index'],
            'total_waypoints': len(self.waypoints),
            'progress': self.state['progress'],
            'path_type': self.state['path_type']
        }