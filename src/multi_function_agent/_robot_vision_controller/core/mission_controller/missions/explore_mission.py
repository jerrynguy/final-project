"""
Explore Mission Module
SLAM-based exploration mission implementation.
"""

import logging
from typing import Dict, List, Optional

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class ExploreMission(BaseMission):
    """
    Mission: Explore area with SLAM mapping.
    """
    
    # Constants
    GRID_SIZE = 1.0  # Meters - grid cell size for coverage tracking
    DEFAULT_DURATION = 60.0  # Seconds
    COVERAGE_LOG_INTERVAL = 10  # Log every N new areas
    
    def _initialize_state(self) -> Dict:
        """Initialize explore-specific state."""
        duration = self.config.parameters.get('duration', self.DEFAULT_DURATION)
        
        # Handle None or inf duration
        if duration is None or duration == float('inf'):
            duration = self.DEFAULT_DURATION

        MAX_EXPLORE_DURATION = 600.0  # 10 minutes absolute max
        if duration > MAX_EXPLORE_DURATION:
            logger.warning(
                f"[EXPLORE] Duration {duration}s capped to {MAX_EXPLORE_DURATION}s"
            )
            duration = MAX_EXPLORE_DURATION
        
        return {
            'coverage': self.config.parameters.get('coverage', 'full'),
            'duration': duration,
            'areas_visited': set(),
            'slam_enabled': True,
            'map_saved': False,
            'mapping_completed': False
        }
    
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Update exploration state."""
        elapsed = self.get_elapsed_time()
        duration = self.state['duration']
        
        # Update progress
        if duration is None or duration == float('inf'):
            self.state['progress'] = 0.0
        else:
            self.state['progress'] = min(1.0, elapsed / duration)
        
        # Track coverage
        if robot_pos:
            self._update_coverage(robot_pos)
        
        return self.state
    
    def _update_coverage(self, robot_pos: Dict) -> None:
        """Update area coverage tracking."""
        # Convert position to grid cell
        grid_x = int(robot_pos['x'] / self.GRID_SIZE)
        grid_y = int(robot_pos['y'] / self.GRID_SIZE)
        grid_cell = (grid_x, grid_y)
        
        # Add new cell
        if grid_cell not in self.state['areas_visited']:
            self.state['areas_visited'].add(grid_cell)
            
            # Log coverage periodically
            area_count = len(self.state['areas_visited'])
            if area_count % self.COVERAGE_LOG_INTERVAL == 0:
                logger.info(f"[EXPLORE] Covered {area_count} areas")
    
    def _check_completion(self) -> bool:
        """Check if exploration duration completed OR stuck too long."""
        elapsed = self.get_elapsed_time()
        duration = self.state['duration']
        
        # Handle None or inf duration
        if duration is None or duration == float('inf'):
            return False
        
        # ADDED: Check if stuck detector indicates stuck
        if hasattr(self, 'stuck_detector'):
            stuck_stats = self.stuck_detector.get_stats()
            
            # If stuck for >30s, force completion
            if stuck_stats['stuck_duration'] > 30.0:
                logger.warning(
                    f"[EXPLORE] Force complete: Stuck {stuck_stats['stuck_duration']:.0f}s "
                    f"(elapsed: {elapsed:.0f}s/{duration:.0f}s)"
                )
                self.state['mapping_complete'] = True
                return True
        
        # Normal completion by time
        if elapsed >= duration:
            self.state['mapping_complete'] = True
            area_count = len(self.state['areas_visited'])
            logger.info(
                f"[EXPLORE] Completed: {area_count} areas in {elapsed:.0f}s"
            )
            return True
        
        return False
    
    def _get_directive(self) -> str:
        """Get exploration directive."""
        return 'explore_random'