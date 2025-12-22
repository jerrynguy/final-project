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

    # Stuck bailout threshold
    STUCK_BAILOUT_THRESHOLD = 8.0  # Seconds - force complete if stuck too long
    
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
        """
        Multi-condition completion check (Q3: Option B).
        
        Completion criteria:
        1. Duration elapsed (MANDATORY - must always be checked)
        2. OR (Stuck >30s AND duration >50% complete)
        3. OR (Safety aborts >10 times AND duration >50% complete)
        
        Duration is ALWAYS required to complete.
        """
        elapsed = self.get_elapsed_time()
        duration = self.state['duration']
        
        # Handle None or inf duration
        if duration is None or duration == float('inf'):
            return False
        
        # Calculate progress percentage
        progress_pct = (elapsed / duration) * 100 if duration > 0 else 0
        
        # Duration timeout (MANDATORY - always wins)
        if elapsed >= duration:
            self.state['mapping_complete'] = True
            area_count = len(self.state['areas_visited'])
            logger.info(
                f"[EXPLORE] ✅ Completed: Duration timeout "
                f"({elapsed:.0f}s >= {duration:.0f}s), covered {area_count} areas"
            )
            return True
        
        # Stuck detection (only if >50% duration passed)
        if progress_pct >= 50.0:
            if hasattr(self, 'stuck_detector') and self.stuck_detector:
                stuck_stats = self.stuck_detector.get_stats()
                
                if stuck_stats['stuck_duration'] > self.STUCK_BAILOUT_THRESHOLD:
                    logger.error(
                        f"[EXPLORE BAILOUT] Stuck {stuck_stats['stuck_duration']:.0f}s "
                        f"(threshold: {self.STUCK_BAILOUT_THRESHOLD}s) "
                        f"→ EMERGENCY COMPLETE"
                    )
                    logger.error(
                        f"[EXPLORE BAILOUT] Total elapsed: {elapsed:.0f}s/{duration:.0f}s, "
                        f"areas covered: {len(self.state['areas_visited'])}"
                    )
                    self.state['mapping_complete'] = True
                    return True
                
            # Additional Nav2 rescue attempt flag
            # If Nav2 rescue was attempted but failed, force complete sooner
            if hasattr(self, '_nav2_rescue_failed') and self._nav2_rescue_failed:
                if elapsed > duration * 0.3:  # If >30% of time passed and Nav2 failed
                    logger.error(
                        f"[EXPLORE BAILOUT] Nav2 rescue failed and "
                        f"{elapsed:.0f}s/{duration:.0f}s elapsed → FORCE COMPLETE"
                    )
                    self.state['mapping_complete'] = True
                    return True
        
            # Safety abort spam (only if >50% duration passed)
            # Track abort count from stuck detector or separate counter
            if hasattr(self, 'stuck_detector') and self.stuck_detector:
                stuck_stats = self.stuck_detector.get_stats()
                abort_count = stuck_stats.get('total_stuck_count', 0)
                
                if abort_count > 10:
                    logger.warning(
                        f"[EXPLORE] ⚠️ Force complete: Too many aborts ({abort_count}) "
                        f"(progress: {progress_pct:.0f}%, elapsed: {elapsed:.0f}s/{duration:.0f}s)"
                    )
                    self.state['mapping_complete'] = True
                    return True
        
        return False
    
    def _get_directive(self) -> str:
        """Get exploration directive."""
        return 'explore_random'