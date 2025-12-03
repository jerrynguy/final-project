"""
Stuck Detector Module
Detects when robot is stuck in a loop using position tracking.
"""

import logging
import numpy as np
from typing import Dict, List, Optional
from collections import deque

logger = logging.getLogger(__name__)


class StuckDetector:
    """
    Lightweight stuck detection using position history.
    Detects when robot fails to make progress over multiple iterations.
    """
    
    def __init__(self, window_size: int = 2, displacement_threshold: float = 0.08):
        """
        Initialize stuck detector.
        
        Args:
            window_size: Number of iterations to check (default: 2)
            displacement_threshold: Minimum displacement in meters to consider "moving" (default: 8cm)
        """
        self.window_size = window_size
        self.displacement_threshold = displacement_threshold
        
        self.position_history = deque(maxlen=window_size)
        self.action_history = deque(maxlen=window_size)
        
        self.stuck_count = 0
        self.total_checks = 0
    
    def update(self, robot_pos: Optional[Dict], action: str) -> bool:
        """
        Update detector with new position and action.
        
        Args:
            robot_pos: Robot position {x, y, theta}
            action: Current navigation action
            
        Returns:
            bool: True if robot is stuck
        """
        if robot_pos is None:
            return False
        
        self.total_checks += 1
        
        # Add to history
        self.position_history.append(robot_pos)
        self.action_history.append(action)
        
        # Need full window to detect
        if len(self.position_history) < self.window_size:
            return False
        
        # Check if stuck
        is_stuck = self._check_stuck()
        
        if is_stuck:
            self.stuck_count += 1
            logger.warning(
                f"[STUCK DETECTED] No movement in last {self.window_size} iterations "
                f"(Total stuck events: {self.stuck_count})"
            )
        
        return is_stuck
    
    def _check_stuck(self) -> bool:
        """
        Check if robot has made minimal progress.
        
        Returns:
            bool: True if stuck (displacement < threshold)
        """
        positions = list(self.position_history)
        
        # Calculate total displacement
        total_displacement = 0.0
        for i in range(1, len(positions)):
            dx = positions[i]['x'] - positions[i-1]['x']
            dy = positions[i]['y'] - positions[i-1]['y']
            displacement = np.sqrt(dx**2 + dy**2)
            total_displacement += displacement
        
        # Average displacement per iteration
        avg_displacement = total_displacement / (len(positions) - 1)
        
        # Check if stuck
        stuck = avg_displacement < self.displacement_threshold
        
        if stuck:
            logger.debug(
                f"[STUCK CHECK] Avg displacement: {avg_displacement:.4f}m "
                f"(threshold: {self.displacement_threshold}m)"
            )
            logger.debug(f"[STUCK CHECK] Recent actions: {list(self.action_history)}")
        
        return stuck
    
    def reset(self):
        """Reset detector state after successful escape."""
        self.position_history.clear()
        self.action_history.clear()
        logger.info("[STUCK DETECTOR] Reset after escape attempt")
    
    def get_stats(self) -> Dict:
        """Get detector statistics."""
        return {
            'total_checks': self.total_checks,
            'stuck_events': self.stuck_count,
            'stuck_rate': self.stuck_count / max(1, self.total_checks)
        }