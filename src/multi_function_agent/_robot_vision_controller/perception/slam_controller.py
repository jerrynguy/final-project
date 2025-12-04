"""
SLAM Controller Module
Manages SLAM Toolbox integration for autonomous mapping during exploration.
"""

import os
import time
import logging
from typing import Optional, Dict

logger = logging.getLogger(__name__)


class SLAMController:
    """
    Controls SLAM Toolbox for real-time mapping during exploration missions.
    Simplified version - external SLAM process only, manual save.
    """
    
    def __init__(self, map_save_path: str = "~/my_map"):
        """
        Initialize SLAM controller.
        
        Args:
            map_save_path: Suggested path for manual map save
        """
        # Expand path
        if map_save_path.startswith('~'):
            map_save_path = os.path.expanduser(map_save_path)
        
        self.map_save_path = map_save_path
        self.is_running = False
        self.start_time: Optional[float] = None
        
        logger.info(f"[SLAM] Initialized - Manual save to: {self.map_save_path}")
    
    def start_slam(self) -> bool:
        """
        Flag SLAM as running (assumes external SLAM process on host).
        
        Returns:
            bool: True (assumes SLAM running externally)
        """
        if self.is_running:
            logger.warning("[SLAM] Already flagged as running")
            return True
        
        logger.warning("=" * 60)
        logger.warning("[SLAM] Container cannot spawn SLAM subprocess")
        logger.warning("[SLAM] MANUAL START REQUIRED on HOST:")
        logger.warning("  Terminal 1: ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True")
        logger.warning("  Wait ~3 seconds for SLAM to initialize")
        logger.warning("=" * 60)
        
        # Flag as running (assumes user started SLAM on host)
        self.is_running = True
        self.start_time = time.time()
        logger.info("[SLAM] Flagged as running (external process)")
        
        return True
    
    def stop_slam(self) -> bool:
        """
        Stop SLAM tracking and show manual save instructions.
        
        Returns:
            bool: True if stopped successfully
        """
        if not self.is_running:
            logger.warning("[SLAM] Not running")
            return True
        
        try:
            # Show manual save instructions
            logger.warning("=" * 60)
            logger.warning("[SLAM] Exploration completed")
            logger.warning("[SLAM] To save map, run on HOST:")
            logger.warning(f"  ros2 run nav2_map_server map_saver_cli -f {self.map_save_path}")
            logger.warning("=" * 60)
            logger.warning("[SLAM] To stop SLAM Toolbox:")
            logger.warning("  Ctrl+C in SLAM terminal")
            logger.warning("=" * 60)
            
            self.is_running = False
            
            # Log mapping duration
            if self.start_time:
                duration = time.time() - self.start_time
                logger.info(f"[SLAM] Total mapping time: {duration:.1f}s")
            
            return True
            
        except Exception as e:
            logger.error(f"[SLAM] Stop error: {e}")
            return False
    
    def get_mapping_stats(self) -> Dict:
        """
        Get current SLAM mapping statistics.
        
        Returns:
            dict: Mapping statistics
        """
        stats = {
            'is_running': self.is_running,
            'suggested_save_path': self.map_save_path,
            'duration': 0
        }
        
        if self.start_time:
            stats['duration'] = time.time() - self.start_time
        
        return stats
    
    def __del__(self):
        """Cleanup on deletion."""
        if self.is_running:
            logger.warning("[SLAM] Controller deleted, showing save instructions...")
            self.stop_slam()