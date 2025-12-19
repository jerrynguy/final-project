"""
SLAM Controller Module
Manages SLAM Toolbox integration for autonomous mapping during exploration.
"""

import os
import time
import logging
import subprocess
from typing import Optional, Dict

logger = logging.getLogger(__name__)


class SLAMController:
    """
    Controls SLAM Toolbox for real-time mapping during exploration missions.
    Auto-saves map every 5 seconds via ros2 map_saver_cli.
    """
    DEFAULT_MAP_PATHS = [
        "/workspace/mounted_code/maps",  # Persistent across restarts
        "/workspace/persistent_data/maps",  # Backup location
        "/root/maps",  # Container fallback
    ]

    def __init__(self, map_save_path: str = None):
            """Initialize with persistent path resolution."""
            
            # FIXED: Resolve to persistent location
            if map_save_path is None:
                map_save_path = self._resolve_persistent_path()
            else:
                # Ensure parent dir exists
                map_dir = os.path.dirname(map_save_path)
                os.makedirs(map_dir, exist_ok=True)
            
            self.map_save_path = map_save_path
            self.map_dir = os.path.dirname(map_save_path)
            self.map_name = os.path.basename(map_save_path)
            
            # State tracking
            self.is_running = False
            self.start_time: Optional[float] = None
            self.auto_save_interval = 5.0
            self.last_save_time: Optional[float] = None
            self.save_count = 0
            
            # ADDED: Save method tracking
            self.last_save_method = None
            self.save_failures = 0
            
            logger.info(
                f"[SLAM] Initialized - Map dir: {self.map_dir}, "
                f"Name: {self.map_name}"
            )

    def _resolve_persistent_path(self) -> str:
        """
        Resolve map path to persistent storage.
        Priority: mounted_code > persistent_data > root
        """
        for base_dir in self.DEFAULT_MAP_PATHS:
            try:
                os.makedirs(base_dir, exist_ok=True)
                
                # Test write permission
                test_file = os.path.join(base_dir, ".write_test")
                with open(test_file, 'w') as f:
                    f.write("test")
                os.remove(test_file)
                
                # Success - use this dir
                map_path = os.path.join(base_dir, "my_map")
                logger.info(f"[SLAM] Using persistent path: {map_path}")
                return map_path
                
            except (PermissionError, OSError) as e:
                logger.warning(f"[SLAM] Cannot use {base_dir}: {e}")
                continue
        
        # Fallback to /tmp (not persistent but works)
        fallback = "/tmp/my_map"
        logger.error(f"[SLAM] All persistent paths failed, using: {fallback}")
        return fallback

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
        logger.info("[SLAM] Auto-save enabled: every 5 seconds")
        
        return True
    
    def _auto_save_map(self) -> bool:
        """
        Auto-save map using ROS2 bridge.
        
        Returns:
            bool: True if save successful
        """
        try:
            # ✅ GỌI QUA ROS2 BRIDGE
            from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
            
            ros_node = get_ros2_node()
            
            logger.info(f"[SLAM] Requesting map save via ROS2 bridge...")
            success = ros_node.save_slam_map(
                self.map_save_path, 
                timeout=10.0
            )
            
            if success:
                self.save_count += 1
                logger.info(
                    f"[SLAM AUTO-SAVE #{self.save_count}] "
                    f"Map saved: {self.map_save_path}.yaml"
                )
                return True
            else:
                logger.error("[SLAM AUTO-SAVE] Failed via ROS2 bridge")
                return False
                
        except Exception as e:
            logger.error(f"[SLAM AUTO-SAVE] Error: {e}")
            return False
    
    def maybe_auto_save(self) -> None:
        """
        Maybe trigger auto-save based on interval.
        
        Should be called from main control loop every iteration.
        """
        if not self.is_running:
            return
        
        current_time = time.time()
        
        # First save or interval elapsed
        if (self.last_save_time is None or 
            current_time - self.last_save_time >= self.auto_save_interval):
            
            logger.debug("[SLAM] Auto-save interval reached, saving...")
            success = self._auto_save_map()
            
            if success:
                self.last_save_time = current_time
    
    def wait_for_map_save(self, timeout: float = 10.0) -> bool:
        """
        Wait for final map save and validate files exist.
        
        Args:
            timeout: Max wait time in seconds
            
        Returns:
            bool: True if map exists and valid
        """
        yaml_file = f"{self.map_save_path}.yaml"
        pgm_file = f"{self.map_save_path}.pgm"
        
        start_time = time.time()
        
        # Try one final save
        logger.info("[SLAM] Final map save...")
        self._auto_save_map()
        
        # Poll for files with validation
        while time.time() - start_time < timeout:
            if os.path.exists(yaml_file) and os.path.exists(pgm_file):
                # Validate file sizes (sanity check)
                yaml_size = os.path.getsize(yaml_file)
                pgm_size = os.path.getsize(pgm_file)
                
                # YAML should be ~200-500 bytes, PGM at least 1KB for any map
                if yaml_size > 100 and pgm_size > 1000:
                    logger.info(
                        f"[SLAM] ✅ Map validated: "
                        f"YAML={yaml_size}B, PGM={pgm_size}B"
                    )
                    return True
                else:
                    logger.warning(
                        f"[SLAM] Files too small (YAML={yaml_size}B, PGM={pgm_size}B), "
                        f"waiting..."
                    )
            
            time.sleep(0.5)
        
        logger.error(f"[SLAM] ❌ Map save timeout after {timeout}s")
        
        # Show what files exist for debugging
        if os.path.exists(yaml_file):
            logger.error(f"  YAML exists: {os.path.getsize(yaml_file)}B")
        else:
            logger.error(f"  YAML missing: {yaml_file}")
        
        if os.path.exists(pgm_file):
            logger.error(f"  PGM exists: {os.path.getsize(pgm_file)}B")
        else:
            logger.error(f"  PGM missing: {pgm_file}")
        
        return False
    
    def stop_slam(self) -> bool:
        """
        Stop SLAM tracking and ensure map saved.
        
        Returns:
            bool: True if map successfully saved
        """
        if not self.is_running:
            logger.warning("[SLAM] Not running")
            return True
        
        try:
            # Final save with validation
            logger.info("[SLAM] Stopping and saving final map...")
            map_saved = self.wait_for_map_save(timeout=10.0)
            
            if not map_saved:
                logger.error("[SLAM] ⚠️  Warning: Map may be incomplete or missing")
                logger.error("[SLAM] You may need to save manually:")
                logger.error(f"  ros2 run nav2_map_server map_saver_cli -f {self.map_save_path}")
            
            self.is_running = False
            
            # Log statistics
            if self.start_time:
                duration = time.time() - self.start_time
                logger.info("=" * 60)
                logger.info("[SLAM COMPLETE]")
                logger.info(f"  Duration: {duration:.1f}s")
                logger.info(f"  Auto-saves: {self.save_count}")
                logger.info(f"  Final map: {self.map_save_path}.yaml")
                logger.info("=" * 60)
            
            return map_saved
            
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
            'map_save_path': self.map_save_path,
            'duration': 0,
            'auto_saves': self.save_count,
            'map_exists': False
        }
        
        if self.start_time:
            stats['duration'] = time.time() - self.start_time
        
        # Check if map files exist
        yaml_file = f"{self.map_save_path}.yaml"
        pgm_file = f"{self.map_save_path}.pgm"
        stats['map_exists'] = os.path.exists(yaml_file) and os.path.exists(pgm_file)
        
        return stats
    
    def __del__(self):
        """Cleanup on deletion."""
        if self.is_running:
            logger.warning("[SLAM] Controller deleted while running, attempting cleanup...")
            self.stop_slam()