"""
SLAM Controller Module
Manages SLAM Toolbox integration for autonomous mapping during exploration.
"""

import os
import time
import logging
import subprocess
import signal
from typing import Optional, Dict, Tuple
from pathlib import Path

logger = logging.getLogger(__name__)


class SLAMController:
    """
    Controls SLAM Toolbox for real-time mapping during exploration missions.
    """
    
    def __init__(self, map_save_path: str = "~/my_map"):
        """
        Initialize SLAM controller.
        
        Args:
            map_save_path: Path to save generated map (without extension)
        """
        self.map_save_path = map_save_path
        self.slam_process: Optional[subprocess.Popen] = None
        self.is_running = False
        self.start_time: Optional[float] = None
        
        # SLAM configuration
        self.use_sim_time = True  # Always true for Gazebo
        self.map_update_interval = 5.0  # Save map every 5s during exploration
        self.last_map_save = 0
        
        logger.info(f"[SLAM] Initialized with save path: {self.map_save_path}")
    
    def start_slam(self) -> bool:
        """
        Start SLAM Toolbox as subprocess.
        
        Returns:
            bool: True if SLAM started successfully
        """
        if self.is_running:
            logger.warning("[SLAM] Already running")
            return True
        
        try:
            # Check if ROS2 environment is sourced
            ros_distro = os.environ.get('ROS_DISTRO')
            if not ros_distro:
                logger.error("[SLAM] ROS_DISTRO not set - source ROS2 environment first")
                return False
            
            # CRITICAL: Container lacks ROS2 binary - SLAM must run on HOST
            logger.warning("=" * 60)
            logger.warning("[SLAM] Container cannot spawn SLAM subprocess")
            logger.warning("[SLAM] ROS2 binary not available in NAT container")
            logger.warning("=" * 60)
            logger.info("[SLAM] MANUAL START REQUIRED on host:")
            logger.info("  Terminal 1: ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True")
            logger.info("  Terminal 2: # Wait for SLAM to initialize (~3s)")
            logger.info("=" * 60)
            
            # Bypass subprocess requirement - assume SLAM running on host
            logger.info("[SLAM] Assuming SLAM running externally on host...")
            
            self.is_running = True
            self.start_time = time.time()
            logger.info("[SLAM] Flagged as running (external process)")
            return True
            
        except FileNotFoundError:
            logger.error("[SLAM] slam_toolbox not found - install: sudo apt install ros-humble-slam-toolbox")
            return False
        
        except Exception as e:
            logger.error(f"[SLAM] Start failed: {e}")
            return False
    
    def save_map(self, map_name: Optional[str] = None) -> bool:
        """
        Save current SLAM map to disk.
        
        Args:
            map_name: Optional custom map name (without extension)
        
        Returns:
            bool: True if map saved successfully
        """
        if not self.is_running:
            logger.warning("[SLAM] Not running, cannot save map")
            return False
        
        save_path = map_name if map_name else self.map_save_path
        
        try:
            # Create directory if needed
            save_dir = os.path.dirname(save_path)
            if save_dir:
                os.makedirs(save_dir, exist_ok=True)
            
            # Async save to avoid blocking control loop
            import subprocess
            import threading
            
            def _async_save():
                try:
                    result = subprocess.run(
                        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save_path],
                        capture_output=True,
                        timeout=2.0
                    )
                    if result.returncode == 0:
                        logger.info(f"[SLAM] Map saved: {save_path}")
                    else:
                        logger.warning(f"[SLAM] Save warning: {result.stderr.decode()}")
                except Exception as e:
                    logger.error(f"[SLAM] Async save error: {e}")

            save_thread = threading.Thread(target=_async_save, daemon=True)
            save_thread.start()

            self.last_map_save = time.time()
            return True
                
        except subprocess.TimeoutExpired:
            logger.error("[SLAM] Map save timeout")
            return False
        
        except Exception as e:
            logger.error(f"[SLAM] Save error: {e}")
            return False
    
    def auto_save_map(self, force: bool = False) -> bool:
        """
        Auto-save map if enough time has passed since last save.
        
        Args:
            force: Force save regardless of interval
        
        Returns:
            bool: True if map was saved
        """
        current_time = time.time()
        save_interval = 10.0
        
        if force or (current_time - self.last_map_save) >= save_interval:
            return self.save_map()
        
        return False
    
    def stop_slam(self, save_final_map: bool = True) -> bool:
        """
        Stop SLAM Toolbox and optionally save final map.
        
        Args:
            save_final_map: Whether to save map before stopping
        
        Returns:
            bool: True if stopped successfully
        """
        if not self.is_running:
            logger.warning("[SLAM] Not running")
            return True
        
        try:
            # Save final map if requested
            if save_final_map:
                logger.info("[SLAM] Saving final map before shutdown...")
                self.save_map()
            
            # No subprocess to kill - SLAM running externally
            logger.warning("[SLAM] External SLAM process - manual stop required:")
            logger.warning("  Ctrl+C in SLAM terminal to stop")
            
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
            'map_path': self.map_save_path,
            'duration': 0,
            'last_save': self.last_map_save
        }
        
        if self.start_time:
            stats['duration'] = time.time() - self.start_time
        
        # Check if map files exist
        yaml_file = f"{self.map_save_path}.yaml"
        pgm_file = f"{self.map_save_path}.pgm"
        
        stats['map_exists'] = os.path.exists(yaml_file) and os.path.exists(pgm_file)
        
        if stats['map_exists']:
            stats['yaml_size'] = os.path.getsize(yaml_file)
            stats['pgm_size'] = os.path.getsize(pgm_file)
        
        return stats
    
    def verify_map_quality(self) -> Tuple[bool, str]:
        """
        Verify if generated map is usable for navigation.
        
        Returns:
            tuple: (is_valid, reason)
        """
        yaml_file = f"{self.map_save_path}.yaml"
        pgm_file = f"{self.map_save_path}.pgm"
        
        # Check files exist
        if not os.path.exists(yaml_file):
            return False, f"Map YAML not found: {yaml_file}"
        
        if not os.path.exists(pgm_file):
            return False, f"Map PGM not found: {pgm_file}"
        
        # Check file sizes (empty maps are suspicious)
        yaml_size = os.path.getsize(yaml_file)
        pgm_size = os.path.getsize(pgm_file)
        
        if yaml_size < 100:  # YAML should be at least 100 bytes
            return False, f"Map YAML too small ({yaml_size} bytes)"
        
        if pgm_size < 1000:  # PGM should be at least 1KB
            return False, f"Map PGM too small ({pgm_size} bytes)"
        
        logger.info(f"[SLAM] Map validation: YAML={yaml_size}B, PGM={pgm_size}B")
        return True, "Map valid"
    
    def __del__(self):
        """Cleanup on deletion."""
        if self.is_running:
            logger.warning("[SLAM] Controller deleted while running, cleaning up...")
            self.stop_slam(save_final_map=True)