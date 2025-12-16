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
    
    def __init__(self, map_save_path: str = "~/my_map"):
        """
        Initialize SLAM controller.
        
        Args:
            map_save_path: Path for map files (without extension)
        """
        # Expand path
        if map_save_path.startswith('~'):
            map_save_path = os.path.expanduser(map_save_path)
        
        self.map_save_path = map_save_path
        self.is_running = False
        self.start_time: Optional[float] = None
        
        # Auto-save tracking
        self.auto_save_interval = 5.0  # seconds
        self.last_save_time: Optional[float] = None
        self.save_count = 0
        
        logger.info(f"[SLAM] Initialized - Auto-save to: {self.map_save_path}")
    
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
    
    def _get_ros_env(self) -> dict:
        """
        Get ROS2 environment for subprocess calls.
        
        Returns:
            dict: Environment with ROS2 sourced
        """
        env = os.environ.copy()
        
        # Source ROS2 setup and extract environment
        try:
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && env'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            for line in result.stdout.split('\n'):
                if '=' in line:
                    parts = line.split('=', 1)
                    if len(parts) == 2:
                        env[parts[0]] = parts[1]
        except Exception as e:
            logger.warning(f"[SLAM] ROS2 env setup warning: {e}")
        
        # Ensure critical ROS2 variables
        if 'ROS_DOMAIN_ID' not in env:
            env['ROS_DOMAIN_ID'] = '0'
        if 'RMW_IMPLEMENTATION' not in env:
            env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        
        return env
    
    def _auto_save_map(self) -> bool:
        """
        Auto-save map using nav2_map_server map_saver_cli.
        
        Returns:
            bool: True if save successful
        """
        try:
            # Call map_saver_cli via subprocess
            result = subprocess.run(
                [
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', self.map_save_path,
                    '--ros-args', '-p', 'save_map_timeout:=5.0'
                ],
                capture_output=True,
                text=True,
                timeout=10.0,
                env=self._get_ros_env()
            )
            
            if result.returncode == 0:
                self.save_count += 1
                logger.info(
                    f"[SLAM AUTO-SAVE #{self.save_count}] "
                    f"Map saved: {self.map_save_path}.yaml"
                )
                return True
            else:
                logger.error(
                    f"[SLAM AUTO-SAVE] Failed (exit {result.returncode}): "
                    f"{result.stderr.strip()}"
                )
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("[SLAM AUTO-SAVE] Timeout after 10s")
            return False
        except FileNotFoundError:
            logger.error(
                "[SLAM AUTO-SAVE] map_saver_cli not found. "
                "Install: sudo apt install ros-humble-nav2-map-server"
            )
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