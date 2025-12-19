"""
ROS2 Node Module - Persistent Daemon Subprocess
Uses system Python 3.10 with rclpy via long-running subprocess.
"""

import os
import subprocess
import json
import logging
import threading
import time
from typing import Optional, Dict, Any
from pathlib import Path

logger = logging.getLogger(__name__)


class ROS2Bridge:
    """
    Bridge between Python 3.11+ (NAT) and Python 3.10 (ROS2).
    Uses persistent subprocess daemon for real-time sensor data AND commands.
    """
    
    # Constants
    SYSTEM_PYTHON = '/usr/bin/python3'
    CMD_QUEUE_FILE = '/tmp/ros2_cmd_queue.txt'
    
    def __init__(self):
        """Initialize bridge."""
        self._lidar_cache = None
        self._odom_cache = None
        self._nav2_state_cache = 'idle'
        self._slam_map_cache = None

        self._slam_save_result = None
        self._slam_save_event = threading.Event()
        
        self._daemon_process = None
        self._monitor_thread = None
        self._running = False
        self._lock = threading.Lock()
        
        # Start daemon
        self._start_monitor()
        
        logger.info("✅ ROS2Bridge initialized (daemon mode)")
    
    # =========================================================================
    # Public API - Nav2
    # =========================================================================
    
    def send_nav2_goal(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Send Nav2 goal via daemon."""
        try:
            cmd = {'type': 'nav2_goal', 'x': x, 'y': y, 'theta': theta}
            self._send_daemon_command(cmd)
            return True
        except Exception as e:
            logger.error(f"Nav2 goal send failed: {e}")
            return False
    
    def cancel_nav2_goal(self) -> bool:
        """Cancel Nav2 goal."""
        try:
            cmd = {'type': 'nav2_cancel'}
            self._send_daemon_command(cmd)
            return True
        except Exception as e:
            logger.error(f"Nav2 cancel failed: {e}")
            return False
    
    def get_nav2_state(self) -> Optional[str]:
        """Get Nav2 navigation state."""
        with self._lock:
            return self._nav2_state_cache

    def save_slam_map(self, map_path: str, timeout: float = 10.0) -> bool:
        """
        Save SLAM map via daemon (blocking call).
        
        Args:
            map_path: Path to save map (without extension)
            timeout: Max wait time for response in seconds
            
        Returns:
            bool: True if save successful
        """
        try:
            # Reset result and event
            with self._lock:
                self._slam_save_result = None
                self._slam_save_event.clear()
            
            # Send command to daemon
            cmd = {
                'type': 'slam_save_map',
                'map_path': map_path
            }
            
            logger.info(f"[BRIDGE] Requesting SLAM map save: {map_path}")
            self._send_daemon_command(cmd)
            
            # Wait for response (blocking with timeout)
            response_received = self._slam_save_event.wait(timeout=timeout)
            
            if not response_received:
                logger.error(f"[BRIDGE] ❌ SLAM save timeout after {timeout}s")
                return False
            
            # Check result
            with self._lock:
                success = self._slam_save_result
            
            if success:
                logger.info(f"[BRIDGE] ✅ SLAM map saved: {map_path}.yaml")
            else:
                logger.error(f"[BRIDGE] ❌ SLAM map save failed")
            
            return success
            
        except Exception as e:
            logger.error(f"[BRIDGE] SLAM save request error: {e}")
            return False
    
    # =========================================================================
    # Public API - Data
    # =========================================================================
    
    def get_name(self):
        """Get node name (compatibility)."""
        return "ros2_bridge_daemon"
    
    def get_scan(self) -> Optional[Any]:
        """Get cached LIDAR data."""
        with self._lock:
            return self._lidar_cache
    
    def get_odom(self) -> Optional[Any]:
        """Get cached odometry data."""
        with self._lock:
            return self._odom_cache
    
    def get_robot_pose(self) -> Optional[Dict]:
        """Get robot pose from odometry."""
        with self._lock:
            if not self._odom_cache:
                return None
            try:
                return {
                    'x': self._odom_cache['position']['x'],
                    'y': self._odom_cache['position']['y'],
                    'theta': self._odom_cache['orientation']['yaw']
                }
            except (KeyError, TypeError):
                return None
    
    def get_slam_map(self) -> Optional[Dict]:
        """Get SLAM map data."""
        with self._lock:
            return self._slam_map_cache
    
    # =========================================================================
    # Public API - Commands
    # =========================================================================
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command via file queue (non-blocking)."""
        try:
            with open(self.CMD_QUEUE_FILE, 'w') as f:
                f.write(f"{linear},{angular}\n")
        except Exception as e:
            logger.debug(f"Queue write error: {e}")
    
    def publish_stop(self):
        """Publish stop command."""
        self.publish_velocity(0.0, 0.0)
    
    # =========================================================================
    # Daemon Management
    # =========================================================================
    
    def _start_monitor(self):
        """Start persistent daemon subprocess."""
        self._running = True
        
        # Create empty queue file
        self._create_cmd_queue_file()
        
        # Get daemon script path
        script_path = self._get_daemon_script_path()
        
        # Create daemon environment
        env = self._create_daemon_environment()
        
        # Start daemon process
        try:
            self._daemon_process = subprocess.Popen(
                [self.SYSTEM_PYTHON, script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env
            )
            
            # Start stderr logger
            self._start_stderr_logger()
            
            # Start stdout reader
            self._monitor_thread = threading.Thread(
                target=self._read_daemon,
                daemon=True
            )
            self._monitor_thread.start()
            
            logger.info("✅ ROS2 daemon subprocess started")
            
        except Exception as e:
            logger.error(f"❌ Failed to start daemon: {e}")
            raise
    
    def _read_daemon(self):
        """Read sensor data + Nav2 state from daemon stdout."""
        while self._running:
            try:
                line = self._daemon_process.stdout.readline()
                if not line:
                    break
                
                msg = self._parse_daemon_message(line)
                if msg:
                    self._process_daemon_message(msg)
            
            except json.JSONDecodeError:
                continue
            except Exception as e:
                self._handle_daemon_error(e)
                break
        
        logger.warning("Daemon reader stopped")
    
    # =========================================================================
    # Helper Methods
    # =========================================================================
    
    def _get_daemon_script_path(self) -> str:
        """
        Get path to daemon script.
        
        Returns:
            str: Absolute path to ros2_daemon_script.py
        """
        # Try multiple locations
        possible_paths = [
            Path(__file__).parent / "ros2_daemon_script.py",
            Path("/workspace/mounted_code/src/multi_function_agent/_robot_vision_controller/core/ros2_node/ros2_daemon_script.py"),
            Path.home() / "nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/_robot_vision_controller/core/ros2_node/ros2_daemon_script.py"
        ]
        
        for path in possible_paths:
            if path.exists():
                logger.info(f"Using daemon script: {path}")
                return str(path)
        
        raise FileNotFoundError("Could not find ros2_daemon_script.py")
    
    def _create_daemon_environment(self) -> Dict[str, str]:
        """
        Create environment for daemon subprocess.
        
        Returns:
            dict: Environment variables with ROS2 setup
        """
        env = os.environ.copy()
        
        # Source ROS2 and extract env vars
        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && env'],
            capture_output=True, 
            text=True
        )
        
        for line in result.stdout.split('\n'):
            if '=' in line:
                parts = line.split('=', 1)
                if len(parts) == 2:
                    env[parts[0]] = parts[1]
        
        # Set RMW implementation
        env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        
        return env
    
    def _create_cmd_queue_file(self):
        """Create empty command queue file."""
        try:
            open(self.CMD_QUEUE_FILE, 'w').close()
        except Exception as e:
            logger.warning(f"Failed to create queue file: {e}")
    
    def _send_daemon_command(self, cmd: Dict):
        """Send command to daemon via stdin."""
        if self._daemon_process:
            self._daemon_process.stdin.write(json.dumps(cmd) + '\n')
            self._daemon_process.stdin.flush()
    
    def _parse_daemon_message(self, line: str) -> Optional[Dict]:
        """
        Parse JSON message from daemon.
        
        Args:
            line: Raw line from stdout
            
        Returns:
            dict: Parsed message or None
        """
        try:
            return json.loads(line.strip())
        except json.JSONDecodeError:
            return None
    
    def _process_daemon_message(self, msg: Dict):
        """Process message from daemon."""
        msg_type = msg.get('type')
        
        with self._lock:
            if msg_type == 'lidar':
                self._lidar_cache = self._create_lidar_data(msg['data'])
            
            elif msg_type == 'odom':
                self._odom_cache = msg['data']
            
            elif msg_type == 'nav2_state':
                self._nav2_state_cache = msg['state']
            
            elif msg_type == 'slam_map':
                self._slam_map_cache = msg['data']
            
            elif msg_type == 'slam_save_response':
                # ✅ HANDLER MỚI cho SLAM save response
                self._slam_save_result = msg.get('success', False)
                self._slam_save_event.set()  # Signal waiting thread
                
                logger.info(
                    f"[BRIDGE] SLAM save response received: "
                    f"success={self._slam_save_result}"
                )
    
    def _create_lidar_data(self, data: Dict):
        """
        Create LaserScan-like object from data.
        
        Args:
            data: LiDAR data dict
            
        Returns:
            LaserScan-like object
        """
        class LaserScanData:
            def __init__(self, d):
                self.ranges = d['ranges']
                self.angle_min = d['angle_min']
                self.angle_max = d['angle_max']
                self.angle_increment = d['angle_increment']
                self.range_min = d['range_min']
                self.range_max = d['range_max']
        
        return LaserScanData(data)
    
    def _handle_daemon_error(self, error: Exception):
        """
        Handle daemon communication error.
        
        Args:
            error: Exception that occurred
        """
        logger.error(f"Daemon communication error: {error}")
        # Could add reconnection logic here
    
    def _start_stderr_logger(self):
        """Start thread to log daemon stderr."""
        def log_stderr():
            for line in self._daemon_process.stderr:
                logger.error(f"[DAEMON STDERR] {line.strip()}")
        
        stderr_thread = threading.Thread(target=log_stderr, daemon=True)
        stderr_thread.start()
    
    # =========================================================================
    # Shutdown
    # =========================================================================
    
    def shutdown(self):
        """Shutdown bridge and daemon."""
        self._running = False
        
        # Send stop command
        self.publish_stop()
        time.sleep(0.1)
        
        # Terminate daemon
        if self._daemon_process:
            self._daemon_process.terminate()
            try:
                self._daemon_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._daemon_process.kill()
        
        # Wait for monitor thread
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        
        # Cleanup queue file
        try:
            os.remove(self.CMD_QUEUE_FILE)
        except:
            pass
        
        logger.info("✅ ROS2Bridge shutdown")


# =============================================================================
# Singleton
# =============================================================================

_bridge_instance = None
_bridge_lock = threading.Lock()


def get_ros2_node():
    """Get or create ROS2 bridge singleton."""
    global _bridge_instance
    with _bridge_lock:
        if _bridge_instance is None:
            _bridge_instance = ROS2Bridge()
    return _bridge_instance


def shutdown_ros2():
    """Shutdown ROS2 bridge."""
    global _bridge_instance
    with _bridge_lock:
        if _bridge_instance:
            _bridge_instance.shutdown()
            _bridge_instance = None