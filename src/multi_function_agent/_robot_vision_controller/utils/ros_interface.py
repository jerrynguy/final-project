"""
ROS Interface Module
Thread-safe ROS2 lifecycle management and spinning.
"""

import rclpy
import logging
import threading
from rclpy.executors import MultiThreadedExecutor

logger = logging.getLogger(__name__)


# =============================================================================
# ROS Manager
# =============================================================================

class ROSManager:
    """
    Manages ROS2 node spinning in background thread.
   """
    
    def __init__(self, node):
        """
        Initialize ROS manager.
        """
        self.node = node
        self.executor = None
        self.spin_thread = None
        self.is_spinning = False
    
    def start_spinning(self):
        """
        Start ROS2 node spinning in background thread.
        """
        if self.is_spinning:
            return
        
        try:
            # Create executor and add node
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Create and start background thread
            self.spin_thread = threading.Thread(
                target=self._spin_worker,
                daemon=True
            )
            
            self.is_spinning = True
            self.spin_thread.start()
            
            logger.info("ROS spinning started")
            
        except Exception as e:
            logger.error(f"Failed to start ROS spinning: {e}")
            self.is_spinning = False
    
    def stop_spinning(self):
        """
        Stop ROS2 node spinning and cleanup resources.
        """
        if not self.is_spinning:
            return
        
        try:
            self.is_spinning = False
            
            # Wait for spin thread to finish
            if self.spin_thread and self.spin_thread.is_alive():
                self.spin_thread.join(timeout=2.0)
            
            # Shutdown executor
            if self.executor:
                self.executor.shutdown()
                self.executor = None
            
            logger.info("ROS spinning stopped")
            
        except Exception as e:
            logger.error(f"Error stopping ROS spinning: {e}")
    
    def _spin_worker(self):
        """
        Worker function for background spin thread.
        """
        try:
            while self.is_spinning and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.01)
                
        except Exception as e:
            logger.error(f"ROS spin error: {e}")
        
        logger.debug("ROS spin thread finished")


# =============================================================================
# Global ROS2 Lifecycle Management
# =============================================================================

_rclpy_initialized = False
_initialization_lock = threading.Lock()


def initialize_ros2():
    """
    Initialize ROS2 context (thread-safe singleton).
    """
    global _rclpy_initialized
    
    with _initialization_lock:
        if not _rclpy_initialized:
            try:
                rclpy.init()
                _rclpy_initialized = True
                logger.info("ROS2 initialized")
                
            except Exception as e:
                logger.error(f"ROS2 initialization failed: {e}")
                raise


def shutdown_ros2():
    """
    Shutdown ROS2 context (thread-safe).
    """
    global _rclpy_initialized
    
    with _initialization_lock:
        if _rclpy_initialized:
            try:
                rclpy.shutdown()
                _rclpy_initialized = False
                logger.info("ROS2 shutdown")
                
            except Exception as e:
                logger.error(f"ROS2 shutdown error: {e}")