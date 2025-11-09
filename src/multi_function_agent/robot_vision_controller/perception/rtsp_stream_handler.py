"""
RTSP Stream Handler Module
Real-time video stream management with automatic reconnection and frame buffering.
"""

import cv2
import time
import yaml
import logging
import asyncio
import threading
import numpy as np
from queue import Queue, Empty
from dataclasses import dataclass
from typing import Optional, Tuple

from multi_function_agent.robot_vision_controller.utils.geometry_utils import FastFrameProcessor

logger = logging.getLogger(__name__)


# =============================================================================
# Stream Information Data Structure
# =============================================================================

@dataclass
class StreamInfo:
    """
    Video stream metadata and statistics.
    
    Attributes:
        width: Frame width in pixels
        height: Frame height in pixels
        fps: Frames per second
        is_connected: Connection status
        total_frames: Total frames received
        dropped_frames: Number of dropped frames
    """
    width: int
    height: int
    fps: float
    is_connected: bool
    total_frames: int = 0
    dropped_frames: int = 0


# =============================================================================
# RTSP Stream Handler
# =============================================================================

class RTSPStreamHandler:
    """
    Manages RTSP video stream with robust error handling.
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize RTSP stream handler.
        """
        # Dynamic config path resolution
        if config_path is None:
            from pathlib import Path
            config_paths = [
                Path("/workspace/mounted_code/src/multi_function_agent/configs/config.yml"),  # Container
                Path(__file__).parent.parent.parent / "configs/config.yml",  # Relative
                Path.home() / "nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/configs/config.yml",  # Host
            ]
            
            for path in config_paths:
                if path.exists():
                    config_path = str(path)
                    break
        
        self.config = self._load_config(config_path)
        
        # Stream objects
        self._cap = None
        self._stream_url = None
        self._is_streaming = False
        self._stream_thread = None
        self._frame_queue = Queue(maxsize=self.config['buffer_size'])
        
        # Statistics tracking
        self._stats = StreamInfo(0, 0, 0, False)
        self._frame_count = 0
        self._dropped_frame_count = 0
        
        self.processor = FastFrameProcessor()
    
    def _load_config(self, config_path: str) -> dict:
        """
        Load stream configuration from YAML file.
        """
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config.get('rtsp_stream', {
                    'buffer_size': 1,
                    'target_fps': 10,
                    'target_width': 640,
                    'target_height': 480,
                    'connection_timeout': 5.0,
                    'reconnect_delay': 2.0,
                    'max_reconnect_attempts': 3
                })
                
        except FileNotFoundError:
            logger.warning(f"Config file {config_path} not found, using defaults")
            return {
                'buffer_size': 1,
                'target_fps': 10,
                'target_width': 640,
                'target_height': 480,
                'connection_timeout': 5.0,
                'reconnect_delay': 2.0,
                'max_reconnect_attempts': 3
            }
    
    async def validate_stream(self, stream_url: str) -> Tuple[bool, str]:
        """
        Validate RTSP stream connectivity and frame availability.
        """
        try:
            cap = cv2.VideoCapture(stream_url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if not cap.isOpened():
                return False, "Cannot connect to stream"
            
            ret, frame = cap.read()
            cap.release()
            
            if not ret or frame is None:
                return False, "Cannot read frame from stream"
            
            if frame.size == 0:
                return False, "Empty frame received"
            
            return True, "Stream validated successfully"
            
        except Exception as e:
            return False, f"Validation error: {str(e)}"
    
    async def start_stream(self, stream_url: str) -> bool:
        """
        Start streaming from RTSP URL.
        """
        try:
            # Stop any existing stream
            await self.stop_stream()
            
            self._stream_url = stream_url
            self._cap = cv2.VideoCapture(stream_url)
            
            # Configure capture settings
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cap.set(cv2.CAP_PROP_FPS, self.config['target_fps'])
            
            if not self._cap.isOpened():
                return False
            
            self._update_stream_info()
            
            # Start streaming thread
            self._is_streaming = True
            self._stream_thread = threading.Thread(
                target=self._stream_loop,
                daemon=True
            )
            self._stream_thread.start()
            
            logger.info(
                f"Real-time stream started: "
                f"{self._stats.width}x{self._stats.height}"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to start stream: {e}")
            return False
    
    async def stop_stream(self):
        """
        Stop streaming and cleanup resources.
        """
        self._is_streaming = False
        
        # Wait for thread to finish
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=2.0)
        
        # Release video capture
        if self._cap:
            self._cap.release()
            self._cap = None
        
        # Clear frame queue
        while not self._frame_queue.empty():
            try:
                self._frame_queue.get_nowait()
            except Empty:
                break
    
    def _stream_loop(self):
        """
        Background thread main loop for continuous frame capture.
        """
        reconnect_attempts = 0
        frame_interval = 1.0 / self.config['target_fps']
        last_frame_time = 0
        
        while self._is_streaming:
            try:
                current_time = time.time()
                
                # Rate limiting
                if current_time - last_frame_time < frame_interval:
                    time.sleep(0.001)
                    continue
                
                # Check connection and reconnect if needed
                if not self._cap or not self._cap.isOpened():
                    if reconnect_attempts < self.config['max_reconnect_attempts']:
                        if self._reconnect():
                            reconnect_attempts = 0
                        else:
                            reconnect_attempts += 1
                            time.sleep(self.config['reconnect_delay'])
                        continue
                    else:
                        logger.error("Max reconnection attempts reached")
                        break
                
                # Read frame
                ret, frame = self._cap.read()
                
                if not ret or frame is None:
                    self._dropped_frame_count += 1
                    continue
                
                # Drop old frame if queue is full
                if self._frame_queue.full():
                    try:
                        self._frame_queue.get_nowait()
                        self._dropped_frame_count += 1
                    except Empty:
                        pass
                
                # Add frame to queue with metadata
                frame_data = {
                    'frame': frame,
                    'timestamp': current_time,
                    'frame_number': self._frame_count
                }
                
                self._frame_queue.put_nowait(frame_data)
                self._frame_count += 1
                last_frame_time = current_time
                
            except Exception as e:
                logger.error(f"Stream loop error: {e}")
                time.sleep(0.1)
    
    def _reconnect(self) -> bool:
        """
        Attempt to reconnect to stream.
        """
        try:
            if self._cap:
                self._cap.release()
            
            self._cap = cv2.VideoCapture(self._stream_url)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cap.set(cv2.CAP_PROP_FPS, self.config['target_fps'])
            
            if self._cap.isOpened():
                self._update_stream_info()
                return True
                
        except Exception as e:
            logger.error(f"Reconnection failed: {e}")
        
        return False
    
    def _update_stream_info(self):
        """
        Update stream information from video capture properties.
        """
        if self._cap and self._cap.isOpened():
            self._stats.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self._stats.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self._stats.fps = self._cap.get(cv2.CAP_PROP_FPS)
            self._stats.is_connected = True
        else:
            self._stats.is_connected = False
    
    async def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        Get the most recent frame from stream.
        """
        try:
            if not self._is_streaming:
                return None
            
            latest_frame = None
            
            # Drain queue to get most recent frame
            while not self._frame_queue.empty():
                try:
                    frame_data = self._frame_queue.get_nowait()
                    
                    # Only use recent frames (< 200ms old)
                    if time.time() - frame_data['timestamp'] < 0.2:
                        latest_frame = frame_data['frame']
                    
                except Empty:
                    break
            
            return latest_frame
            
        except Exception as e:
            logger.error(f"Failed to get latest frame: {e}")
            return None
    
    def get_stream_stats(self) -> dict:
        """
        Get streaming statistics and metadata.
        """
        total_frames = self._frame_count
        drop_rate = self._dropped_frame_count / max(1, total_frames) * 100
        
        return {
            'is_streaming': self._is_streaming,
            'total_frames': total_frames,
            'dropped_frames': self._dropped_frame_count,
            'drop_rate_percent': drop_rate,
            'queue_size': self._frame_queue.qsize(),
            'stream_info': {
                'width': self._stats.width,
                'height': self._stats.height,
                'fps': self._stats.fps,
                'is_connected': self._stats.is_connected
            }
        }