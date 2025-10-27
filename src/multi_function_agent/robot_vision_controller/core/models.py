"""
Model Manager Module
Manages loading and lifecycle of YOLO model (BLIP2 removed).
"""

import gc
import time
import torch
import psutil
import logging
import threading
from pathlib import Path
from typing import Optional
from dataclasses import dataclass

from ultralytics import YOLO

logger = logging.getLogger(__name__)


# =============================================================================
# Model Information Data Structure
# =============================================================================

@dataclass
class ModelInfo:
    """
    Metadata and statistics for loaded models.
    """
    model_name: str = ""
    model_size_gb: float = 0.0
    device: str = "cpu"
    is_loaded: bool = False
    load_time_seconds: float = 0.0
    memory_usage_gb: float = 0.0
    inference_count: int = 0
    total_inference_time: float = 0.0


# =============================================================================
# Robot Vision Model Manager
# =============================================================================

class RobotVisionModelManager:
    """
    Manages loading, caching, and lifecycle of YOLO model.
    """
    
    def __init__(
        self,
        yolo_model_path: str = "/home/dung/nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/robot_vision_controller/model/yolo11n.pt"
    ):
        """
        Initialize model manager (YOLO only).
        """
        self.yolo_model_path = yolo_model_path
        
        # Model instance
        self.yolo_model = None
        self.device = None
        
        # Model metadata
        self.yolo_info = ModelInfo(model_name="YOLO11n")
        
        # Thread safety
        self._model_lock = threading.Lock()
        self._is_loading = False
        
        # Setup computation device
        self._setup_device()
        
        logger.info("Robot Vision Model Manager initialized (YOLO only)")
    
    def _setup_device(self):
        """
        Auto-detect and configure computation device.
        """
        if hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            self.device = "mps"
            logger.info("Using Apple Silicon MPS")
        else:
            self.device = "cpu"
            
            cpu_count = psutil.cpu_count()
            ram_gb = psutil.virtual_memory().total / 1e9
            
            logger.info(f"Using CPU: {cpu_count} cores, {ram_gb:.1f}GB RAM")
            
            # Set optimal thread count (max 8 threads)
            torch.set_num_threads(min(cpu_count, 8))
        
        self.yolo_info.device = self.device
    
    def preload_model(self) -> bool:
        """
        Preload YOLO model into memory.
        """
        if self.yolo_info.is_loaded:
            logger.info("YOLO already loaded")
            return True
        
        if self._is_loading:
            logger.info("YOLO is currently loading...")
            return False
        
        with self._model_lock:
            try:
                self._is_loading = True
                return self._load_yolo()
                
            except Exception as e:
                logger.error(f"❌ YOLO loading failed: {e}")
                return False
            
            finally:
                self._is_loading = False
    
    def _load_yolo(self) -> bool:
        """
        Load YOLO model from local weights file.
        """
        try:
            start_time = time.time()
            logger.info(f"Loading YOLO model: {self.yolo_model_path}")
            
            # Load YOLO model
            self.yolo_model = YOLO(self.yolo_model_path)
            
            # Debug logging
            logger.info(f"[YOLO DEBUG] Model type: {type(self.yolo_model)}")
            logger.info(f"[YOLO DEBUG] Model names: {self.yolo_model.names}")
            logger.info(f"[YOLO DEBUG] Number of classes: {len(self.yolo_model.names)}")
            
            # Update metadata
            load_time = time.time() - start_time
            self.yolo_info.load_time_seconds = load_time
            self.yolo_info.is_loaded = True
            
            logger.info(f"✅ YOLO model loaded successfully!")
            logger.info(f"   Load time: {load_time:.1f}s")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ YOLO loading failed: {e}")
            self.yolo_info.is_loaded = False
            return False
    
    def is_yolo_ready(self) -> bool:
        """
        Check if YOLO model is ready for inference.
        """
        return (
            self.yolo_info.is_loaded and
            self.yolo_model is not None
        )
    
    def get_yolo_model(self):
        """
        Get YOLO model, loading if necessary.
        """
        if not self.is_yolo_ready():
            logger.info("YOLO not ready, attempting synchronous preload")
            if not self.preload_model():
                raise RuntimeError("Failed to preload YOLO model")
        
        return self.yolo_model
    
    def update_inference_time(self, inference_time: float):
        """
        Update YOLO inference statistics.
        """
        self.yolo_info.inference_count += 1
        self.yolo_info.total_inference_time += inference_time
        
        logger.debug(
            f"YOLO inference #{self.yolo_info.inference_count}: "
            f"{inference_time:.3f}s"
        )
    
    def cleanup_gpu_memory(self):
        """
        Clean up GPU/MPS memory cache.
        """
        try:
            if self.device == "mps":
                if hasattr(torch.mps, 'empty_cache'):
                    torch.mps.empty_cache()
                logger.debug("MPS memory cache cleared")
            
            gc.collect()
            
        except Exception as e:
            logger.warning(f"Memory cleanup failed: {e}")


# =============================================================================
# Global Singleton Instance
# =============================================================================

_model_manager_instance: Optional[RobotVisionModelManager] = None
_manager_lock = threading.Lock()


def get_robot_vision_model_manager() -> RobotVisionModelManager:
    """
    Get singleton instance of model manager.
    """
    global _model_manager_instance
    
    with _manager_lock:
        if _model_manager_instance is None:
            _model_manager_instance = RobotVisionModelManager()
        
        return _model_manager_instance


def preload_robot_vision_model() -> bool:
    """
    Preload YOLO model using singleton manager.
    """
    manager = get_robot_vision_model_manager()
    return manager.preload_model()


def is_yolo_ready() -> bool:
    """
    Check if YOLO model is ready.
    """
    manager = get_robot_vision_model_manager()
    return manager.is_yolo_ready()