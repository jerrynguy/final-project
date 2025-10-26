"""
Model Manager Module
Manages loading and lifecycle of vision models (BLIP2 and YOLO).
"""

import gc
import time
import torch
import psutil
import logging
import threading
from pathlib import Path
from typing import Optional, Tuple
from dataclasses import dataclass

from transformers import logging as transformers_logging
from transformers import Blip2Processor, Blip2ForConditionalGeneration
from ultralytics import YOLO

transformers_logging.set_verbosity_error()

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
    Manages loading, caching, and lifecycle of robot vision models.
    """
    
    def __init__(
        self,
        model_name: str = "Salesforce/blip2-opt-2.7b",
        yolo_model_path: str = "/home/dung/nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/robot_vision_controller/model/yolo11n.pt"
    ):
        """
        Initialize model manager.
        """
        self.model_name = model_name
        self.yolo_model_path = yolo_model_path
        
        # Model instances
        self.model = None
        self.processor = None
        self.yolo_model = None
        self.device = None
        
        # Model metadata
        self.model_info = ModelInfo(model_name=model_name)
        self.yolo_info = ModelInfo(model_name="YOLO11n")
        
        # Thread safety
        self._model_lock = threading.Lock()
        self._is_loading = False
        
        # Setup computation device
        self._setup_device()
        
        # Setup cache directory for HuggingFace models
        self.cache_dir = Path.home() / ".cache" / "huggingface" / "hub"
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"Robot Vision Model Manager initialized for {model_name}")
    
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
        
        self.model_info.device = self.device
        self.yolo_info.device = self.device
    
    def preload_model(self) -> bool:
        """
        Preload both BLIP2 and YOLO models into memory.
        """
        if self.model_info.is_loaded and self.yolo_info.is_loaded:
            logger.info("Models already loaded")
            return True
        
        if self._is_loading:
            logger.info("Models are currently loading...")
            return False
        
        with self._model_lock:
            try:
                self._is_loading = True
                
                # Load BLIP2 if not already loaded
                if not self.model_info.is_loaded:
                    blip_success = self._load_blip2()
                    if not blip_success:
                        return False
                
                # Load YOLO if not already loaded
                if not self.yolo_info.is_loaded:
                    yolo_success = self._load_yolo()
                    if not yolo_success:
                        return False
                
                return True
                
            except Exception as e:
                logger.error(f"❌ Model loading failed: {e}")
                return False
            
            finally:
                self._is_loading = False
    
    def _load_blip2(self) -> bool:
        """
        Load BLIP2 model and processor.
        """
        try:
            start_time = time.time()
            logger.info(f"Loading BLIP2 model: {self.model_name}")
            
            # Load processor
            self.processor = Blip2Processor.from_pretrained(
                self.model_name,
                cache_dir=self.cache_dir,
                torch_dtype=torch.float32
            )
            
            # Load model
            self.model = Blip2ForConditionalGeneration.from_pretrained(
                self.model_name,
                cache_dir=self.cache_dir,
                torch_dtype=torch.float32,
            )
            
            # Move to device and set to eval mode
            self.model = self.model.to(self.device)
            self.model.eval()
            
            # Update metadata
            load_time = time.time() - start_time
            self.model_info.load_time_seconds = load_time
            self.model_info.is_loaded = True
            self._update_memory_usage()
            
            logger.info(f"✅ BLIP2 model loaded successfully!")
            logger.info(f"   Load time: {load_time:.1f}s")
            logger.info(f"   Device: {self.device}")
            logger.info(f"   Memory usage: {self.model_info.memory_usage_gb:.2f}GB")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ BLIP2 loading failed: {e}")
            self.model_info.is_loaded = False
            return False
    
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
    
    def is_model_ready(self) -> bool:
        """
        Check if BLIP2 model is ready for inference.
        """
        return (
            self.model_info.is_loaded and
            self.model is not None and
            self.processor is not None
        )
    
    def is_yolo_ready(self) -> bool:
        """
        Check if YOLO model is ready for inference.
        """
        return (
            self.yolo_info.is_loaded and
            self.yolo_model is not None
        )
    
    def get_model(self) -> Tuple:
        """
        Get BLIP2 model and processor, loading if necessary.
        """
        if not self.is_model_ready():
            logger.info("BLIP2 not ready, attempting synchronous preload")
            if not self.preload_model():
                raise RuntimeError("Failed to preload BLIP2 model")
        
        return self.processor, self.model
    
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
        Update inference statistics.
        """
        self.model_info.inference_count += 1
        self.model_info.total_inference_time += inference_time
        
        logger.debug(
            f"Inference #{self.model_info.inference_count}: "
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
    
    def _update_memory_usage(self):
        """
        Calculate and update model memory usage statistics.
        """
        if self.model is None:
            return
        
        try:
            # Calculate memory used by model parameters
            model_memory_bytes = sum(
                p.numel() * p.element_size()
                for p in self.model.parameters()
            )
            
            self.model_info.memory_usage_gb = model_memory_bytes / 1e9
            self.model_info.model_size_gb = self.model_info.memory_usage_gb
            
            logger.debug(
                f"Model memory usage: "
                f"{self.model_info.memory_usage_gb:.2f}GB"
            )
            
        except Exception as e:
            logger.warning(f"Could not calculate model memory usage: {e}")


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
    Preload vision models using singleton manager.
    """
    manager = get_robot_vision_model_manager()
    return manager.preload_model()


def is_model_ready() -> bool:
    """
    Check if BLIP2 model is ready.
    """
    manager = get_robot_vision_model_manager()
    return manager.is_model_ready()


def is_yolo_ready() -> bool:
    """
    Check if YOLO model is ready.
    """
    manager = get_robot_vision_model_manager()
    return manager.is_yolo_ready()