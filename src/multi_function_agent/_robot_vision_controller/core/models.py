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
from langchain_nvidia_ai_endpoints import ChatNVIDIA

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

@dataclass
class AIRecoveryModelInfo:
    """
    Metadata and statistics for AI Recovery model.
    """
    model_name: str = "meta/llama-3.2-3b-instruct"
    is_loaded: bool = False
    load_time_seconds: float = 0.0
    inference_count: int = 0
    total_inference_time: float = 0.0
    is_available: bool = False


# =============================================================================
# Robot Vision Model Manager
# =============================================================================

class RobotVisionModelManager:
    """
    Manages loading, caching, and lifecycle of YOLO + AI Recovery models.
    """
    
    def __init__(
        self,
        yolo_model_path: str = None  # Dynamic path resolution
    ):
        """
        Initialize model manager (YOLO + AI Recovery).
        """
        # Dynamic model path resolution
        if yolo_model_path is None:
            model_paths = [
                "/workspace/persistent_data/models/yolo11n.pt",  # Container volume
                Path.home() / "nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/robot_vision_controller/model/yolo11n.pt",  # Host
                Path.home() / ".local/share/Ultralytics/models/yolo11n.pt",  # Cache
                "yolo11n.pt"  # Auto-download
            ]
            
            for path in model_paths:
                if isinstance(path, str):
                    path = Path(path)
                if path.exists():
                    yolo_model_path = str(path)
                    logger.info(f"✅ Found YOLO model: {yolo_model_path}")
                    break
            
            if yolo_model_path is None:
                yolo_model_path = "yolo11n.pt"
                logger.warning("⏳ Model not found, will auto-download")
        
        self.yolo_model_path = yolo_model_path
        
        # YOLO model instance
        self.yolo_model = None
        self.device = None
        
        # YOLO metadata
        self.yolo_info = ModelInfo(model_name="YOLO11n")

        # AI Recovery model instance
        self.ai_recovery_model = None
        self.ai_recovery_info = AIRecoveryModelInfo()
        
        # Thread safety
        self._model_lock = threading.Lock()
        self._is_loading = False
        
        # Setup computation device
        self._setup_device()
        
        logger.info("Robot Vision Model Manager initialized (YOLO + AI Recovery)")
    
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
        
    def preload_ai_recovery_model(self) -> bool:
        """
        Preload AI Recovery model into memory.
        """
        if self.ai_recovery_info.is_loaded:
            logger.info("AI Recovery model already loaded")
            return True
        
        if self._is_loading:
            logger.info("AI Recovery model is currently loading...")
            return False
        
        with self._model_lock:
            try:
                self._is_loading = True
                return self._load_ai_recovery()
                
            except Exception as e:
                logger.error(f"❌ AI Recovery model loading failed: {e}")
                return False
            
            finally:
                self._is_loading = False

    def _load_ai_recovery(self) -> bool:
        """
        Load AI Recovery model using LangChain NVIDIA endpoint.
        """
        try:
            start_time = time.time()
            model_name=self.ai_recovery_info.model_name

            logger.info(f"Loading AI Recovery model: {self.ai_recovery_info.model_name}")
            
            # Load AI Recovery model
            self.ai_recovery_model = ChatNVIDIA(
                model_name=model_name,
                temperature=0.3,
                max_tokens=100
            )
            
            # Test with dummy inference to warm up connection
            logger.info("[AI RECOVERY] Testing model connection....")
            test_response = self.ai_recovery_model.invoke(
                [{"role": "user", "content": "ping"}])
            
            if test_response:
                load_time = time.time() - start_time
                self.ai_recovery_info.load_time_seconds = load_time
                self.ai_recovery_info.is_loaded = True
                self.ai_recovery_info.is_available = True
                logger.info("[AI RECOVERY] Model connection successful")

                return True
            else:
                raise RuntimeError("AI Recovery model test inference failed")
            
        except Exception as e:
            logger.error(f"❌ AI Recovery model loading failed: {e}")
            self.ai_recovery_info.is_loaded = False
            self.ai_recovery_info.is_available = False
            return False
    
    def is_yolo_ready(self) -> bool:
        """
        Check if YOLO model is ready for inference.
        """
        return (
            self.yolo_info.is_loaded and
            self.yolo_model is not None
        )
    
    def is_ai_recovery_ready(self) -> bool:
        """
        Check if AI Recovery model is ready for inference.
        """
        return (
            self.ai_recovery_info.is_loaded and
            self.ai_recovery_info.is_available and
            self.ai_recovery_model is not None
        )
    
    def get_ai_recovery_model(self):
        """
        Get AI Recovery model, loading if necessary.
        """
        if not self.is_ai_recovery_ready():
            logger.info("AI Recovery model not ready, attempting synchronous preload")
            if not self.preload_ai_recovery_model():
                raise RuntimeError("Failed to preload AI Recovery model")
        
        return self.ai_recovery_model
    
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

    def update_ai_recovery_inference_time(self, inference_time: float):
        """
        Update AI Recovery inference statistics.
        """
        self.ai_recovery_info.inference_count += 1
        self.ai_recovery_info.total_inference_time += inference_time

        avg_time = (
            self.ai_recovery_info.total_inference_time /
            self.ai_recovery_info.inference_count
        )
        
        logger.debug(
            f"AI Recovery inference #{self.ai_recovery_info.inference_count}: "
            f"{inference_time:.3f}s (avg: {avg_time:.3f}s)"
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
    Preload YOLO + AI Recovery models using singleton manager.
    """
    manager = get_robot_vision_model_manager()
    
    yolo_sucess = manager.preload_model()
    ai_recovery_success = manager.preload_ai_recovery_model()

    if yolo_sucess and ai_recovery_success:
        logger.info("✅ All Robot Vision models preloaded successfully")
        return True
    elif yolo_sucess:
        logger.warning("⚠️ Only YOLO model preloaded successfully")
        return True
    elif ai_recovery_success:
        logger.warning("⚠️ Only AI Recovery model preloaded successfully")
        return True
    else:
        logger.error("❌ Failed to preload any Robot Vision models")
        return False


def is_yolo_ready() -> bool:
    """
    Check if YOLO model is ready.
    """
    manager = get_robot_vision_model_manager()
    return manager.is_yolo_ready()

def is_ai_recovery_ready() -> bool:
    """
    Check if AI Recovery model is ready.
    """
    manager = get_robot_vision_model_manager()
    return manager.is_ai_recovery_ready()