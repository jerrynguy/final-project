"""
Robot Vision Analyzer Module
Fast vision analysis pipeline combining spatial detection and minimal VLM queries.
"""

import cv2
import time
import torch
import logging
import numpy as np
from PIL import Image
from typing import Dict, List, Optional

from ultralytics import YOLO
from multi_function_agent.robot_vision_controller.perception.spatial_detector import SpatialDetector
from multi_function_agent.robot_vision_controller.navigation.navigation_reasoner import NavigationReasoner
from multi_function_agent.robot_vision_controller.utils.safety_checks import SafetyValidator
from multi_function_agent.robot_vision_controller.utils.geometry_utils import FastFrameProcessor
from multi_function_agent.robot_vision_controller.core.models import get_robot_vision_model_manager

logger = logging.getLogger(__name__)


# =============================================================================
# Robot Vision Analyzer
# =============================================================================

class RobotVisionAnalyzer:
    """
    Fast vision analysis pipeline for robot navigation.
    """
    
    def __init__(self, robot_controller=None):
        """
        Initialize vision analyzer.
        """
        # Caching for performance
        self._frame_cache = None
        self._last_analysis_time = 0
        self._last_vision_query_time = 0
        self._vision_query_interval = 2.0
        self._cached_vision_result = None
        self._min_analysis_interval = 0.1
        
        # Core components
        self.spatial_detector = SpatialDetector()
        self.processor = FastFrameProcessor()
        self.safety_validator = SafetyValidator()
        self.navigation_reasoner = NavigationReasoner()
        self.robot_controller = robot_controller
        
        # Navigation parameters
        self.OBSTACLE_THRESHOLD = 0.3
        self.SAFETY_ZONE_SIZE = 50
        self.MIN_PATH_WIDTH = 80
        
        # YOLO model for object detection (mission-specific)
        self.yolo_model = None
        self._yolo_initialized = False
    
    async def _minimal_vision_query(
        self,
        frame: np.ndarray,
        processor,
        model,
        navigation_goal: str,
        spatial_analysis: Dict
    ) -> Dict[str, str]:
        """
        Execute minimal VLM query with intelligent caching.
        """
        current_time = time.time()
        
        # Use cache if recent
        if (self._cached_vision_result is not None and
            current_time - self._last_vision_query_time < self._vision_query_interval):
            logger.debug(f"Using cached vision result (age: {current_time - self._last_vision_query_time:.1f}s)")
            return self._cached_vision_result
        
        # Skip inference for clearly safe paths
        if spatial_analysis.get('safety_score', 0) >= 8 and 'center' in spatial_analysis.get('clear_paths', []):
            logger.debug("Clear path detected, skipping model inference")
            return {
                'vision_instruction': 'move forward safely',
                'model_confidence': 0.9,
            }
        
        # Prepare input for VLM
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)
        pil_image = pil_image.resize((128, 128))  # Fast resize
        
        prompt = f"Goal: {navigation_goal}"
        
        # Run VLM inference
        device = next(model.parameters()).device
        inputs = processor(pil_image, prompt, return_tensors="pt").to(device)
        
        with torch.no_grad():
            generated_ids = model.generate(
                **inputs,
                max_length=100,
                max_new_tokens=50,
                num_beams=2,
                do_sample=False,
                early_stopping=True,
                pad_token_id=processor.tokenizer.eos_token_id,
                use_cache=True,
            )
        
        result = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
        
        # Clean up result
        if prompt in result:
            result = result.replace(prompt, "").strip()
        
        navigation_instruction = result.split('.')[0][:100]
        
        # Cache result
        self._last_vision_query_time = current_time
        
        self._cached_vision_result = {
            'vision_instruction': navigation_instruction,
            'model_confidence': 0.8,
        }
        
        return self._cached_vision_result
    
    def _merge_analysis_results(
        self,
        spatial: Dict,
        vision: Dict,
        goal: str
    ) -> Dict[str, any]:
        """
        Merge spatial and vision analysis into unified result.
        """
        return {
            'obstacles': spatial['obstacles'],
            'clear_paths': spatial['clear_paths'],
            'safety_score': spatial['safety_score'],
            'recommended_direction': spatial['recommended_direction'],
            'vision_instruction': vision.get('vision_instruction', 'continue straight'),
            'model_confidence': vision.get('model_confidence', 0.5),
            'overall_confidence': min(
                spatial['spatial_confidence'],
                vision.get('model_confidence', 0.5)
            ),
            'immediate_action': self.navigation_reasoner._determine_immediate_action(
                spatial,
                vision,
                goal
            ),
            'navigation_priority': 'safety_first',
        }
    
    def _get_cached_analysis(self) -> Dict[str, any]:
        """
        Return cached analysis for rate-limited calls.
        """
        return {
            'obstacles': [],
            'clear_paths': ['center'],
            'safety_score': 5,
            'recommended_direction': 'center',
            'vision_instruction': 'continue',
            'immediate_action': 'move_forward',
            'processing_time_ms': 0.1,
            'cached': True
        }
    
    def _initialize_yolo(self):
        """
        Lazy load YOLO model for object detection.
        
        Only loads when needed to save memory.
        """
        if not self._yolo_initialized:
            try:
                model_manager = get_robot_vision_model_manager()
                self.yolo_model = model_manager.get_yolo_model()
                self._yolo_initialized = True
                logger.info("[YOLO] Model initialized for object detection")
            except Exception as e:
                logger.error(f"[YOLO] Failed to initialize: {e}")
                self._yolo_initialized = False
    
    def detect_target_objects(
        self,
        frame: np.ndarray,
        target_class: str,
        confidence_threshold: float = 0.5
    ) -> List[Dict]:
        """
        Detect specific object class in frame using YOLO.
        """
        if not self._yolo_initialized:
            self._initialize_yolo()
        
        if self.yolo_model is None:
            logger.warning("[YOLO] Model not available, returning empty detections")
            return []
        
        try:
            # Run YOLO inference
            results = self.yolo_model(frame, conf=confidence_threshold, verbose=False)
            
            detected_objects = []
            
            # COCO class names mapping (common targets)
            class_map = {
                'person': 0,
                'bicycle': 1,
                'car': 2,
                'bottle': 39,
                'cup': 41,
                'chair': 56,
                'couch': 57,
                'potted plant': 58,
                'laptop': 63,
                'cell phone': 67
            }
            
            target_class_id = class_map.get(target_class.lower())
            
            # Process detection results
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                for box in boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    
                    # Filter by target class
                    if target_class_id is not None and class_id != target_class_id:
                        continue
                    
                    # Extract bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    bbox = [int(x1), int(y1), int(x2), int(y2)]
                    w, h = int(x2 - x1), int(y2 - y1)
                    center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
                    
                    # Estimate distance from bbox height
                    distance_estimate = self._estimate_distance_from_bbox(h, y2, frame.shape[0])
                    
                    detected_obj = {
                        'class': self.yolo_model.names[class_id],
                        'class_id': class_id,
                        'confidence': confidence,
                        'bbox': bbox,
                        'center': (int(center_x), int(center_y)),
                        'size': (w, h),
                        'distance_estimate': distance_estimate
                    }
                    
                    detected_objects.append(detected_obj)
                    
                    logger.debug(
                        f"[YOLO] Detected {detected_obj['class']} "
                        f"at {center_x:.0f},{center_y:.0f} "
                        f"(~{distance_estimate:.1f}m, conf={confidence:.2f})"
                    )
            
            if detected_objects:
                logger.info(f"[YOLO] Found {len(detected_objects)} {target_class}(s)")
            
            return detected_objects
            
        except Exception as e:
            logger.error(f"[YOLO] Detection failed: {e}")
            return []
    
    def _estimate_distance_from_bbox(
        self,
        bbox_height: int,
        bbox_bottom: float,
        frame_height: int
    ) -> float:
        """
        Estimate distance to object based on bounding box size.
        """
        normalized_height = bbox_height / frame_height
        normalized_bottom = bbox_bottom / frame_height
        
        # Heuristic calibration (adjust based on camera setup)
        if normalized_height > 0.7:
            return 0.5  # Very close
        elif normalized_height > 0.5:
            return 1.0
        elif normalized_height > 0.3:
            return 1.5
        elif normalized_height > 0.2:
            return 2.5
        elif normalized_height > 0.1:
            return 4.0
        else:
            return 6.0
    
    def find_target_for_tracking(
        self,
        frame: np.ndarray,
        target_class: str
    ) -> Optional[Dict]:
        """
        Find closest target object for tracking (follow mission).
        """
        detected = self.detect_target_objects(frame, target_class, confidence_threshold=0.6)
        
        if not detected:
            return None
        
        # Find closest object by distance estimate
        closest = min(detected, key=lambda obj: obj['distance_estimate'])
        
        logger.info(
            f"[TRACKING] Target locked: {closest['class']} "
            f"at ~{closest['distance_estimate']:.1f}m"
        )
        
        return closest
    
    async def analyze_for_navigation(
        self,
        frame: np.ndarray,
        processor,
        model,
        navigation_goal: str
    ) -> Dict[str, any]:
        """
        Fast vision analysis pipeline for navigation decisions.
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self._last_analysis_time < self._min_analysis_interval:
            return self._get_cached_analysis()
        
        start_time = time.time()
        
        try:
            # Step 1: Preprocess frame
            preprocessed_frame = self.processor.preprocess_for_speed(frame)
            edges_frame = self.processor.enhance_edges_fast(preprocessed_frame)
            
            # Step 2: Gather sensor data from robot controller
            lidar_scan = None
            map_data = None
            robot_position = None
            
            if self.robot_controller:
                if hasattr(self.robot_controller, 'lidar_data'):
                    lidar_scan = self.robot_controller.lidar_data
                
                if lidar_scan is None:
                    logger.error("❌ No lidar data available")
                    return self.safety_validator._get_safe_fallback_analysis()
            
            # Step 3: Spatial analysis (LiDAR-primary)
            spatial_analysis = self.spatial_detector._fast_spatial_analysis(
                edges_frame,
                lidar_scan=lidar_scan,
            )
            
            # Step 4: Minimal VLM query for context
            vision_analysis = await self._minimal_vision_query(
                preprocessed_frame,
                processor,
                model,
                navigation_goal,
                spatial_analysis
            )
            
            # Step 5: Merge results
            final_analysis = self._merge_analysis_results(
                spatial_analysis,
                vision_analysis,
                navigation_goal
            )
            
            # Track performance
            processing_time = (time.time() - start_time) * 1000
            final_analysis['processing_time_ms'] = processing_time
            
            logger.debug(f"Robot vision analysis completed in {processing_time:.1f}ms")
            
            self._last_analysis_time = current_time
            
            return final_analysis
            
        except Exception as e:
            logger.error(f"Fast analysis failed: {e}")
            return self.safety_validator._get_safe_fallback_analysis()
        
    # Integrated mission-aware analysis
    async def analyze_with_mission(
        self,
        frame: np.ndarray,
        processor,
        model,
        navigation_goal: str,
        mission_type: str = None,
        target_class: str = None
    ) -> Dict:
        """
        Unified vision analysis with mission-specific detection.
        """
        # Standard navigation analysis
        vision_analysis = await self.analyze_for_navigation(
            frame,
            processor,
            model,
            navigation_goal
        )
        
        # Add mission-specific detection
        detected_objects = []
        if mission_type in ['count_objects', 'follow_target', 'avoid_target'] and target_class:
            detected_objects = self.detect_target_objects(frame, target_class)
            vision_analysis['detected_objects'] = detected_objects
        
        # Special handling for target tracking
        if mission_type == 'follow_target' and target_class:
            target_obj = self.find_target_for_tracking(frame, target_class)
            
            if target_obj:
                center_x = target_obj['center'][0]
                # Determine direction (assuming 640px width)
                if 260 < center_x < 380:
                    direction = 'forward'
                elif center_x <= 260:
                    direction = 'left'
                else:
                    direction = 'right'
                
                vision_analysis['target_tracking'] = {
                    'visible': True,
                    'position': target_obj['center'],
                    'distance': target_obj['distance_estimate'],
                    'direction': direction,
                    'bbox': target_obj['bbox']
                }
            else:
                vision_analysis['target_tracking'] = {'visible': False}
        
        return vision_analysis