"""
Robot Vision Analyzer Module
Fast vision analysis pipeline using LIDAR spatial detection + YOLO (BLIP2 removed).
"""

import cv2
import time
import logging
import numpy as np
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
    Fast vision analysis pipeline for robot navigation (YOLO + LIDAR only).
    """
    
    def __init__(self, robot_controller=None):
        """
        Initialize vision analyzer.
        """
        # Caching for performance
        self._frame_cache = None
        self._last_analysis_time = 0
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

        # Camera configuration (TurtleBot3 Waffle)
        self.CAMERA_FOV_HORIZONTAL = 62.0  # degrees
        self.CAMERA_FOV_VERTICAL = 48.8    # degrees
        self.CAMERA_WIDTH = 640
        self.CAMERA_HEIGHT = 480
        self.CAMERA_CENTER_X = self.CAMERA_WIDTH / 2
        self.CAMERA_CENTER_Y = self.CAMERA_HEIGHT / 2
    
    def _merge_analysis_results(
        self,
        spatial: Dict,
        goal: str
    ) -> Dict[str, any]:
        """
        Merge spatial analysis into unified result (no BLIP2 needed).
        """
        return {
            'obstacles': spatial['obstacles'],
            'clear_paths': spatial['clear_paths'],
            'safety_score': spatial['safety_score'],
            'recommended_direction': spatial['recommended_direction'],
            'overall_confidence': spatial['spatial_confidence'],
            'immediate_action': self.navigation_reasoner._determine_immediate_action(
                spatial,
                {},  # No vision context needed
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
            'immediate_action': 'move_forward',
            'processing_time_ms': 0.1,
            'cached': True
        }
    
    def _initialize_yolo(self):
        """
        Lazy load YOLO model for object detection.
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

    def pixel_to_camera_angle(self, pixel_x: float, pixel_y: float) -> tuple[float, float]:
        """
        Convert pixel coordinates to camera-relative angles.
        """
        # Normalize to [-0.5, 0.5]
        norm_x = (pixel_x - self.CAMERA_CENTER_X) / self.CAMERA_WIDTH
        norm_y = (pixel_y - self.CAMERA_CENTER_Y) / self.CAMERA_HEIGHT
        
        # Scale by FOV
        horizontal_angle = norm_x * self.CAMERA_FOV_HORIZONTAL
        vertical_angle = norm_y * self.CAMERA_FOV_VERTICAL
        
        return horizontal_angle, vertical_angle


    def get_lidar_distance_for_bbox(
        self,
        bbox_center_x: float,
        full_lidar_scan: Dict[int, float]
    ) -> Optional[float]:
        """
        Get accurate LiDAR distance for YOLO detected object.
        """
        if not full_lidar_scan:
            logger.warning("[LIDAR FUSION] No full scan available")
            return None
        
        # Convert pixel to camera angle
        horizontal_angle, _ = self.pixel_to_camera_angle(bbox_center_x, self.CAMERA_CENTER_Y)
        
        # Camera faces forward (0°), so horizontal_angle IS the LiDAR angle
        lidar_angle = horizontal_angle
        
        # Query LiDAR scan at that angle
        distance = self.spatial_detector.query_distance_at_angle(
            full_lidar_scan,
            lidar_angle,
            tolerance=5  # ±5° tolerance
        )
        
        if distance is not None:
            logger.debug(
                f"[LIDAR FUSION] Pixel {bbox_center_x:.0f} → "
                f"Angle {lidar_angle:.1f}° → Distance {distance:.2f}m"
            )
        else:
            logger.warning(
                f"[LIDAR FUSION] No LiDAR reading at {lidar_angle:.1f}° "
                f"(tolerance ±5°)"
            )
        
        return distance
    
    def detect_target_objects(
        self,
        frame: np.ndarray,
        target_class: str,
        confidence_threshold: float = 0.5,
        full_lidar_scan: Dict[int, float] = None
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
                    distance_estimate, distance_source = self.estimate_object_distance(
                        h, y2, center_x, frame.shape[0], full_lidar_scan
                    )
                    
                    detected_obj = {
                        'class': self.yolo_model.names[class_id],
                        'class_id': class_id,
                        'confidence': confidence,
                        'bbox': bbox,
                        'center': (int(center_x), int(center_y)),
                        'size': (w, h),
                        'distance_estimate': distance_estimate,
                        'distance_source': distance_source
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

    def estimate_object_distance(
        self,
        bbox_height: int,
        bbox_bottom: float,
        bbox_center_x: float,
        frame_height: int,
        full_lidar_scan: Dict[int, float] = None
    ) -> tuple[float, str]:
        """
        Estimate object distance using LiDAR fusion or fallback heuristic.
        """
        # PRIORITY 1: Try LiDAR fusion
        if full_lidar_scan:
            lidar_distance = self.get_lidar_distance_for_bbox(bbox_center_x, full_lidar_scan)
            
            if lidar_distance is not None:
                return lidar_distance, 'lidar'
        
        # PRIORITY 2: Fallback to heuristic (when LiDAR unavailable/out of range)
        logger.warning("[DISTANCE] Falling back to heuristic estimation")
        
        normalized_height = bbox_height / frame_height
        
        if normalized_height > 0.7:
            heuristic_distance = 0.5
        elif normalized_height > 0.5:
            heuristic_distance = 1.0
        elif normalized_height > 0.3:
            heuristic_distance = 1.5
        elif normalized_height > 0.2:
            heuristic_distance = 2.5
        elif normalized_height > 0.1:
            heuristic_distance = 4.0
        else:
            heuristic_distance = 6.0
        
        return heuristic_distance, 'heuristic'
    
    def find_target_for_tracking(
        self,
        frame: np.ndarray,
        target_class: str,
        full_lidar_scan: Dict[int, float] = None
    ) -> Optional[Dict]:
        """
        Find closest target object for tracking (follow mission).
        """
        detected = self.detect_target_objects(
            frame, 
            target_class, 
            confidence_threshold=0.6,
            full_lidar_scan=full_lidar_scan
        )
        
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
        navigation_goal: str
    ) -> Dict[str, any]:
        """
        Fast vision analysis pipeline for navigation decisions (LIDAR spatial only).
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
            
            # Step 4: Merge results (no BLIP2 vision query needed)
            final_analysis = self._merge_analysis_results(
                spatial_analysis,
                navigation_goal
            )
            
            # Track performance
            processing_time = (time.time() - start_time) * 1000
            final_analysis['processing_time_ms'] = processing_time
            
            logger.debug(f"Spatial analysis completed in {processing_time:.1f}ms")
            
            self._last_analysis_time = current_time
            
            return final_analysis
            
        except Exception as e:
            logger.error(f"Spatial analysis failed: {e}")
            return self.safety_validator._get_safe_fallback_analysis()
        
    # Integrated mission-aware analysis
    async def analyze_with_mission(
        self,
        frame: np.ndarray,
        navigation_goal: str,
        mission_type: str = None,
        target_class: str = None
    ) -> Dict:
        """
        Unified vision analysis with mission-specific detection (YOLO only).
        """
        # Standard navigation analysis (LIDAR spatial)
        vision_analysis = await self.analyze_for_navigation(
            frame,
            navigation_goal
        )

        # Extract full LiDAR scan from result
        full_lidar_scan = vision_analysis.get('full_lidar_scan', {})
        
        # Add mission-specific YOLO detection
        detected_objects = []
        if mission_type in ['count_objects', 'follow_target', 'avoid_target'] and target_class:
            detected_objects = self.detect_target_objects(
                frame, 
                target_class,
                full_lidar_scan=full_lidar_scan
            )
            vision_analysis['detected_objects'] = detected_objects
        
        # Special handling for target tracking
        if mission_type == 'follow_target' and target_class:
            target_obj = self.find_target_for_tracking(
                frame, 
                target_class,
                full_lidar_scan=full_lidar_scan
            )
            
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