"""
Geometry Utils Module
Fast frame preprocessing utilities for efficient vision processing.
"""

import cv2
import math
import numpy as np
from typing import Tuple


# Fast Frame Processor
class FastFrameProcessor:
    """
    Fast image preprocessing utilities optimized for real-time navigation.
    """
    
    @staticmethod
    def preprocess_for_speed(
        frame: np.ndarray,
        target_size: Tuple[int, int] = (320, 240)
    ) -> np.ndarray:
        """
        Quickly downsample frame for fast processing.
        """
        resized = cv2.resize(
            frame,
            target_size,
            interpolation=cv2.INTER_NEAREST
        )
        
        return resized
    
    @staticmethod
    def enhance_edges_fast(frame: np.ndarray) -> np.ndarray:
        """
        Fast edge enhancement using Sobel operators.
        """
        # Convert to grayscale if needed
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Compute gradients
        grad_x = cv2.Sobel(gray, cv2.CV_8U, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_8U, 0, 1, ksize=3)
        
        # Combine gradients
        edges = cv2.addWeighted(grad_x, 0.5, grad_y, 0.5, 0)
        
        return edges
    
    @staticmethod
    def process_frame_for_speed(
        frame: np.ndarray,
        target_width: int = 320,
        target_height: int = 240
    ) -> np.ndarray:
        """
        Adaptively scale frame to target size while preserving aspect ratio.
        """
        height, width = frame.shape[:2]
        
        # Only resize if frame exceeds target dimensions
        if width > target_width or height > target_height:
            scale_w = target_width / width
            scale_h = target_height / height
            scale = min(scale_w, scale_h)  # Preserve aspect ratio
            
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            frame = cv2.resize(
                frame,
                (new_width, new_height),
                interpolation=cv2.INTER_NEAREST
            )
        
        return frame