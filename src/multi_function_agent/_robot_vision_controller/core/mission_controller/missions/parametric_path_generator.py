"""
Parametric Path Generator Module
Generates waypoint sequences for geometric curves.
"""

import numpy as np
import logging
from typing import List, Dict, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """Single waypoint in path."""
    x: float
    y: float
    theta: float  # Heading angle (radians)
    index: int


class ParametricPathGenerator:
    """
    Generate waypoints for parametric curves.
    
    Supported shapes:
    - Eclipse (ellipse)
    - Spiral (Archimedean)
    - Figure-8 (Lissajous)
    - Circle (special case of eclipse)
    - Custom parametric functions
    """
    
    def __init__(self, resolution: int = 100):
        """
        Initialize generator.
        
        Args:
            resolution: Number of waypoints per curve (default: 100)
        """
        self.resolution = resolution
    
    def generate_eclipse(
        self,
        center: Tuple[float, float],
        semi_major: float,
        semi_minor: float,
        rotation: float = 0.0
    ) -> List[Waypoint]:
        """
        Generate eclipse (ellipse) path.
        
        Parametric equation:
          x = cx + a*cos(t)*cos(θ) - b*sin(t)*sin(θ)
          y = cy + a*cos(t)*sin(θ) + b*sin(t)*cos(θ)
        
        Args:
            center: (cx, cy) center position
            semi_major: Semi-major axis length (a)
            semi_minor: Semi-minor axis length (b)
            rotation: Rotation angle in radians (θ)
        
        Returns:
            List of waypoints
        """
        cx, cy = center
        a = semi_major
        b = semi_minor
        theta = rotation
        
        waypoints = []
        
        for i, t in enumerate(np.linspace(0, 2*np.pi, self.resolution)):
            # Rotated eclipse equations
            x = cx + a*np.cos(t)*np.cos(theta) - b*np.sin(t)*np.sin(theta)
            y = cy + a*np.cos(t)*np.sin(theta) + b*np.sin(t)*np.cos(theta)
            
            # Calculate heading (tangent direction)
            dx = -a*np.sin(t)*np.cos(theta) - b*np.cos(t)*np.sin(theta)
            dy = -a*np.sin(t)*np.sin(theta) + b*np.cos(t)*np.cos(theta)
            heading = np.arctan2(dy, dx)
            
            waypoints.append(Waypoint(x=x, y=y, theta=heading, index=i))
        
        logger.info(
            f"[PATH] Generated eclipse: {len(waypoints)} waypoints, "
            f"a={semi_major:.2f}m, b={semi_minor:.2f}m"
        )
        
        return waypoints
    
    def generate_spiral(
        self,
        center: Tuple[float, float],
        a: float,
        b: float,
        turns: int = 3
    ) -> List[Waypoint]:
        """
        Generate Archimedean spiral path.
        
        Parametric equation:
          r = a + b*theta
          x = cx + r*cos(theta)
          y = cy + r*sin(theta)
        
        Args:
            center: (cx, cy) center position
            a: Initial radius offset
            b: Spiral growth rate
            turns: Number of complete turns
        
        Returns:
            List of waypoints
        """
        cx, cy = center
        
        waypoints = []
        
        for i, theta in enumerate(np.linspace(0, 2*np.pi*turns, self.resolution)):
            r = a + b*theta
            
            x = cx + r*np.cos(theta)
            y = cy + r*np.sin(theta)
            
            # Heading (perpendicular to radius)
            heading = theta + np.pi/2
            
            waypoints.append(Waypoint(x=x, y=y, theta=heading, index=i))
        
        logger.info(
            f"[PATH] Generated spiral: {len(waypoints)} waypoints, "
            f"{turns} turns, growth={b:.3f}"
        )
        
        return waypoints
    
    def generate_figure8(
        self,
        center: Tuple[float, float],
        width: float,
        height: float
    ) -> List[Waypoint]:
        """
        Generate figure-8 path (Lissajous curve).
        
        Parametric equation:
          x = cx + width*sin(t)
          y = cy + height*sin(2t)
        
        Args:
            center: (cx, cy) center position
            width: Horizontal extent
            height: Vertical extent
        
        Returns:
            List of waypoints
        """
        cx, cy = center
        
        waypoints = []
        
        for i, t in enumerate(np.linspace(0, 2*np.pi, self.resolution)):
            x = cx + width*np.sin(t)
            y = cy + height*np.sin(2*t)
            
            # Calculate heading
            dx = width*np.cos(t)
            dy = 2*height*np.cos(2*t)
            heading = np.arctan2(dy, dx)
            
            waypoints.append(Waypoint(x=x, y=y, theta=heading, index=i))
        
        logger.info(
            f"[PATH] Generated figure-8: {len(waypoints)} waypoints, "
            f"size={width}x{height}m"
        )
        
        return waypoints
    
    def generate_circle(
        self,
        center: Tuple[float, float],
        radius: float
    ) -> List[Waypoint]:
        """
        Generate circular path (special case of eclipse).
        
        Args:
            center: (cx, cy) center position
            radius: Circle radius
        
        Returns:
            List of waypoints
        """
        return self.generate_eclipse(
            center=center,
            semi_major=radius,
            semi_minor=radius,
            rotation=0.0
        )
    
    def generate_custom(
        self,
        center: Tuple[float, float],
        x_func,
        y_func,
        t_range: Tuple[float, float] = (0, 2*np.pi)
    ) -> List[Waypoint]:
        """
        Generate path from custom parametric functions.
        
        Args:
            center: (cx, cy) center position
            x_func: Function x(t) → float
            y_func: Function y(t) → float
            t_range: (t_min, t_max) parameter range
        
        Returns:
            List of waypoints
        
        Example:
            # Cardioid
            gen.generate_custom(
                center=(0, 0),
                x_func=lambda t: 2*r*(1 - cos(t))*cos(t),
                y_func=lambda t: 2*r*(1 - cos(t))*sin(t)
            )
        """
        cx, cy = center
        t_min, t_max = t_range
        
        waypoints = []
        
        t_values = np.linspace(t_min, t_max, self.resolution)
        
        for i, t in enumerate(t_values):
            x = cx + x_func(t)
            y = cy + y_func(t)
            
            # Numerical derivative for heading
            if i < len(t_values) - 1:
                dt = t_values[i+1] - t
                dx = x_func(t + dt) - x_func(t)
                dy = y_func(t + dt) - y_func(t)
                heading = np.arctan2(dy, dx)
            else:
                heading = waypoints[-1].theta
            
            waypoints.append(Waypoint(x=x, y=y, theta=heading, index=i))
        
        logger.info(
            f"[PATH] Generated custom path: {len(waypoints)} waypoints"
        )
        
        return waypoints
    
    def validate_path(self, waypoints: List[Waypoint]) -> bool:
        """
        Validate path feasibility.
        
        Checks:
        - No NaN/Inf coordinates
        - No duplicate consecutive points
        - Reasonable distances between waypoints
        
        Returns:
            True if valid
        """
        if not waypoints:
            logger.error("[PATH] Empty waypoint list")
            return False
        
        for i, wp in enumerate(waypoints):
            # Check for invalid values
            if not np.isfinite(wp.x) or not np.isfinite(wp.y):
                logger.error(f"[PATH] Invalid coordinates at waypoint {i}")
                return False
            
            # Check spacing
            if i > 0:
                prev = waypoints[i-1]
                dist = np.sqrt((wp.x - prev.x)**2 + (wp.y - prev.y)**2)
                
                if dist < 0.001:  # Too close
                    logger.warning(f"[PATH] Duplicate waypoint at {i}")
                
                if dist > 10.0:  # Too far
                    logger.error(f"[PATH] Excessive spacing at {i}: {dist:.2f}m")
                    return False
        
        logger.info(f"[PATH] Validation passed: {len(waypoints)} waypoints")
        return True
    
    def export_to_dict(self, waypoints: List[Waypoint]) -> List[Dict]:
        """
        Export waypoints to JSON-serializable format.
        
        Returns:
            List of dicts with x, y, theta, index
        """
        return [
            {
                'x': wp.x,
                'y': wp.y,
                'theta': wp.theta,
                'index': wp.index
            }
            for wp in waypoints
        ]