"""
Performance Logger Module
Structured logging utilities for robot control loop.
"""

import logging
from typing import Dict, Any, List

logger = logging.getLogger(__name__)


class PerformanceLogger:
    """Centralized logging for robot control operations."""
    
    @staticmethod
    def log_iteration_start(iteration: int) -> None:
        """Log iteration boundary."""
        logger.info(f"\n{'='*60}")
        logger.info(f"ITERATION {iteration}")
        logger.info(f"{'='*60}")
    
    @staticmethod
    def log_vision_analysis(vision_analysis: Dict[str, Any], obstacles: List[Dict]) -> None:
        """
        Log vision analysis results in structured format.
        """
        logger.info("Vision Analysis:")
        logger.info(f"  Safety Score: {vision_analysis.get('safety_score', 'N/A')}/10")
        logger.info(f"  Clear Paths: {vision_analysis.get('clear_paths', [])}")
        logger.info(f"  Recommended Direction: {vision_analysis.get('recommended_direction', 'N/A')}")
        
        if obstacles:
            logger.info(f"  Obstacles Detected: {len(obstacles)}")
            for i, obs in enumerate(obstacles[:3], 1):
                logger.info(
                    f"    {i}. {obs.get('type', 'Unknown')} at {obs.get('position', 'unknown')} "
                    f"(distance: ~{obs.get('distance_estimate', 'N/A')}m)"
                )
        else:
            logger.info("  Obstacles Detected: None")
    
    @staticmethod
    def log_navigation_decision(navigation_decision: Dict[str, Any]) -> None:
        """
        Log navigation decision details.
        """
        logger.info("\nNavigation Decision:")
        logger.info(f"  Action: {navigation_decision.get('action', 'N/A')}")
        logger.info(f"  Confidence: {navigation_decision.get('confidence', 0):.2f}")
        logger.info(f"  Reason: {navigation_decision.get('reason', 'N/A')}")
        
        params = navigation_decision.get("parameters", {})
        logger.info("  Parameters:")
        logger.info(f"    Linear velocity: {params.get('linear_velocity', 0):.3f} m/s")
        logger.info(f"    Angular velocity: {params.get('angular_velocity', 0):.3f} rad/s")
        logger.info(f"    Duration: {params.get('duration', 0):.2f} s")
    
    @staticmethod
    def log_mission_status(mission_type: str, detected_count: int, target_class: str = None) -> None:
        """
        Log mission-specific status.
        """
        if target_class:
            logger.info(f"[MISSION] Detected {detected_count} {target_class}(s)")
    
    @staticmethod
    def log_safety_override(command_type: str) -> None:
        """Log safety override actions."""
        logger.warning(f"[SAFETY VETO] Executing {command_type} command")
    
    @staticmethod
    def log_command_result(success: bool) -> None:
        """Log command execution result."""
        if success:
            logger.info("  ✓ Command executed successfully")
        else:
            logger.warning("  ✗ Command execution failed or aborted")
    
    @staticmethod
    def log_safety_abort(min_distance: float, action: str) -> None:
        """Log pre-execution safety abort."""
        logger.error(
            f"[PRE-EXECUTION ABORT] Obstacle at {min_distance:.2f}m! "
            f"Rejecting command: {action}"
        )
    
    @staticmethod
    def log_safety_warning(min_distance: float) -> None:
        """Log pre-execution safety warning."""
        logger.warning(
            f"[PRE-EXECUTION WARNING] Close obstacle at {min_distance:.2f}m, "
            f"reducing speed"
        )