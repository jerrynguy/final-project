"""
LiDAR Safety Monitor Module
Real-time collision avoidance system using LiDAR sensor data.
"""

import asyncio
import logging
import numpy as np
from typing import Dict, Optional

from multi_function_agent.robot_vision_controller.utils.safety_checks import SafetyValidator

logger = logging.getLogger(__name__)


# =============================================================================
# LiDAR Safety Monitor
# =============================================================================

class LidarSafetyMonitor:
    """
    Real-time collision avoidance using raw LiDAR data.
    """
    
    def __init__(self):
        """Initialize LiDAR safety monitor with safety thresholds."""
        self.safety_validator = SafetyValidator()
        self.CRITICAL_DISTANCE = self.safety_validator.CRITICAL_DISTANCE
        self.WARNING_DISTANCE = self.safety_validator.WARNING_DISTANCE
    
    def check_safety(self, lidar_data) -> Dict:
        """
        Check LiDAR data for immediate collision threats.
        """
        if lidar_data is None:
            logger.error("[SAFETY CRITICAL] No LIDAR data - FORCING STOP!")
            return {
                'veto': True,  # ← FORCE VETO!
                'command': self._emergency_stop(),
                'reason': 'no_lidar_critical'
            }
        
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            # Find closest obstacle and its angular position
            min_distance = float('inf')
            threat_angle = 0.0
            
            for i, distance in enumerate(ranges):
                # Skip invalid readings
                if np.isnan(distance) or np.isinf(distance):
                    continue

                # Only check front hemisphere (±90°)
                angle = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle)
                if not (-90 <= angle_deg <= 90):
                    continue
                
                # Track closest obstacle
                if distance < min_distance:
                    min_distance = distance
                    threat_angle = angle_min + (i * angle_increment)
            
            # CRITICAL: Immediate collision risk
            if min_distance < self.CRITICAL_DISTANCE:
                escape_command = self._generate_escape_command(threat_angle, min_distance)
                logger.warning(f"[SAFETY VETO] Collision risk at {min_distance:.2f}m")
                return {
                    'veto': True,
                    'command': escape_command,
                    'reason': f'collision_risk_{min_distance:.2f}m'
                }
            
            # WARNING: Close but not critical
            if min_distance < self.WARNING_DISTANCE:
                logger.debug(f"[SAFETY WARNING] Close obstacle at {min_distance:.2f}m")
            
            return {'veto': False, 'command': None, 'reason': 'safe'}
            
        except Exception as e:
            logger.error(f"Safety check failed: {e}")
            # Safe fallback: emergency stop on error
            return {
                'veto': True,
                'command': self._emergency_stop(),
                'reason': 'safety_check_error'
            }
    
    def _generate_escape_command(self, threat_angle: float, distance: float) -> Dict:
        """Generate emergency escape command based on threat direction."""
        
        # CRITICAL: If extremely close, STOP FIRST
        if distance < self.CRITICAL_DISTANCE:
            logger.error(f"[EMERGENCY STOP] Obstacle at {distance:.2f}m!")
            return {
                'action': 'stop',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.0,
                    'duration': 0.2
                },
                'confidence': 1.0,
                'reason': f'emergency_stop_{distance:.2f}m'
            }
        
        angle_deg = np.degrees(threat_angle)
        
        # Threat in front (-45° to +45°): AGGRESSIVE backup
        if -45 <= angle_deg <= 45:
            return {
                'action': 'move_backward',
                'parameters': {
                    'linear_velocity': -0.4,   # Fast backup
                    'angular_velocity': 0.0,   # Straight back
                    'duration': 0.5
                },
                'confidence': 1.0,
                'reason': f'emergency_backup_{distance:.2f}m'
            }
        
        # Threat on left side (45° to 135°): Rotate RIGHT (no forward movement)
        elif 45 < angle_deg <= 135:
            return {
                'action': 'rotate_right',
                'parameters': {
                    'linear_velocity': 0.0,     # ← FIX: Đã là 0.0, không di chuyển
                    'angular_velocity': -1.5,   # Fast rotate away
                    'duration': 0.3
                },
                'confidence': 1.0,
                'reason': 'left_threat_rotate_right'
            }
        
        # Threat on right side (-45° to -135°): Rotate LEFT (no forward movement)
        elif -135 <= angle_deg < -45:
            return {
                'action': 'rotate_left',
                'parameters': {
                    'linear_velocity': 0.0,     # ← FIX: Đã là 0.0, không di chuyển
                    'angular_velocity': 1.5,    # Fast rotate away
                    'duration': 0.3
                },
                'confidence': 1.0,
                'reason': 'right_threat_rotate_left'
            }
        
        # Threat behind (should not happen in front-only check): backup anyway
        else:
            return {
                'action': 'move_backward',
                'parameters': {
                    'linear_velocity': -0.3,
                    'angular_velocity': 0.0,
                    'duration': 0.3
                },
                'confidence': 0.8,
                'reason': 'rear_threat_backup'
            }
    
    def _emergency_stop(self) -> Dict:
        """
        Generate emergency stop command.
        """
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'confidence': 1.0,
            'reason': 'emergency_stop'
        }
    
    def get_min_distance(self, lidar_data) -> float:
        """
        Get minimum distance from LIDAR data (for continuous monitoring).
        Returns inf if no valid data.
        """
        if lidar_data is None:
            return float('inf')
        
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            min_distance = float('inf')
            
            for i, distance in enumerate(ranges):
                if np.isnan(distance) or np.isinf(distance):
                    continue
                
                # Only check front hemisphere (±9d0°)
                angle = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle)
                if not (-90 <= angle_deg <= 90):
                    continue
                
                if distance < min_distance:
                    min_distance = distance
            
            return min_distance
            
        except Exception as e:
            logger.error(f"Get min distance failed: {e}")
            return float('inf')
        
    # Unified safety override handler
    async def handle_safety_override(
        self,
        lidar_data,
        robot_interface,
        results: Dict
    ) -> bool:
        """
        Check safety and execute escape if needed.
        """
        safety_override = self.check_safety(lidar_data)
        
        if safety_override['veto']:
            logger.warning("[SAFETY VETO] Executing escape command")
            
            # Execute escape command (BLOCKING until complete)
            success = await robot_interface.execute_command(safety_override['command'])
            
            if success:
                results["commands_sent"].append(safety_override['command'])
            
            # Wait for escape to complete before next iteration
            escape_duration = safety_override['command'].get('parameters', {}).get('duration', 0.2)
            await asyncio.sleep(escape_duration)
            
            return True  # Signal to skip to next iteration
        
        return False  # Safe to continue