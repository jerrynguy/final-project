"""
LiDAR Safety Monitor Module (Refactored - No Hysteresis)
Critical abort với state machine đơn giản và intelligent recovery.
"""

import time
import logging
import numpy as np
from enum import Enum
from typing import Dict, Optional, List, Tuple

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds

logger = logging.getLogger(__name__)


# ===== NEW: State Machine =====
class SafetyState(Enum):
    """Safety monitor states."""
    NORMAL = "normal"           # Hoạt động bình thường
    ABORT = "abort"             # Đang execute recovery
    ESCAPE_WAIT = "escape_wait" # Chờ robot thoát (3s pause)


class LidarSafetyMonitor:
    """
    Safety monitor với state machine đơn giản.
    
    Flow:
    NORMAL → (obstacle < 0.20m) → ABORT → ESCAPE_WAIT (3s) → NORMAL
    
    Features:
    - Loại bỏ hysteresis phức tạp
    - Stuck detection: 3 aborts tại cùng vị trí → force escape
    - Smart recovery: validate direction trước khi rotate/backup
    """
    
    def __init__(self):
        """Initialize monitor."""
        self.thresholds = SafetyThresholds
        
        # State machine
        self.state = SafetyState.NORMAL
        self.abort_count = 0
        
        # Escape tracking
        self.escape_start_time = 0.0
        self.escape_duration = 3.0  # 3 seconds escape window
        
        # Stuck detection
        self.last_abort_position = None
        self.consecutive_aborts_at_same_spot = 0
        
        # Movement tracking (for directional abort)
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
    
    def update_movement_state(self, linear_vel: float, angular_vel: float):
        """Track current movement for directional abort logic."""
        self.last_linear_velocity = linear_vel
        self.last_angular_velocity = angular_vel
    
    def _is_moving_forward(self) -> bool:
        """Check if robot is primarily moving forward (not turning)."""
        return (abs(self.last_linear_velocity) > 0.1 and 
                abs(self.last_angular_velocity) < 0.3)
    
    def _should_abort_for_obstacle(self, angle_deg: float, distance: float) -> bool:
        """
        Smart abort decision based on obstacle angle and movement direction.
        
        Rules:
        - Forward motion: chỉ abort nếu obstacle ở front arc (±45°)
        - Turning/stopped: abort nếu obstacle ở wider arc (±90°)
        """
        is_forward = self._is_moving_forward()
        
        if is_forward:
            # Moving forward: strict front check
            if abs(angle_deg) <= 45:
                critical_threshold = 0.22  # Front arc
            else:
                critical_threshold = 0.15  # Side (more lenient)
        else:
            # Turning: check wider arc
            if abs(angle_deg) <= 90:
                critical_threshold = 0.20
            else:
                critical_threshold = 0.15
        
        should_abort = distance < critical_threshold
        
        if should_abort:
            logger.debug(
                f"[ABORT CHECK] angle={angle_deg:.1f}°, dist={distance:.3f}m, "
                f"threshold={critical_threshold:.3f}m → ABORT"
            )
        
        return should_abort
    
    def check_critical_abort(
        self,
        lidar_data,
        robot_pos: Optional[Dict] = None
    ) -> Dict:
        """
        Main safety check với state machine.
        
        Returns:
            Dict with keys: abort, command, min_distance, state
        """
        if lidar_data is None:
            logger.error("[CRITICAL] No LIDAR data - ABORT")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0,
                'state': 'no_lidar'
            }
        
        try:
            current_time = time.time()
            obstacles = self._get_obstacles_with_angles(lidar_data)
            min_distance = self._get_min_distance(lidar_data)
            
            # ===== STATE MACHINE =====
            
            # STATE 1: ESCAPE_WAIT (đang thoát, không xử lý abort mới)
            if self.state == SafetyState.ESCAPE_WAIT:
                elapsed = current_time - self.escape_start_time
                
                # Check if escaped successfully
                if min_distance > self.thresholds.RESUME_SAFE:  # 0.5m
                    logger.info(
                        f"[ESCAPE SUCCESS] Distance: {min_distance:.2f}m "
                        f"after {elapsed:.1f}s → NORMAL"
                    )
                    self.state = SafetyState.NORMAL
                    return {'abort': False, 'command': None, 'state': 'normal'}
                
                # Check if escape timeout (stuck)
                if elapsed > self.escape_duration:
                    logger.error(
                        f"[ESCAPE TIMEOUT] Still at {min_distance:.2f}m "
                        f"after {elapsed:.1f}s → FORCE ESCAPE"
                    )
                    return self._force_escape_response(obstacles, robot_pos)
                
                # Still escaping - pause commands
                logger.debug(
                    f"[ESCAPING] {elapsed:.1f}s/{self.escape_duration:.1f}s, "
                    f"dist={min_distance:.2f}m"
                )
                return {
                    'abort': False,
                    'command': self._pause_command(),
                    'min_distance': min_distance,
                    'state': 'escape_wait'
                }
            
            # STATE 2: NORMAL - Check for new abort
            abort_obstacle = self._find_abort_obstacle(obstacles)
            
            if abort_obstacle:
                angle, distance = abort_obstacle
                
                # Check if stuck at same position
                if self._is_stuck_at_position(robot_pos):
                    logger.error("[STUCK DETECTED] Same position abort → FORCE ESCAPE")
                    return self._force_escape_response(obstacles, robot_pos)
                
                # Normal abort → execute recovery
                self.abort_count += 1
                self.state = SafetyState.ABORT
                
                logger.error(
                    f"[ABORT #{self.abort_count}] Obstacle at {angle:.1f}° "
                    f"({distance:.3f}m) → RECOVERY"
                )
                
                # Generate recovery command
                clearances = self._analyze_clearances(obstacles)
                recovery_cmd = self._smart_recovery_command(
                    angle, clearances, obstacles, robot_pos
                )
                
                # Transition to ESCAPE_WAIT
                self.state = SafetyState.ESCAPE_WAIT
                self.escape_start_time = current_time
                
                return {
                    'abort': True,
                    'command': recovery_cmd,
                    'min_distance': distance,
                    'obstacle_angle': angle,
                    'clearances': clearances,
                    'state': 'abort'
                }
            
            # No abort - continue normal
            return {
                'abort': False,
                'command': None,
                'min_distance': min_distance,
                'state': 'normal'
            }
            
        except Exception as e:
            logger.error(f"Safety check failed: {e}")
            return {
                'abort': True,
                'command': self._emergency_stop(),
                'min_distance': 0.0,
                'state': 'error'
            }
    
    def _find_abort_obstacle(
        self, 
        obstacles: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        """
        Tìm obstacle cần abort (gần nhất và thỏa điều kiện).
        
        Returns:
            (angle, distance) hoặc None
        """
        abort_obstacle = None
        min_abort_distance = float('inf')
        
        for angle_deg, distance in obstacles:
            if self._should_abort_for_obstacle(angle_deg, distance):
                if distance < min_abort_distance:
                    min_abort_distance = distance
                    abort_obstacle = (angle_deg, distance)
        
        return abort_obstacle
    
    def _is_stuck_at_position(self, robot_pos: Optional[Dict]) -> bool:
        """
        Detect if aborting at same position repeatedly.
        
        Logic: Nếu 3 lần abort gần nhau (< 0.1m) → STUCK
        """
        if robot_pos is None:
            return False
        
        current_pos = (robot_pos['x'], robot_pos['y'])
        
        # First abort at this mission
        if self.last_abort_position is None:
            self.last_abort_position = current_pos
            self.consecutive_aborts_at_same_spot = 1
            return False
        
        # Calculate distance from last abort
        dx = current_pos[0] - self.last_abort_position[0]
        dy = current_pos[1] - self.last_abort_position[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Same spot (< 10cm)
        if distance < 0.1:
            self.consecutive_aborts_at_same_spot += 1
            logger.warning(
                f"[STUCK CHECK] Abort #{self.consecutive_aborts_at_same_spot} "
                f"at same spot (distance={distance:.3f}m)"
            )
            
            # 3 aborts at same spot = STUCK
            if self.consecutive_aborts_at_same_spot >= 3:
                return True
        else:
            # Moved away - reset counter
            self.last_abort_position = current_pos
            self.consecutive_aborts_at_same_spot = 1
        
        return False
    
    def _force_escape_response(
        self,
        obstacles: List[Tuple[float, float]],
        robot_pos: Optional[Dict]
    ) -> Dict:
        """
        Force escape khi bị stuck (3 aborts tại cùng vị trí).
        
        Strategy: Tìm hướng OPEN nhất và đi thẳng 1.5m
        """
        clearances = self._analyze_clearances(obstacles)
        rear_clear = self._get_rear_clearance(obstacles)
        
        # Build clearance map (4 directions)
        directions = {
            'front': clearances.get('front', 0),
            'left': clearances.get('left', 0),
            'right': clearances.get('right', 0),
            'rear': rear_clear
        }
        
        # Find MAX clearance direction
        best_dir = max(directions, key=directions.get)
        best_clearance = directions[best_dir]
        
        logger.error(
            f"[FORCE ESCAPE] Best direction: {best_dir} ({best_clearance:.2f}m)\n"
            f"  Clearances: F:{directions['front']:.2f} "
            f"L:{directions['left']:.2f} R:{directions['right']:.2f} "
            f"Rear:{rear_clear:.2f}"
        )
        
        # Reset stuck counter
        self.consecutive_aborts_at_same_spot = 0
        self.last_abort_position = None
        
        # Transition back to ESCAPE_WAIT với extended duration
        self.state = SafetyState.ESCAPE_WAIT
        self.escape_start_time = time.time()
        self.escape_duration = 6.0  # Extended for force escape
        
        # Generate escape command (1.5m straight in best direction)
        if best_dir == 'front':
            return {
                'abort': True,
                'command': {
                    'action': 'force_escape_forward',
                    'parameters': {
                        'linear_velocity': 0.25,
                        'angular_velocity': 0.0,
                        'duration': 6.0  # 1.5m at 0.25m/s
                    },
                    'reason': 'force_escape_front',
                    'force_execute': True
                },
                'min_distance': best_clearance,
                'state': 'force_escape'
            }
        
        elif best_dir == 'rear':
            return {
                'abort': True,
                'command': {
                    'action': 'force_escape_backward',
                    'parameters': {
                        'linear_velocity': -0.25,
                        'angular_velocity': 0.0,
                        'duration': 6.0
                    },
                    'reason': 'force_escape_rear',
                    'force_execute': True
                },
                'min_distance': best_clearance,
                'state': 'force_escape'
            }
        
        elif best_dir == 'left':
            # Rotate left 90° + forward
            return {
                'abort': True,
                'command': {
                    'action': 'force_escape_left',
                    'parameters': {
                        'linear_velocity': 0.20,
                        'angular_velocity': 0.6,
                        'duration': 4.0
                    },
                    'reason': 'force_escape_left',
                    'force_execute': True
                },
                'min_distance': best_clearance,
                'state': 'force_escape'
            }
        
        else:  # right
            return {
                'abort': True,
                'command': {
                    'action': 'force_escape_right',
                    'parameters': {
                        'linear_velocity': 0.20,
                        'angular_velocity': -0.6,
                        'duration': 4.0
                    },
                    'reason': 'force_escape_right',
                    'force_execute': True
                },
                'min_distance': best_clearance,
                'state': 'force_escape'
            }

    def _get_obstacles_with_angles(self, lidar_data) -> List[Tuple[float, float]]:
        """Extract obstacles with their angles from LIDAR scan."""
        try:
            ranges = lidar_data.ranges
            angle_min = lidar_data.angle_min
            angle_increment = lidar_data.angle_increment
            
            obstacles = []
            
            for i, distance in enumerate(ranges):
                if np.isnan(distance) or np.isinf(distance):
                    continue
                
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = np.degrees(angle_rad)
                
                # Normalize to [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                obstacles.append((angle_deg, distance))
            
            obstacles.sort(key=lambda x: x[1])
            return obstacles
            
        except Exception as e:
            logger.error(f"Failed to extract obstacles: {e}")
            return []
    
    def _get_min_distance(self, lidar_data) -> float:
        """Extract minimum distance from LIDAR scan."""
        try:
            ranges = lidar_data.ranges
            valid_ranges = [
                r for r in ranges 
                if not (np.isnan(r) or np.isinf(r))
            ]
            
            if not valid_ranges:
                logger.warning("No valid LIDAR ranges")
                return float('inf')
            
            return min(valid_ranges)
            
        except Exception as e:
            logger.error(f"Failed to get min distance: {e}")
            return float('inf')
    
    def get_min_distance(self, lidar_data) -> float:
        """Public interface for minimum distance."""
        return self._get_min_distance(lidar_data)
    
    def _analyze_clearances(self, obstacles: List[Tuple[float, float]]) -> Dict:
        """
        Analyze clearance distances in each direction.
        """
        clearances = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }
        
        for angle_deg, distance in obstacles:
            abs_angle = abs(angle_deg)
            
            # Front: ±45°
            if abs_angle <= 45:
                clearances['front'] = min(clearances['front'], distance)
            
            # Left: 45-135°
            elif 45 < angle_deg <= 135:
                clearances['left'] = min(clearances['left'], distance)
            
            # Right: -135 to -45°
            elif -135 <= angle_deg < -45:
                clearances['right'] = min(clearances['right'], distance)
        
        # Cap at max sensor range
        for key in clearances:
            if clearances[key] == float('inf'):
                clearances[key] = 3.5  # Max LiDAR range
        
        return clearances
    
    def _get_rear_clearance(self, obstacles: List[Tuple[float, float]]) -> float:
        """
        Get minimum clearance in rear arc for safe backup validation.
        
        Rear arc: |angle| > 120°
        """
        rear_distances = [
            distance for angle_deg, distance in obstacles
            if abs(angle_deg) > 120
        ]
        
        if not rear_distances:
            return float('inf')
        
        min_rear = min(rear_distances)
        logger.debug(f"[REAR CHECK] Min rear clearance: {min_rear:.3f}m")
        return min_rear
    
    def _is_dead_end(self, clearances: Dict, rear_clear: float) -> bool:
        """
        Detect if robot is trapped in dead-end corner.
        
        Dead-end: ALL 4 sides blocked within critical distance.
        """
        DEAD_END_THRESHOLD = 0.18
        
        front = clearances.get('front', float('inf'))
        left = clearances.get('left', float('inf'))
        right = clearances.get('right', float('inf'))
        
        all_sides_blocked = (
            front < DEAD_END_THRESHOLD and
            left < DEAD_END_THRESHOLD and
            right < DEAD_END_THRESHOLD and
            rear_clear < DEAD_END_THRESHOLD
        )
        
        if all_sides_blocked:
            logger.error(
                f"[DEAD-END] F:{front:.2f} L:{left:.2f} "
                f"R:{right:.2f} Rear:{rear_clear:.2f} - ALL < {DEAD_END_THRESHOLD}m"
            )
            return True
        
        return False
    
    def _smart_recovery_command(
        self, 
        obstacle_angle: float, 
        clearances: Dict,
        obstacles_with_angles: List[Tuple[float, float]],
        robot_pos: Optional[Dict] = None
    ) -> Dict:
        """
        Generate smart recovery command with rear validation.
        """
        left_clear = clearances.get('left', 0)
        right_clear = clearances.get('right', 0)
        rear_clear = self._get_rear_clearance(obstacles_with_angles)
        
        # Check dead-end
        is_tight_corner = left_clear < 0.3 and right_clear < 0.3
        
        # STRATEGY 1: ROTATE-ONLY if rear blocked or tight corner
        if rear_clear < 0.25 or is_tight_corner:
            rotation_dir = self._choose_rotation_direction(
                left_clear, right_clear, obstacle_angle
            )
            
            logger.warning(
                f"[ROTATE-ONLY] Rear: {rear_clear:.2f}m, "
                f"L/R: {left_clear:.2f}/{right_clear:.2f}m → Rotate {rotation_dir}"
            )
            
            return {
                'action': 'rotate_recovery',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.8 if rotation_dir == 'left' else -0.8,
                    'duration': 1.5  # INCREASED from 0.8s
                },
                'reason': f'rotate_only_{rotation_dir}_rear_blocked'
            }
        
        # STRATEGY 2: SAFE BACKUP with adaptive duration
        backup_dir = self._choose_backup_direction(
            left_clear, right_clear, obstacle_angle
        )
        
        # Adaptive duration based on rear clearance
        adaptive_duration = min(1.0, rear_clear / 0.30)
        
        logger.info(
            f"[SAFE BACKUP] Rear: {rear_clear:.2f}m, "
            f"L/R: {left_clear:.2f}/{right_clear:.2f}m → "
            f"Backup {adaptive_duration:.2f}s + turn {backup_dir}"
        )
        
        return {
            'action': 'safe_backup_recovery',
            'parameters': {
                'linear_velocity': -0.18,  # REDUCED from -0.20
                'angular_velocity': 0.6 if backup_dir == 'left' else -0.6,
                'duration': adaptive_duration
            },
            'reason': f'safe_backup_{backup_dir}',
            'rear_clearance': rear_clear
        }
    
    def _choose_rotation_direction(
        self,
        left_clear: float,
        right_clear: float,
        obstacle_angle: float
    ) -> str:
        """
        Choose rotation direction INTELLIGENTLY.
        
        Rules:
        1. If one side MUCH clearer (>0.2m diff) → rotate to that side
        2. Else → rotate AWAY from obstacle
        """
        CLEAR_DIFF_THRESHOLD = 0.2
        
        if left_clear > right_clear + CLEAR_DIFF_THRESHOLD:
            return 'left'
        elif right_clear > left_clear + CLEAR_DIFF_THRESHOLD:
            return 'right'
        else:
            # Rotate away from obstacle
            return 'right' if obstacle_angle > 0 else 'left'
    
    def _choose_backup_direction(
        self,
        left_clear: float,
        right_clear: float,
        obstacle_angle: float
    ) -> str:
        """Choose backup turn direction (same logic as rotation)."""
        return self._choose_rotation_direction(left_clear, right_clear, obstacle_angle)
    
    def _emergency_stop(self) -> Dict:
        """Generate emergency stop command."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.1
            },
            'reason': 'emergency_stop'
        }
    
    def _pause_command(self) -> Dict:
        """Generate pause command during ESCAPE_WAIT."""
        return {
            'action': 'stop',
            'parameters': {
                'linear_velocity': 0.0,
                'angular_velocity': 0.0,
                'duration': 0.3
            },
            'reason': 'escape_wait_pause'
        }
    
    def get_stats(self) -> Dict:
        """Get safety monitor statistics."""
        return {
            'total_aborts': self.abort_count,
            'current_state': self.state.value,
            'consecutive_same_spot_aborts': self.consecutive_aborts_at_same_spot,
            'movement_state': 'forward' if self._is_moving_forward() else 'turning'
        }
    
    def reset_stats(self):
        """Reset statistics (for testing)."""
        self.abort_count = 0
        self.last_abort_position = None
        self.consecutive_aborts_at_same_spot = 0
        self.state = SafetyState.NORMAL
        logger.info("[STATS RESET] Safety monitor cleared")