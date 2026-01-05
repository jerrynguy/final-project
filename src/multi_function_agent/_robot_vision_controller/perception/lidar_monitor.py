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

    def _generate_escape_command(
        self,
        action_type: str,
        target_sector: int,
        clearance: float
    ) -> Dict:
        """
        Generate precise movement command for escape maneuver.
        """
        if action_type == 'none':
            return self._emergency_stop()
        
        # ===== FORWARD MOVEMENT =====
        if action_type == 'forward':
            # Duration: travel 60% of clearance, max 3s
            duration = min(3.0, (clearance * 0.6) / 0.20)
            
            return {
                'action': 'escape_forward',
                'parameters': {
                    'linear_velocity': 0.20,
                    'angular_velocity': 0.0,
                    'duration': duration
                },
                'reason': f'escape_forward_{clearance:.2f}m'
            }
        
        # ===== GENTLE TURNS (30° sectors) =====
        elif action_type == 'turn_right':
            # Turn right while moving forward
            # Angular: 0.4 rad/s → ~23°/s → 1.3s for 30°
            return {
                'action': 'escape_turn_right',
                'parameters': {
                    'linear_velocity': 0.15,
                    'angular_velocity': -0.4,
                    'duration': 2.0
                },
                'reason': f'escape_turn_right_{target_sector}deg'
            }
        
        elif action_type == 'turn_left':
            return {
                'action': 'escape_turn_left',
                'parameters': {
                    'linear_velocity': 0.15,
                    'angular_velocity': 0.4,
                    'duration': 2.0
                },
                'reason': f'escape_turn_left_{target_sector}deg'
            }
        
        # ===== SHARP ROTATIONS (90-120° sectors) =====
        elif action_type == 'rotate_right':
            # In-place rotation right
            # Angular: 0.7 rad/s → ~40°/s → 2.5s for 90°
            return {
                'action': 'escape_rotate_right',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': -0.7,
                    'duration': 2.0
                },
                'reason': f'escape_rotate_right_{target_sector}deg'
            }
        
        elif action_type == 'rotate_left':
            return {
                'action': 'escape_rotate_left',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.7,
                    'duration': 2.0
                },
                'reason': f'escape_rotate_left_{target_sector}deg'
            }
        
        # ===== BACKUP (180° sector) =====
        elif action_type == 'backup':
            # CAREFUL: Only backup with validated clearance
            # Duration: travel 50% of clearance, max 2s
            duration = min(2.0, (clearance * 0.5) / 0.15)
            
            return {
                'action': 'escape_backup',
                'parameters': {
                    'linear_velocity': -0.15,
                    'angular_velocity': 0.0,
                    'duration': duration
                },
                'reason': f'escape_backup_{clearance:.2f}m'
            }
        
        else:
            logger.error(f"[ESCAPE CMD] Unknown action type: {action_type}")
            return self._emergency_stop()

    def _analyze_360_clearances(
        self, 
        obstacles: List[Tuple[float, float]]
    ) -> Dict[int, float]:
        """
        Analyze clearance for 12 sectors (30° each).
        
        Returns complete 360° clearance map for intelligent escape planning.
        """
        SECTOR_SIZE = 30  # degrees
        NUM_SECTORS = 12
        MAX_LIDAR_RANGE = 3.5
        
        # Initialize all sectors with max range
        sector_clearances = {
            sector * SECTOR_SIZE: MAX_LIDAR_RANGE
            for sector in range(NUM_SECTORS)
        }
        
        # Map obstacles to sectors
        for angle_deg, distance in obstacles:
            # Normalize angle to [0, 360)
            normalized_angle = angle_deg % 360
            
            # Find corresponding sector (0, 30, 60, ..., 330)
            sector_index = int(normalized_angle // SECTOR_SIZE)
            sector_angle = sector_index * SECTOR_SIZE
            
            # Update sector with minimum distance
            current_min = sector_clearances[sector_angle]
            sector_clearances[sector_angle] = min(current_min, distance)
        
        return sector_clearances

    def _select_escape_direction(
        self,
        sector_clearances: Dict[int, float],
        current_heading_deg: float,
        obstacle_angle_deg: float
    ) -> Tuple[str, int, float]:
        """
        Select BEST escape direction using 360° clearance analysis.
        
        Strategy:
        1. Filter sectors with clearance > 0.30m (safe threshold)
        2. Score by: clearance (40%) + obstacle avoidance (30%) + 
                    opposite direction bonus (20%) + forward bias (10%)
        3. Select highest-scoring sector
        4. Map sector to action type (forward/turn/rotate/backup)
        """
        # --- CONFIGURATION ---
        SAFE_THRESHOLD = 0.30      # Minimum clearance (meters)
        MAX_LIDAR_RANGE = 3.5      # Used for normalization
        
        # Weights (Must sum to ~1.0)
        W_CLEARANCE = 0.35         # Safety is #1 priority
        W_FORWARD   = 0.25         # Aggressive forward bias (Claude's suggestion)
        W_OBSTACLE  = 0.25         # Specific obstacle avoidance
        W_OPPOSITE  = 0.15         # Escape from trap logic
            
        # Step 1: Filter safe sectors
        safe_sectors = {
            sec: clr for sec, clr in sector_clearances.items()
            if clr > SAFE_THRESHOLD
        }
        
        if not safe_sectors:
            logger.error(f"[ESCAPE] BLOCKED! All clearances < {SAFE_THRESHOLD}m")
            return ('none', 0, 0.0)
        
        # Step 2: Calculate opposite direction from obstacle
        normalized_obstacle = obstacle_angle_deg % 360
        opposite_angle = (normalized_obstacle + 180) % 360
        scored_sectors = []

        # Step 3: Score each safe sector
        for sector, clearance in safe_sectors.items():
            # Factor 1: Clearance score (35% weight)
            # Normalize to [0, 1] based on max LiDAR range
            clearance_score = min(clearance / MAX_LIDAR_RANGE, 1.0)
            
            # Factor 2: Obstacle avoidance score (25% weight)
            # Higher angular distance from obstacle = better
            angle_diff = min(
                abs(sector - normalized_obstacle),
                360 - abs(sector - normalized_obstacle)
            )
            obstacle_avoidance_score = angle_diff / 180.0
            
            # Factor 3: Opposite direction bonus (25% weight)
            # Sectors near 180° from obstacle get bonus
            angle_to_opposite = min(
                abs(sector - opposite_angle),
                360 - abs(sector - opposite_angle)
            )
            opposite_bonus = 1.0 if angle_to_opposite < 60 else 0.0
            
            # Factor 4: Forward bias (15% weight)
            # Prefer forward-facing sectors for natural movement
            if sector == 0:
                forward_bias = 1.0  # Strong preference for straight ahead
            elif sector in [330, 30]:
                forward_bias = 0.85  # Slight turns forward
            elif sector in [60, 300]:
                forward_bias = 0.6  # Moderate turns
            elif sector in [90, 270]:
                forward_bias = 0.5  # Side movements
            elif sector in [120, 240]:
                forward_bias = 0.3  # Rear-side movements
            else:  # 150, 180, 210 (rear hemisphere)
                forward_bias = 0.0  # Reverse movements (least preferred)
            
            # Weighted final score
            total_score = (
                clearance_score * W_CLEARANCE +
                obstacle_avoidance_score * W_OBSTACLE +
                opposite_bonus * W_OPPOSITE +
                forward_bias * W_FORWARD
            )
            
            scored_sectors.append((sector, clearance, total_score))
        
        # Step 4: Sort & Select
        # Sort by score (descending)
        scored_sectors.sort(key=lambda x: x[2], reverse=True)
        
        # Select best sector
        best_sector, best_clearance, best_score = scored_sectors[0]
        
        # Log top 3 candidates for debugging
        logger.info("[ESCAPE SELECTION] Top 3 candidates:")
        for i, (sector, clearance, score) in enumerate(scored_sectors[:3], 1):
            logger.info(
                f"  {i}. Sector {sector:3d}°: clearance={clearance:.2f}m, "
                f"score={score:.2f}"
            )
        
        logger.info(
            f"[ESCAPE SELECTION] SELECTED: {best_sector}° "
            f"(clearance: {best_clearance:.2f}m, score: {best_score:.2f})"
        )
        
        # Step 5: Map sector to action type
        # Front hemisphere (0° ± 60°)
        sec = best_sector % 360
        
        if (sec >= 350) or (sec <= 10):
            action = 'forward'
        elif 10 < sec <= 60:
            action = 'turn_right'
        elif 300 <= sec < 350:
            action = 'turn_left'
        elif 60 < sec <= 150:
            action = 'rotate_right'
        elif 210 <= sec < 300:
            action = 'rotate_left'
        else:
            # Khu vực phía sau (150 - 210)
            action = 'backup'
        
        return (action, best_sector, best_clearance)

    def _force_escape_response(
        self,
        obstacles: List[Tuple[float, float]],
        robot_pos: Optional[Dict]
    ) -> Dict:
        """
        Force escape using intelligent 360° clearance analysis.
        
        NEW STRATEGY:
        1. Analyze all 12 sectors (30° each) for complete clearance map
        2. Select BEST escape direction using weighted scoring
        3. If NO safe direction (all < 0.30m) → Try rotate in-place
        4. If TRUE DEADLOCK (all < 0.20m) → Emergency stop + Nav2 rescue
        
        This replaces the old left/right clearance comparison with
        comprehensive spatial awareness.
        """
        logger.error("[FORCE ESCAPE] Starting 360° analysis...")
        
        # Step 1: Analyze complete 360° clearance map
        sector_clearances = self._analyze_360_clearances(obstacles)
        
        # Log clearances for debugging
        logger.error("[360° CLEARANCES]")
        for sector in sorted(sector_clearances.keys()):
            clearance = sector_clearances[sector]
            status = "✓" if clearance > 0.30 else "⚠" if clearance > 0.20 else "✗"
            logger.error(f"  {status} {sector:3d}°: {clearance:.2f}m")
        
        # Step 2: Get obstacle angle for avoidance scoring
        obstacle_angle = obstacles[0][0] if obstacles else 0.0
        logger.error(f"[OBSTACLE] Primary at {obstacle_angle:.1f}°")
        
        # Step 3: Get robot heading (for forward bias calculation)
        robot_heading = 0.0
        if robot_pos:
            robot_heading = np.degrees(robot_pos.get('theta', 0.0)) % 360
            logger.error(f"[ROBOT HEADING] {robot_heading:.1f}°")
        
        # Step 4: Select best escape direction using intelligent scoring
        action_type, target_sector, clearance = self._select_escape_direction(
            sector_clearances,
            robot_heading,
            obstacle_angle
        )
        
        # ✅ CASE 1: Found safe direction (clearance > 0.30m)
        if action_type != 'none':
            logger.error(
                f"[FORCE ESCAPE] ✓ Safe direction found: {action_type} "
                f"(sector: {target_sector}°, clearance: {clearance:.2f}m)"
            )
            
            # Reset stuck detection
            self.consecutive_aborts_at_same_spot = 0
            self.last_abort_position = None
            
            # Transition to ESCAPE_WAIT state
            self.state = SafetyState.ESCAPE_WAIT
            self.escape_start_time = time.time()
            self.escape_duration = 6.0  # Full 6s for escape
            
            # Generate movement command
            escape_cmd = self._generate_escape_command(
                action_type, target_sector, clearance
            )
            escape_cmd['force_execute'] = True  # Bypass safety checks
            
            return {
                'abort': True,
                'command': escape_cmd,
                'min_distance': clearance,
                'state': 'force_escape',
                'clearances': sector_clearances  # For main loop
            }
        
        # ✅ CASE 2: No safe linear movement → Try rotate in-place
        logger.error("[DEADLOCK] No direction with clearance > 0.30m")
        
        # Find sector with MAXIMUM clearance (even if < 0.30m)
        max_sector = max(sector_clearances, key=sector_clearances.get)
        max_clearance = sector_clearances[max_sector]
        
        logger.error(
            f"[DEADLOCK] Max clearance: {max_clearance:.2f}m at {max_sector}°"
        )
        
        # If ANY sector has clearance > 0.20m → Rotate toward it
        if max_clearance > 0.20:
            logger.warning(
                f"[ROTATE IN-PLACE] Attempting rotation toward {max_sector}° "
                f"(clearance: {max_clearance:.2f}m)"
            )
            
            # Determine rotation direction (shortest path to target)
            if max_sector <= 180:
                angular_vel = 0.6  # Rotate left (CCW)
                direction = 'left'
            else:
                angular_vel = -0.6  # Rotate right (CW)
                direction = 'right'
            
            # Shorter escape window for rotation
            self.state = SafetyState.ESCAPE_WAIT
            self.escape_start_time = time.time()
            self.escape_duration = 3.0  # 3s for in-place rotation
            
            return {
                'abort': True,
                'command': {
                    'action': f'deadlock_rotate_{direction}',
                    'parameters': {
                        'linear_velocity': 0.0,
                        'angular_velocity': angular_vel,
                        'duration': 2.0
                    },
                    'reason': f'deadlock_rotate_{direction}_{max_sector}deg',
                    'force_execute': True
                },
                'min_distance': max_clearance,
                'state': 'deadlock_rotate',
                'clearances': sector_clearances
            }
        
        # ✅ CASE 3: TRUE DEADLOCK → Emergency stop + Nav2 rescue
        logger.error(
            f"[TRUE DEADLOCK] All sectors < 0.20m! Cannot escape locally."
        )
        logger.error("Requesting Nav2 global planner rescue...")
        
        # Reset state machine
        self.state = SafetyState.NORMAL
        
        return {
            'abort': True,
            'command': self._emergency_stop(),
            'min_distance': max_clearance,
            'state': 'deadlock_nav2_rescue_needed',
            'request_nav2_rescue': True,  # Flag for main loop
            'clearances': sector_clearances
        }
    
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

    def should_trigger_nav2_rescue(self) -> bool:
        """
        Check nếu nên trigger Nav2 rescue.
        
        Điều kiện: Force escape thất bại (vẫn ở ESCAPE_WAIT sau >5s)
        
        Returns:
            True nếu cần Nav2 rescue, False nếu không
        """
        if self.state != SafetyState.ESCAPE_WAIT:
            return False
        
        current_time = time.time()
        elapsed = current_time - self.escape_start_time
        
        # Force escape thất bại nếu:
        # 1. Đã >5s trong ESCAPE_WAIT
        # 2. Consecutive aborts >= 3 (đã trigger force escape)
        if elapsed > 5.0 and self.consecutive_aborts_at_same_spot >= 3:
            logger.error(
                f"[FORCE ESCAPE FAILED] Still in ESCAPE_WAIT after {elapsed:.1f}s "
                f"(consecutive aborts: {self.consecutive_aborts_at_same_spot}) "
                f"→ Nav2 rescue needed"
            )
            return True
        
        return False

    def reset_after_nav2_rescue(self):
        """
        Reset state sau khi Nav2 rescue thành công.
        """
        self.state = SafetyState.NORMAL
        self.consecutive_aborts_at_same_spot = 0
        self.last_abort_position = None
        self.escape_start_time = 0.0
        self.abort_count = 0
        
        logger.info("[RESET] Safety monitor reset after Nav2 rescue")

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
            
            # STATE 1: ESCAPE_WAIT
            if self.state == SafetyState.ESCAPE_WAIT:
                elapsed = current_time - self.escape_start_time
                
                # CHECK SUCCESS FREQUENTLY (every call, not just timeout)
                if min_distance > self.thresholds.RESUME_SAFE:  # 0.5m
                    logger.info(
                        f"[ESCAPE SUCCESS] ✓ Cleared to {min_distance:.2f}m "
                        f"after {elapsed:.1f}s → NORMAL"
                    )
                    self.state = SafetyState.NORMAL
                    
                    # Reset rotate attempt flag
                    if hasattr(self, '_rotate_attempted'):
                        self._rotate_attempted = False
                    
                    return {
                        'abort': False,
                        'command': None,
                        'min_distance': min_distance,
                        'state': 'normal'
                    }
                
                # TIMEOUT HANDLING (keep 6s, check every iteration)
                if elapsed > self.escape_duration:
                    logger.error(
                        f"[ESCAPE TIMEOUT] ✗ Failed after {elapsed:.1f}s "
                        f"(still at {min_distance:.2f}m)"
                    )
                    
                    # Analyze current situation
                    obstacles = self._get_obstacles_with_angles(lidar_data)
                    sector_clearances = self._analyze_360_clearances(obstacles)
                    
                    max_sector = max(sector_clearances, key=sector_clearances.get)
                    max_clearance = sector_clearances[max_sector]
                    
                    # ✅ TRY 1: Rotate in-place rescue (if not attempted yet)
                    if max_clearance > 0.20 and not hasattr(self, '_rotate_attempted'):
                        logger.warning(
                            f"[TIMEOUT RESCUE] Attempting rotate toward {max_sector}° "
                            f"(clearance: {max_clearance:.2f}m)"
                        )
                        
                        self._rotate_attempted = True
                        
                        # Determine rotation direction
                        angular_vel = 0.6 if max_sector <= 180 else -0.6
                        direction = 'left' if max_sector <= 180 else 'right'
                        
                        # Extend escape window for rescue attempt
                        self.escape_start_time = current_time  # Reset timer
                        self.escape_duration = 3.0
                        
                        return {
                            'abort': True,
                            'command': {
                                'action': f'timeout_rotate_rescue_{direction}',
                                'parameters': {
                                    'linear_velocity': 0.0,
                                    'angular_velocity': angular_vel,
                                    'duration': 2.0
                                },
                                'reason': f'escape_timeout_rotate_rescue_{max_sector}deg',
                                'force_execute': True
                            },
                            'min_distance': max_clearance,
                            'state': 'timeout_rotate_rescue',
                            'clearances': sector_clearances
                        }
                    
                    # ✅ TRY 2: Nav2 rescue (if rotate attempted OR no clearance)
                    logger.error(
                        f"[TIMEOUT] Cannot escape locally "
                        f"(max clearance: {max_clearance:.2f}m at {max_sector}°)"
                    )
                    logger.error("Requesting Nav2 global planner...")
                    
                    # Reset flags and state
                    if hasattr(self, '_rotate_attempted'):
                        self._rotate_attempted = False
                    self.state = SafetyState.NORMAL
                    
                    return {
                        'abort': True,
                        'command': self._emergency_stop(),
                        'min_distance': min_distance,
                        'state': 'escape_timeout_nav2_needed',
                        'request_nav2_rescue': True,
                        'clearances': sector_clearances
                    }
                
                # ✅ Still escaping - pause commands
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
                
                # Generate recovery command using 360° analysis
                recovery_cmd = self._force_escape_response(obstacles, robot_pos)
                
                return recovery_cmd
            
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