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
    COOLDOWN = "cooldown"


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
        self.escape_duration = 8.0  # 8 seconds escape window

        # Cooldown tracking
        self.cooldown_start_time = 0.0
        self.cooldown_duration = 5.0  # ← 5 giây pause buffer 
        
        # Stuck detection
        self.last_abort_position = None
        self.consecutive_aborts_at_same_spot = 0
        
        # Movement tracking (for directional abort)
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0

        self.last_escape_sector = None      # Last escape direction (0-330)
        self.last_escape_time = 0.0         # Timestamp of escape
        self.directional_cooldown = 4.0     # Cooldown duration (seconds)
    
    def update_movement_state(self, linear_vel: float, angular_vel: float):
        """Track current movement for directional abort logic."""
        self.last_linear_velocity = linear_vel
        self.last_angular_velocity = angular_vel
    
    def _is_moving_forward(self) -> bool:
        """Check if robot is primarily moving forward (not turning)."""
        return (abs(self.last_linear_velocity) > 0.1 and 
                abs(self.last_angular_velocity) < 0.6)

    def _should_abort_for_obstacle(self, angle_deg: float, distance: float) -> bool:
        """
        DIRECTIONAL ABORT: Chỉ abort obstacles trên trajectory di chuyển.
        
        NEW LOGIC:
        - Pure forward: check front ±45° only
        - Turning LEFT: check left hemisphere (0° to 180°) with directional thresholds
        - Turning RIGHT: check right hemisphere (0° to -180°) with directional thresholds
        - Stopped: check wider ±90°
        """
        # Determine movement type
        is_forward = self._is_moving_forward()
        is_turning_left = self.last_angular_velocity > 0.3
        is_turning_right = self.last_angular_velocity < -0.3
        
        # ✅ RULE 1: Pure forward movement - ignore sides
        if is_forward and not (is_turning_left or is_turning_right):
            if abs(angle_deg) <= 45:  # Front arc only
                should_abort = distance < SafetyThresholds.CRITICAL_ABORT_FRONT
            else:
                should_abort = False  # Ignore side obstacles when going straight
        
        # ✅ RULE 2: Turning LEFT - check left hemisphere with directional thresholds
        elif is_turning_left:
            if 0 <= angle_deg <= 180:  # Left hemisphere
                # ✅ NEW: Use directional thresholds
                if abs(angle_deg) <= 60:  # Front-left arc (0° to 60°)
                    threshold = SafetyThresholds.CRITICAL_ABORT_FRONT  # 0.22m (strict)
                else:  # Side-left arc (60° to 180°)
                    threshold = SafetyThresholds.CRITICAL_ABORT_SIDE   # 0.15m (lenient)
                
                should_abort = distance < threshold
            else:
                should_abort = False  # Ignore right hemisphere when turning left
        
        # ✅ RULE 3: Turning RIGHT - check right hemisphere with directional thresholds
        elif is_turning_right:
            if -180 <= angle_deg <= 0:  # Right hemisphere
                # ✅ NEW: Use directional thresholds (symmetric with left)
                if abs(angle_deg) <= 60:  # Front-right arc (-60° to 0°)
                    threshold = SafetyThresholds.CRITICAL_ABORT_FRONT  # 0.22m (strict)
                else:  # Side-right arc (-180° to -60°)
                    threshold = SafetyThresholds.CRITICAL_ABORT_SIDE   # 0.15m (lenient)
                
                should_abort = distance < threshold
            else:
                should_abort = False  # Ignore left hemisphere when turning right
        
        # ✅ RULE 4: Stopped/slow - check wider arc
        else:
            should_abort = (distance < SafetyThresholds.CRITICAL_ABORT and abs(angle_deg) <= 90)
        
        if should_abort:
            logger.debug(
                f"[ABORT CHECK] angle={angle_deg:.1f}°, dist={distance:.3f}m, "
                f"movement={'forward' if is_forward else 'turning_left' if is_turning_left else 'turning_right' if is_turning_right else 'stopped'} → ABORT"
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
        #  TĂNG: linear speed 0.30 → 0.35 m/s
        #  TĂNG: duration multiplier 0.84 → 1.0
            duration = min(4.0, (clearance * 1.0) / 0.35)  # Was: min(3.0, clearance*0.7/0.25)
            
            return {
                'action': 'escape_forward',
                'parameters': {
                    'linear_velocity': 0.35,  
                    'angular_velocity': 0.0,
                    'duration': duration
                },
                'reason': f'escape_forward_{clearance:.2f}m'
            }

        elif action_type == 'turn_right':
            return {
                'action': 'escape_turn_right',
                'parameters': {
                    'linear_velocity': 0.30,  
                    'angular_velocity': -0.5,
                    'duration': 3.5
                },
                'reason': f'escape_turn_right_{target_sector}deg'
            }

        elif action_type == 'turn_left':
            return {
                'action': 'escape_turn_left',
                'parameters': {
                    'linear_velocity': 0.30,  
                    'angular_velocity': 0.5,
                    'duration': 3.5  
                },
                'reason': f'escape_turn_left_{target_sector}deg'
            }

        # (For rotate_right, rotate_left: keep linear=0, only increase duration)
        elif action_type == 'rotate_right':
            return {
                'action': 'escape_rotate_right',
                'parameters': {
                    'linear_velocity': 0.15,
                    'angular_velocity': -0.8,
                    'duration': 3.5  
                },
                'reason': f'escape_rotate_right_{target_sector}deg'
            }
        
        elif action_type == 'rotate_left':
            return {
                'action': 'escape_rotate_left',
                'parameters': {
                    'linear_velocity': 0.15,
                    'angular_velocity': 0.8,  
                    'duration': 3.5  
                },
                'confidence': 0.9,
                'reason': f'escape_rotate_left_{target_sector}deg'
            }

        elif action_type == 'backup':
            duration = min(6.0, (clearance * 1.5) / 0.35)
            angular_bias = 0.2 if clearance > 1.0 else 0.0
            
            return {
                'action': 'escape_backup',
                'parameters': {
                    'linear_velocity': -0.30,  
                    'angular_velocity': angular_bias,
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
        Select best escape direction using centralized thresholds.
        
        Updates:
        - ESCAPE_SAFE_THRESHOLD = 0.35m (was 0.30, accounts for robot width + noise)
        - OBSTACLE_REJECTION_ARC = 45° (was 60°, tighter for differential drive)
        """
        SAFE_THRESHOLD = SafetyThresholds.ESCAPE_SAFE_THRESHOLD  
        OBSTACLE_REJECTION_ARC = SafetyThresholds.OBSTACLE_REJECTION_ARC  
        
        # Weight configuration (unchanged, but now documented)
        W_CLEARANCE = 0.25  # Clearance distance (dominant)
        W_OBSTACLE  = 0.50  # Obstacle avoidance angle
        W_OPPOSITE  = 0.20  # Opposite direction bonus
        W_FORWARD   = 0.05  # Forward bias
        
        # Filter safe sectors
        safe_sectors = {
            sector: clearance
            for sector, clearance in sector_clearances.items()
            if clearance > SAFE_THRESHOLD
        }
        
        if not safe_sectors:
            return ('none', 0, 0.0)
            
        # Filter out cooldown sectors
        filtered_by_cooldown = {}
        for sector, clearance in safe_sectors.items():
            # Check if this sector is on cooldown
            if self.is_direction_on_cooldown(sector, tolerance=45):
                logger.debug(f"[COOLDOWN FILTER] Sector {sector}° skipped (on cooldown)")
                continue
            
            filtered_by_cooldown[sector] = clearance
        
        # If ALL safe sectors are on cooldown → use them anyway (emergency)
        if not filtered_by_cooldown:
            logger.warning("[COOLDOWN] All safe sectors on cooldown - using anyway")
            filtered_by_cooldown = safe_sectors
        
        # Calculate directions
        normalized_obstacle = obstacle_angle_deg % 360
        opposite_angle = (normalized_obstacle + 180) % 360
        
        # Filter out sectors that point TOWARD obstacle
        filtered_sectors = {}
        rejected_sectors = []
        
        for sector, clearance in safe_sectors.items():
            # Calculate angular distance to obstacle
            angle_to_obstacle = min(
                abs(sector - normalized_obstacle),
                360 - abs(sector - normalized_obstacle)
            )
            
            # Reject if within danger arc of obstacle direction
            if angle_to_obstacle < OBSTACLE_REJECTION_ARC:
                rejected_sectors.append((sector, angle_to_obstacle))
                logger.debug(
                    f"[ESCAPE FILTER] Rejected sector {sector}° "
                    f"(too close to obstacle at {normalized_obstacle:.0f}°, "
                    f"angle_diff={angle_to_obstacle:.0f}°)"
                )
                continue
            
            filtered_sectors[sector] = clearance

        # Log filtering results
        if rejected_sectors:
            logger.warning(
                f"[ESCAPE FILTER] Rejected {len(rejected_sectors)} sectors "
                f"pointing toward obstacle at {normalized_obstacle:.0f}°:"
            )
            for sector, angle_diff in rejected_sectors[:3]:  # Show first 3
                logger.warning(f"  • {sector}° (angle_diff={angle_diff:.0f}°)")

        # Check if all sectors were rejected
        if not filtered_sectors:
            logger.error(
                f"[ESCAPE FILTER] ⚠️  ALL safe sectors rejected! "
                f"Obstacle at {normalized_obstacle:.0f}°"
            )
            logger.error(
                f"[ESCAPE FILTER] Emergency fallback: using opposite direction"
            )
            
            # Try opposite direction even if it was rejected
            opposite_sector = int(opposite_angle // 30) * 30
            if opposite_sector in safe_sectors:
                filtered_sectors = {opposite_sector: safe_sectors[opposite_sector]}
                logger.warning(
                    f"[ESCAPE FILTER] Using opposite sector {opposite_sector}° "
                    f"(clearance: {safe_sectors[opposite_sector]:.2f}m)"
                )
            else:
                # Last resort: use best clearance regardless of direction
                best_safe = max(safe_sectors.items(), key=lambda x: x[1])
                filtered_sectors = {best_safe[0]: best_safe[1]}
                logger.error(
                    f"[ESCAPE FILTER] ⚠️  Last resort: using sector {best_safe[0]}° "
                    f"(best clearance: {best_safe[1]:.2f}m, may point toward obstacle!)"
                )

        scored_sectors = []
        
        for sector, clearance in filtered_sectors.items():
            # Factor 1: Clearance Score [0.0-1.0]
            clearance_score = min(clearance / 3.5, 1.0)
            
            # Factor 2: Obstacle Avoidance [0.0-1.0]
            angle_diff = min(
                abs(sector - normalized_obstacle),
                360 - abs(sector - normalized_obstacle)
            )
            obstacle_avoidance_score = angle_diff / 180.0

            # Factor 3: Opposite Bonus [0.0-1.0] - SMOOTH GRADIENT
            angle_to_opposite = min(
                abs(sector - opposite_angle),
                360 - abs(sector - opposite_angle)
            )
            opposite_bonus = max(0.0, 1.0 - angle_to_opposite / 180.0)
            
            # Factor 4: Forward Bias [0.0-1.0]
            if sector == 0:
                forward_bias_score = 1.00  # Perfect forward
            elif sector in [330, 30]:
                forward_bias_score = 0.90  # ← TĂNG từ 0.85 (encourage slight turns)
            elif sector in [300, 60]:
                forward_bias_score = 0.70  # ← TĂNG từ 0.60 (moderate turns acceptable)
            elif sector in [270, 90]:
                forward_bias_score = 0.45  # ← TĂNG từ 0.40 (side movement less bad)
            elif sector in [240, 120]:
                forward_bias_score = 0.25  # ← TĂNG từ 0.20 (rear-side still possible)
            else:  # 150, 180, 210
                forward_bias_score = 0.05  # ← TĂNG từ 0.00 (backup as last resort, not impossible)
            
            # ✅ WEIGHTED TOTAL với updated weights
            total_score = (
                clearance_score * W_CLEARANCE +
                obstacle_avoidance_score * W_OBSTACLE +
                opposite_bonus * W_OPPOSITE +
                forward_bias_score * W_FORWARD
            )
            
            scored_sectors.append((sector, clearance, total_score))
        
        # Sort by score
        scored_sectors.sort(key=lambda x: x[2], reverse=True)
        
        # Log top 3
        logger.info("[ESCAPE SELECTION] Top 3 candidates:")
        for i, (sector, clearance, score) in enumerate(scored_sectors[:3], 1):
            logger.info(
                f"  {i}. Sector {sector:3d}°: "
                f"clearance={clearance:.2f}m, score={score:.3f}"
            )
        
        best_sector, best_clearance, best_score = scored_sectors[0]
        
        logger.info(
            f"[ESCAPE SELECTION] SELECTED: {best_sector}° "
            f"(clearance: {best_clearance:.2f}m, score: {best_score:.3f})"
        )

        # Set cooldown for the selected sector
        self.last_escape_sector = best_sector
        self.last_escape_time = time.time()
        
        logger.warning(f"[COOLDOWN SET] Sector {best_sector}° blocked for "
                  f"{self.directional_cooldown:.0f}s")
        
        # Map to action (giữ nguyên)
        if best_sector == 0:
            action = 'forward'
        elif best_sector == 30:
            action = 'turn_right'
        elif best_sector == 330:
            action = 'turn_left'
        elif 60 <= best_sector <= 150:
            action = 'rotate_right'
        elif 210 <= best_sector <= 300:
            action = 'rotate_left'
        else:  # 180
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

            current_time = time.time()
            self.last_escape_sector = target_sector
            self.last_escape_time = current_time

            logger.warning(
                f"[DIRECTIONAL COOLDOWN] Sector {target_sector}° blocked for "
                f"{self.directional_cooldown:.0f}s"
            )
            
            # Transition to ESCAPE_WAIT state
            self.state = SafetyState.ESCAPE_WAIT
            self.escape_start_time = current_time
            self.escape_duration = 10.0  # Full 10s for escape
            
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
        stats = {
            'total_aborts': self.abort_count,
            'current_state': self.state.value,
            'consecutive_same_spot_aborts': self.consecutive_aborts_at_same_spot,
            'movement_state': 'forward' if self._is_moving_forward() else 'turning'
        }
        
        # ✅ Add cooldown info if in COOLDOWN state
        if self.state == SafetyState.COOLDOWN:
            import time
            elapsed = time.time() - self.cooldown_start_time
            stats['cooldown_progress'] = f"{elapsed:.1f}s/{self.cooldown_duration:.1f}s"
        
        return stats
    
    def reset_stats(self):
        """Reset statistics (for testing)."""
        self.abort_count = 0
        self.last_abort_position = None
        self.consecutive_aborts_at_same_spot = 0
        self.state = SafetyState.NORMAL
        logger.info("[STATS RESET] Safety monitor cleared")

    def should_abort_mission(self) -> bool:
        """
        Check if mission should abort due to prolonged stuck.
        
        Returns:
            True if stuck for >10s in ESCAPE_WAIT, False otherwise
        """
        if self.state != SafetyState.ESCAPE_WAIT:
            return False
        
        current_time = time.time()
        elapsed = current_time - self.escape_start_time
        
        # Abort mission if stuck in escape for too long
        if elapsed > 10.0:
            logger.error(
                f"[MISSION ABORT] Stuck in ESCAPE_WAIT for {elapsed:.1f}s "
                f"without improvement. Cannot escape locally."
            )
            return True
        
        return False
    
    def is_direction_on_cooldown(self, angle_deg: float, tolerance: int = 45) -> bool:
        """
        Check if a direction is currently on cooldown (recently escaped from there).
        
        Args:
            angle_deg: Direction to check (in degrees, -180 to 180)
            tolerance: Angular tolerance (default: ±60°)
        
        Returns:
            True if direction is on cooldown, False otherwise
        """
        if self.last_escape_sector is None:
            return False
        
        current_time = time.time()
        elapsed = current_time - self.last_escape_time
        
        # Cooldown expired
        if elapsed > self.directional_cooldown:
            return False
        
        # Normalize angle to [0, 360)
        normalized_angle = angle_deg % 360
        
        # Calculate angular distance to escape sector
        angle_diff = abs(normalized_angle - self.last_escape_sector)
        
        # Handle wraparound (e.g., 350° vs 10° should be 20° apart, not 340°)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Check if within tolerance
        if angle_diff < tolerance:
            logger.debug(
                f"[COOLDOWN CHECK] {angle_deg:.0f}° is within {tolerance}° of "
                f"escape sector {self.last_escape_sector}° "
                f"(elapsed: {elapsed:.1f}s/{self.directional_cooldown:.0f}s)"
            )
            return True
        
        return False

    def check_critical_abort(
        self,
        lidar_data,
        robot_pos: Optional[Dict] = None
    ) -> Dict:
        """
        Main safety check với state machine (REFACTORED với COOLDOWN state).
        
        State Flow:
        NORMAL → ABORT → ESCAPE_WAIT → COOLDOWN → NORMAL
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
            
            # ========================================
            # STATE 1: ESCAPE_WAIT (đang thực hiện escape)
            # ========================================
            if self.state == SafetyState.ESCAPE_WAIT:
                elapsed = current_time - self.escape_start_time
                
                # ✅ Track start distance ONLY ONCE khi enter state
                if not hasattr(self, '_escape_start_distance'):
                    self._escape_start_distance = min_distance
                    self._escape_enter_time = current_time
                    logger.debug(
                        f"[ESCAPE_WAIT] Started with distance: {min_distance:.2f}m"
                    )
                
                # Calculate improvement
                distance_improvement = min_distance - self._escape_start_distance
                improvement_ratio = distance_improvement / max(self._escape_start_distance, 0.1)
                
                # ✅ SUCCESS CHECK: Cleared to safe distance
                if min_distance > self.thresholds.RESUME_SAFE:  # 0.45m
                    logger.info(
                        f"[ESCAPE SUCCESS] ✓ Cleared to {min_distance:.2f}m "
                        f"after {elapsed:.1f}s (improved +{distance_improvement:.2f}m) "
                        f"→ COOLDOWN"
                    )
                    
                    # Transition to COOLDOWN state
                    self.state = SafetyState.COOLDOWN
                    self.cooldown_start_time = current_time
                    
                    # Cleanup tracking variables
                    if hasattr(self, '_escape_start_distance'):
                        delattr(self, '_escape_start_distance')
                    if hasattr(self, '_rotate_attempted'):
                        self._rotate_attempted = False
                    if hasattr(self, '_alternative_attempted'):
                        delattr(self, '_alternative_attempted')
                    
                    # Return PAUSE command during cooldown
                    return {
                        'abort': False,
                        'command': self._pause_command(),
                        'min_distance': min_distance,
                        'state': 'cooldown_buffer'
                    }
                
                # ✅ EARLY FAILURE DETECTION (after 2s, no improvement)
                if elapsed > 2.0 and improvement_ratio < 0.1:
                    logger.warning(
                        f"[ESCAPE STALL] ⚠️ No improvement after {elapsed:.1f}s "
                        f"(distance: {self._escape_start_distance:.2f}m → {min_distance:.2f}m, "
                        f"improvement: {improvement_ratio:.1%})"
                    )
                    
                    # FALLBACK 1: Try alternative direction
                    if not hasattr(self, '_alternative_attempted'):
                        logger.warning("[ESCAPE STALL] Trying alternative direction...")
                        
                        sector_clearances = self._analyze_360_clearances(obstacles)
                        
                        # Get second-best direction
                        sorted_sectors = sorted(
                            sector_clearances.items(),
                            key=lambda x: x[1],
                            reverse=True
                        )
                        
                        # Try second best if clearance >0.4m
                        if len(sorted_sectors) >= 2 and sorted_sectors[1][1] > 0.4:
                            alt_sector, alt_clearance = sorted_sectors[1]
                            
                            logger.warning(
                                f"[ESCAPE STALL] Alternative: {alt_sector}° "
                                f"(clearance: {alt_clearance:.2f}m)"
                            )
                            
                            self._alternative_attempted = True
                            self.escape_start_time = current_time  # Reset timer
                            self._escape_start_distance = min_distance
                            
                            # Generate alternative escape command
                            action_type, target_sector, clearance = self._select_escape_direction(
                                sector_clearances,
                                0.0,
                                obstacles[0][0] if obstacles else 0.0
                            )
                            
                            return {
                                'abort': True,
                                'command': self._generate_escape_command(
                                    action_type, target_sector, clearance
                                ),
                                'min_distance': min_distance,
                                'state': 'escape_alternative',
                                'clearances': sector_clearances
                            }
                
                # ✅ TIMEOUT HANDLING (8s timeout)
                if elapsed > self.escape_duration:
                    logger.error(
                        f"[ESCAPE TIMEOUT] ✗ Failed after {elapsed:.1f}s "
                        f"(still at {min_distance:.2f}m, started at {self._escape_start_distance:.2f}m)"
                    )
                    
                    # Analyze current situation
                    sector_clearances = self._analyze_360_clearances(obstacles)
                    max_sector = max(sector_clearances, key=sector_clearances.get)
                    max_clearance = sector_clearances[max_sector]
                    
                    # FALLBACK 2: Rotate rescue (if not attempted AND clearance >0.2m)
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
                        self.escape_start_time = current_time
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
                    
                    # FALLBACK 3: Nav2 rescue (last resort)
                    logger.error(
                        f"[TIMEOUT] Cannot escape locally "
                        f"(max clearance: {max_clearance:.2f}m at {max_sector}°)"
                    )
                    logger.error("Requesting Nav2 global planner...")
                    
                    # Reset all flags
                    if hasattr(self, '_rotate_attempted'):
                        delattr(self, '_rotate_attempted')
                    if hasattr(self, '_alternative_attempted'):
                        delattr(self, '_alternative_attempted')
                    if hasattr(self, '_escape_start_distance'):
                        delattr(self, '_escape_start_distance')
                    
                    self.state = SafetyState.NORMAL
                    
                    return {
                        'abort': True,
                        'command': self._emergency_stop(),
                        'min_distance': min_distance,
                        'state': 'escape_timeout_nav2_needed',
                        'request_nav2_rescue': True,
                        'clearances': sector_clearances
                    }
                
                # Still escaping - pause commands
                logger.debug(
                    f"[ESCAPING] {elapsed:.1f}s/8.0s, "
                    f"dist={min_distance:.2f}m (started: {self._escape_start_distance:.2f}m, "
                    f"improvement: {improvement_ratio:.1%})"
                )
                return {
                    'abort': False,
                    'command': self._pause_command(),
                    'min_distance': min_distance,
                    'state': 'escape_wait'
                }
            
            # ========================================
            # STATE 2: COOLDOWN (buffer sau escape)
            # ========================================
            if self.state == SafetyState.COOLDOWN:
                """
                COOLDOWN state: Robot đã escape thành công, pause 3s để stabilize
                trước khi cho phép forward command.
                
                Mục đích:
                - Tránh robot forward ngay sau escape → abort lại
                - Cho robot thời gian stabilize (gyro/encoder settle)
                - Verify distance vẫn safe trước khi continue
                """
                elapsed = current_time - self.cooldown_start_time
                
                # ✅ SAFETY RE-CHECK: Nếu distance giảm → rollback to ESCAPE_WAIT
                if min_distance < 0.50:  # ← WARNING threshold
                    logger.warning(
                        f"[COOLDOWN] ⚠️ Distance decreased to {min_distance:.2f}m "
                        f"→ Rolling back to ESCAPE_WAIT"
                    )
                    
                    # Rollback to escape
                    self.state = SafetyState.ESCAPE_WAIT
                    self.escape_start_time = current_time
                    self._escape_start_distance = min_distance
                    
                    # Re-analyze and escape
                    return self._force_escape_response(obstacles, robot_pos)
                
                # ✅ COOLDOWN COMPLETE
                if elapsed >= self.cooldown_duration:  # ← 3 giây buffer
                    logger.info(
                        f"[COOLDOWN COMPLETE] ✓ {elapsed:.1f}s elapsed, "
                        f"distance stable at {min_distance:.2f}m → NORMAL"
                    )
                    
                    # NOW safe to transition to NORMAL
                    self.state = SafetyState.NORMAL
                    
                    return {
                        'abort': False,
                        'command': None,  # ← Cho phép main loop execute forward
                        'min_distance': min_distance,
                        'state': 'normal'
                    }
                
                # Still in cooldown - continue pausing
                logger.debug(
                    f"[COOLDOWN] {elapsed:.1f}s/{self.cooldown_duration:.1f}s, "
                    f"dist={min_distance:.2f}m (stable)"
                )
                return {
                    'abort': False,
                    'command': self._pause_command(),  # ← Continue pausing
                    'min_distance': min_distance,
                    'state': 'cooldown',
                    'forward_blocked': True
                }
            
            # ========================================
            # STATE 3: NORMAL - Check for new abort
            # ========================================
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
                
                # Track starting distance for escape validation
                self._escape_start_distance = distance
                
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
        