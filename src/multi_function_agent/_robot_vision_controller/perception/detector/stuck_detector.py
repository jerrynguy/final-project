"""
Stuck Pattern Detector Module
Detects repetitive navigation patterns indicating robot is stuck.
"""

import time
import logging
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from collections import deque

logger = logging.getLogger(__name__)


@dataclass
class NavigationSnapshot:
    """Snapshot of robot state at a moment."""
    timestamp: float
    position: Tuple[float, float]
    action: str
    distance_traveled: float


class StuckDetector:
    """
    Detects stuck patterns:
    1. Orbit loop: Same position for N iterations
    2. Oscillation: Alternating left/right commands
    3. Backup loop: Repeated backup commands
    """
    
    def __init__(
        self,
        history_window: int = 20,
        position_variance_threshold: float = 0.01,  # 10cm
        stuck_time_threshold: float = 5.0,  # seconds
        displacement_threshold: float = 0.1
    ):
        """
        Initialize stuck detector.
        """
        self.history_window = history_window
        self.position_threshold = position_variance_threshold
        self.stuck_time_threshold = stuck_time_threshold
        self.displacement_threshold = displacement_threshold
        
        # Tracking
        self.history = deque(maxlen=history_window)
        self.stuck_start_time: Optional[float] = None
        self.last_stuck_detection: float = 0.0
        self.stuck_count: int = 0
        
        # Action pattern tracking
        self.recent_actions = deque(maxlen=10)
        
    def update(self, robot_pos: Dict, action: str) -> Dict:
        """
        Update detector with new navigation snapshot.
        """
        current_time = time.time()
        
        # Skip if no position
        if not robot_pos or 'x' not in robot_pos or 'y' not in robot_pos:
            return self._no_stuck_result()
        
        # Create snapshot
        position = (robot_pos['x'], robot_pos['y'])
        
        # Calculate distance from last position
        distance_traveled = 0.0
        if self.history:
            last_pos = self.history[-1].position
            dx = position[0] - last_pos[0]
            dy = position[1] - last_pos[1]
            distance_traveled = np.sqrt(dx**2 + dy**2)
        
        snapshot = NavigationSnapshot(
            timestamp=current_time,
            position=position,
            action=action,
            distance_traveled=distance_traveled
        )
        
        self.history.append(snapshot)
        self.recent_actions.append(action)
        
        # Wait for enough history
        if len(self.history) < self.history_window // 2:
            return self._no_stuck_result()
        
        # Check stuck patterns
        stuck_result = self._check_stuck_patterns()
        
        # Update stuck tracking
        if stuck_result['is_stuck']:
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time
                self.stuck_count += 1
                logger.warning(
                    f"[STUCK #{self.stuck_count}] Detected: {stuck_result['stuck_type']}"
                )
            
            stuck_result['stuck_duration'] = current_time - self.stuck_start_time
            self.last_stuck_detection = current_time
            
        else:
            # Reset if escaped
            if self.stuck_start_time is not None:
                escape_duration = current_time - self.stuck_start_time
                logger.info(f"[ESCAPED] Stuck resolved after {escape_duration:.1f}s")
                self.stuck_start_time = None
        
        return stuck_result

    def _check_stuck_patterns(self) -> Dict:
        """Check all stuck patterns."""
        
        # Pattern 1: Position variance (orbit loop)
        position_stuck = self._check_position_stuck()
        if position_stuck['is_stuck']:
            return position_stuck
        
        # Pattern 2: Action oscillation
        oscillation_stuck = self._check_oscillation()
        if oscillation_stuck['is_stuck']:
            return oscillation_stuck
        
        # Pattern 3: Repeated backup
        backup_stuck = self._check_backup_loop()
        if backup_stuck['is_stuck']:
            return backup_stuck
        
        # Pattern 4 - Corner trap
        corner_trap = self._check_corner_trap()
        if corner_trap['is_stuck']:
            return corner_trap
        
        return self._no_stuck_result()
    
    def _check_position_stuck(self) -> Dict:
        """
        Check if robot stuck in same position.
        
        Method: Calculate variance of recent positions
        """
        if len(self.history) < 10:
            return self._no_stuck_result()
        
        # Check 1: Position Variance
        positions = np.array([s.position for s in self.history])
        variance = np.var(positions, axis=0).sum()

        # Check 2: Total displacement
        total_displacement = sum(s.distance_traveled for s in self.history)
        avg_displacement = total_displacement / len(self.history)
        
        time_span = self.history[-1].timestamp - self.history[0].timestamp

        # DETECTION LOGIC: Stuck if BOTH conditions met
        is_low_variance = variance < self.position_threshold
        is_minimal_displacement = total_displacement < self.displacement_threshold
        is_long_duration = time_span > self.stuck_time_threshold

        if is_low_variance and is_minimal_displacement and is_long_duration:
            logger.error(
                f"[POSITION STUCK] Variance: {variance:.6f}m², "
                f"Displacement: {total_displacement:.3f}m, "
                f"Duration: {time_span:.1f}s"
            )
            
            return {
                'is_stuck': True,
                'stuck_type': 'minimal_displacement',  # ← CHANGED: more descriptive
                'stuck_duration': time_span,
                'confidence': 0.95,  # ← Higher confidence với dual check
                'recommended_action': 'clearance_based_escape',  # ← NEW
                'details': {
                    'position_variance': variance,
                    'total_displacement': total_displacement,
                    'avg_displacement_per_step': avg_displacement,
                    'center': positions.mean(axis=0).tolist()
                }
            }
    
        if is_low_variance or is_minimal_displacement:
            logger.debug(
                f"[STUCK WARNING] Variance: {variance:.6f}m², "
                f"Displacement: {total_displacement:.3f}m "
                f"(threshold: {self.displacement_threshold}m)"
            )
        
        return self._no_stuck_result()

    def _check_corner_trap(self) -> Dict:
        """
        Detect corner trap pattern:
        - Repeated abort commands
        - Minimal position change
        - Oscillating recovery directions
        """
        if len(self.history) < 8:
            return self._no_stuck_result()
        
        # Count recent abort/recovery actions
        recent_actions = list(self.recent_actions)[-8:]
        
        abort_keywords = ['abort', 'recovery', 'backup', 'critical']
        abort_count = sum(
            1 for action in recent_actions 
            if any(kw in action.lower() for kw in abort_keywords)
        )
        
        # Corner trap signature: >50% abort actions + low displacement
        if abort_count > len(recent_actions) * 0.5:
            # Check displacement
            positions = np.array([s.position for s in list(self.history)[-8:]])
            total_displacement = sum(s.distance_traveled for s in list(self.history)[-8:])
            
            # Corner trap = lots of aborts but minimal progress
            if total_displacement < self.displacement_threshold:  # Less than 30cm in 8 iterations
                logger.error(
                    f"[CORNER TRAP DETECTED] {abort_count}/{len(recent_actions)} aborts, "
                    f"displacement: {total_displacement:.2f}m"
                )
                
                return {
                    'is_stuck': True,
                    'stuck_type': 'corner_trap',
                    'stuck_duration': 0.0,  # Will be updated by caller
                    'confidence': 0.95,
                    'recommended_action': 'corner_escape',
                    'details': {
                        'abort_count': abort_count,
                        'displacement': total_displacement,
                        'recent_actions': recent_actions[-5:]
                    }
                }
        
        return self._no_stuck_result()
    
    def _check_oscillation(self) -> Dict:
        """
        Check if robot oscillating (left-right-left-right pattern).
        
        Method: Detect alternating turn commands
        """
        if len(self.recent_actions) < 6:
            return self._no_stuck_result()
        
        # Count alternating patterns
        actions = list(self.recent_actions)
        alternations = 0
        
        for i in range(len(actions) - 1):
            current = actions[i]
            next_action = actions[i + 1]
            
            # Check for left ↔ right alternation
            if (('left' in current and 'right' in next_action) or
                ('right' in current and 'left' in next_action)):
                alternations += 1
        
        # If >50% of actions are alternating
        if alternations > len(actions) * 0.5:
            logger.error(
                f"[OSCILLATION STUCK] {alternations}/{len(actions)} alternating actions"
            )
            
            return {
                'is_stuck': True,
                'stuck_type': 'oscillation',
                'stuck_duration': 0.0,  # Will be updated by caller
                'confidence': 0.85,
                'recommended_action': 'force_forward',
                'details': {
                    'alternation_count': alternations,
                    'recent_actions': actions[-6:]
                }
            }
        
        return self._no_stuck_result()
    
    def _check_backup_loop(self) -> Dict:
        """
        Check if robot stuck in backup loop.
        
        Method: Count repeated backup commands
        """
        if len(self.recent_actions) < 5:
            return self._no_stuck_result()
        
        # Count backup commands
        backup_count = sum(
            1 for action in self.recent_actions
            if 'backup' in action or 'backward' in action
        )
        
        # If >60% are backups
        if backup_count > len(self.recent_actions) * 0.6:
            logger.error(
                f"[BACKUP LOOP] {backup_count}/{len(self.recent_actions)} backup commands"
            )
            
            return {
                'is_stuck': True,
                'stuck_type': 'backup_loop',
                'stuck_duration': 0.0,
                'confidence': 0.8,
                'recommended_action': 'random_turn',
                'details': {
                    'backup_count': backup_count,
                    'total_actions': len(self.recent_actions)
                }
            }
        
        return self._no_stuck_result()
    
    def _no_stuck_result(self) -> Dict:
        """Return no-stuck result."""
        return {
            'is_stuck': False,
            'stuck_type': None,
            'stuck_duration': 0.0,
            'confidence': 0.0,
            'recommended_action': None,
            'details': {}
        }
    
    def generate_escape_command(
        self,
        stuck_type: str,
        clearances: Dict
    ) -> Dict:
        """
        Generate escape command based on stuck type.
        """
        if stuck_type == 'minimal_displacement':
            # Find direction with MAX clearance
            left = clearances.get('left', 0)
            right = clearances.get('right', 0)
            front = clearances.get('front', 0)
            
            # Also check rear if available
            rear = clearances.get('rear', 0)
            
            # Build clearance map
            directions = {
                'left': left,
                'right': right,
                'front': front,
                'rear': rear
            }
            
            # Find best direction (max clearance)
            best_dir = max(directions, key=directions.get)
            best_clearance = directions[best_dir]
            
            logger.error(
                f"[CLEARANCE ESCAPE] Stuck at corner - "
                f"Best direction: {best_dir} ({best_clearance:.2f}m)\n"
                f"  Clearances: F:{front:.2f} L:{left:.2f} R:{right:.2f} Rear:{rear:.2f}"
            )

            if best_dir == 'front':
                # Front is clear - just go forward
                return {
                    'action': 'escape_forward',
                    'parameters': {
                        'linear_velocity': 0.3,
                        'angular_velocity': 0.0,
                        'duration': 2.0
                    },
                    'confidence': 0.95,
                    'reason': 'clearance_escape_forward'
                }
            
            elif best_dir == 'left':
                # Turn left + move
                return {
                    'action': 'escape_turn_left',
                    'parameters': {
                        'linear_velocity': 0.2,
                        'angular_velocity': 0.8,  # Strong turn
                        'duration': 2.5
                    },
                    'confidence': 0.95,
                    'reason': 'clearance_escape_left'
                }
            
            elif best_dir == 'right':
                # Turn right + move
                return {
                    'action': 'escape_turn_right',
                    'parameters': {
                        'linear_velocity': 0.2,
                        'angular_velocity': -0.8,  # Strong turn
                        'duration': 2.5
                    },
                    'confidence': 0.95,
                    'reason': 'clearance_escape_right'
                }
            
            else:  # rear is best - need to backup first
                logger.warning("[CLEARANCE ESCAPE] Rear is best - backup strategy")
                return {
                    'action': 'escape_backup',
                    'parameters': {
                        'linear_velocity': -0.25,
                        'angular_velocity': 0.0,
                        'duration': 1.5
                    },
                    'confidence': 0.85,
                    'reason': 'clearance_escape_backward'
                }

        elif stuck_type == 'position_orbit':
            # Aggressive random turn + forward
            import random
            angular = random.choice([0.8, -0.8])
            
            logger.warning(
                f"[ESCAPE] Position orbit → Random turn "
                f"({'left' if angular > 0 else 'right'})"
            )
            
            return {
                'action': 'escape_orbit',
                'parameters': {
                    'linear_velocity': 0.2,
                    'angular_velocity': angular,
                    'duration': 2.0
                },
                'confidence': 0.9,
                'reason': 'escape_position_orbit'
            }
        
        elif stuck_type == 'oscillation':
            # Force straight forward (break oscillation)
            logger.warning("[ESCAPE] Oscillation → Force forward")
            
            return {
                'action': 'escape_oscillation',
                'parameters': {
                    'linear_velocity': 0.25,
                    'angular_velocity': 0.0,
                    'duration': 3.0
                },
                'confidence': 0.9,
                'reason': 'escape_oscillation'
            }
        
        elif stuck_type == 'backup_loop':
            # Strong turn + move forward
            left = clearances.get('left', 0)
            right = clearances.get('right', 0)
            
            angular = 0.8 if left > right else -0.8
            
            logger.warning(
                f"[ESCAPE] Backup loop → Turn "
                f"{'left' if angular > 0 else 'right'} + forward"
            )
            
            return {
                'action': 'escape_backup_loop',
                'parameters': {
                    'linear_velocity': 0.2,
                    'angular_velocity': angular,
                    'duration': 2.5
                },
                'confidence': 0.9,
                'reason': 'escape_backup_loop'
            }
        
        elif stuck_type == 'corner_trap':
            """
            Corner trap strategy:
            1. Backup slowly (0.5-1.0s)
            2. Rotate toward maximum clearance
            3. Move forward aggressively
            """
            left = clearances.get('left', 0)
            right = clearances.get('right', 0)
            rear = clearances.get('rear', 0)
            
            # Phase 1: Backup if rear is clear enough
            if rear > self.displacement_threshold:
                logger.error(
                    f"[CORNER ESCAPE Phase 1] Backup 0.8s (rear: {rear:.2f}m)"
                )
                return {
                    'action': 'corner_escape_backup',
                    'parameters': {
                        'linear_velocity': -0.20,
                        'angular_velocity': 0.0,
                        'duration': 0.8
                    },
                    'confidence': 0.90,
                    'reason': 'corner_trap_backup_first'
                }
            
            # Phase 2: If can't backup, aggressive rotation toward open side
            else:
                # Choose side with MAX clearance
                if left > right + 0.2:
                    angular = 1.0  # Strong left turn
                    direction = "left"
                else:
                    angular = -1.0  # Strong right turn
                    direction = "right"
                
                logger.error(
                    f"[CORNER ESCAPE Phase 2] Aggressive rotate {direction} "
                    f"(L:{left:.2f} R:{right:.2f})"
                )
                
                return {
                    'action': 'corner_escape_rotate',
                    'parameters': {
                        'linear_velocity': 0.15,  # Slight forward during turn
                        'angular_velocity': angular,
                        'duration': 1.5  # Longer duration
                    },
                    'confidence': 0.85,
                    'reason': f'corner_trap_aggressive_{direction}'
                }
        
        else:
            # Generic escape
            return {
                'action': 'escape_generic',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.6,
                    'duration': 2.0
                },
                'confidence': 0.7,
                'reason': 'escape_generic'
            }
    
    def reset(self):
        """Reset detector state."""
        self.history.clear()
        self.recent_actions.clear()
        self.stuck_start_time = None
        logger.info("[STUCK DETECTOR] Reset")
    
    def get_stats(self) -> Dict:
        """Get stuck detection statistics."""
        return {
            'total_stuck_count': self.stuck_count,
            'currently_stuck': self.stuck_start_time is not None,
            'stuck_duration': (
                time.time() - self.stuck_start_time 
                if self.stuck_start_time else 0.0
            ),
            'history_size': len(self.history)
        }