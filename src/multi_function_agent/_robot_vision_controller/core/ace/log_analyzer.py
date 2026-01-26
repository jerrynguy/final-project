"""
ACE Log Analyzer Module
Trích xuất và format logs cho LLM phân tích.
"""

import logging
import json
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, asdict
from collections import deque

from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds

logger = logging.getLogger(__name__)


@dataclass
class AbortEvent:
    """Một sự kiện abort được ghi lại."""
    iteration: int
    timestamp: float
    obstacle_angle: float
    obstacle_distance: float
    robot_position: Dict[str, float]
    command_executed: str
    reason: str
    state: str  # 'normal', 'force_escape', 'escape_wait', etc.


@dataclass
class MissionSummary:
    """Tóm tắt một mission."""
    mission_type: str
    mission_description: str
    total_iterations: int
    total_aborts: int
    stuck_episodes: int
    completion_status: str
    duration_seconds: float
    abort_events: List[AbortEvent]

# Key performance metrics
    command_success_rate: float = 0.0  # % commands not aborted
    near_miss_count: int = 0  # Obstacles < CRITICAL_ABORT but didn't abort
    distance_traveled: float = 0.0  # Total distance in meters
    avg_speed: float = 0.0  # m/s

class LogBuffer:
    """
    Buffer lưu trữ logs trong memory cho ACE analysis.
    
    Giới hạn: 200 iterations gần nhất để tránh memory overflow.
    """
    
    def __init__(self, max_size: int = 200):
        """Initialize log buffer."""
        self.max_size = max_size
        self.iterations = deque(maxlen=max_size)
        self.abort_events = []
        self.mission_start_time = None
        self.mission_info = {}
        self.near_misses = []
        self.total_distance = 0.0
        self.last_position = None
    
    def log_iteration(
        self,
        iteration: int,
        robot_pos: Optional[Dict],
        vision_analysis: Dict,
        navigation_decision: Dict,
        abort_info: Optional[Dict] = None
    ):
        """
        Ghi lại một iteration.
        
        Args:
            iteration: Số thứ tự iteration
            robot_pos: {'x': float, 'y': float, 'theta': float}
            vision_analysis: Kết quả phân tích vision
            navigation_decision: Quyết định di chuyển
            abort_info: Thông tin abort (nếu có)
        """
        entry = {
            'iteration': iteration,
            'timestamp': self._get_timestamp(),
            'robot_pos': robot_pos,
            'safety_score': vision_analysis.get('safety_score', 0),
            'obstacles': len(vision_analysis.get('obstacles', [])),
            'action': navigation_decision.get('action'),
            'reason': navigation_decision.get('reason'),
        }
        
        # Nếu có abort, lưu chi tiết
        if abort_info and abort_info.get('abort'):
            abort_event = AbortEvent(
                iteration=iteration,
                timestamp=self._get_timestamp(),
                obstacle_angle=abort_info.get('obstacle_angle', 0),
                obstacle_distance=abort_info.get('min_distance', 0),
                robot_position=robot_pos or {},
                command_executed=abort_info['command'].get('action', 'unknown'),
                reason=abort_info['command'].get('reason', 'unknown'),
                state=abort_info.get('state', 'unknown')
            )
            self.abort_events.append(abort_event)
            entry['abort'] = True
        
        # Track near misses (close calls that didn't abort)
        obstacles = vision_analysis.get('obstacles', [])
        if obstacles and not (abort_info and abort_info.get('abort')):
            min_dist = min(obs.get('distance_estimate', 999) for obs in obstacles)
            
            if min_dist < 0.25:  # SafetyThresholds.CRITICAL_ABORT
                self.near_misses.append({
                    'iteration': iteration,
                    'distance': min_dist,
                    'position': robot_pos
                })
        
        # Track distance traveled
        if robot_pos and self.last_position:
            import math
            dx = robot_pos['x'] - self.last_position['x']
            dy = robot_pos['y'] - self.last_position['y']
            dist = math.sqrt(dx**2 + dy**2)
            self.total_distance += dist
        
        self.last_position = robot_pos
        self.iterations.append(entry)
    
    def set_mission_info(self, mission_type: str, description: str):
        """Lưu thông tin mission."""
        import time
        self.mission_info = {
            'type': mission_type,
            'description': description
        }
        self.mission_start_time = time.time()
    
    def get_mission_summary(self, completion_status: str) -> MissionSummary:
        stuck_episodes = self._count_stuck_episodes()
        duration = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        # Calculate metrics
        total_iterations = len(self.iterations)
        total_aborts = len(self.abort_events)
        
        # Command success rate
        successful_commands = total_iterations - total_aborts
        command_success_rate = (successful_commands / max(total_iterations, 1)) * 100
        
        # Near miss count
        near_miss_count = len(self.near_misses)
        
        # Average speed
        avg_speed = self.total_distance / max(duration, 1)
        
        return MissionSummary(
            mission_type=self.mission_info.get('type', 'unknown'),
            mission_description=self.mission_info.get('description', ''),
            total_iterations=total_iterations,
            total_aborts=total_aborts,
            stuck_episodes=stuck_episodes,
            completion_status=completion_status,
            duration_seconds=duration,
            abort_events=self.abort_events[-50:],
            
            # Performance metrics
            command_success_rate=command_success_rate,
            near_miss_count=near_miss_count,
            distance_traveled=self.total_distance,
            avg_speed=avg_speed
        )
    
    def _count_stuck_episodes(self) -> int:
        """
        Đếm số lần stuck (3+ consecutive aborts cùng vị trí).
        """
        if len(self.abort_events) < 3:
            return 0
        
        stuck_count = 0
        consecutive_near = 0
        last_pos = None
        
        for event in self.abort_events:
            current_pos = event.robot_position
            
            if last_pos is None:
                last_pos = current_pos
                consecutive_near = 1
                continue
            
            # Check if near previous position (<0.1m)
            import math
            distance = math.sqrt(
                (current_pos['x'] - last_pos['x'])**2 +
                (current_pos['y'] - last_pos['y'])**2
            )
            
            if distance < 0.1:
                consecutive_near += 1
                if consecutive_near >= 3:
                    stuck_count += 1
                    consecutive_near = 0  # Reset counter
            else:
                consecutive_near = 1
                last_pos = current_pos
        
        return stuck_count
    
    def _get_timestamp(self) -> float:
        """Get current timestamp."""
        import time
        return time.time()
    
    def clear(self):
        """Clear buffer (sau khi phân tích xong)."""
        self.iterations.clear()
        self.abort_events.clear()
        self.mission_start_time = None
        self.mission_info = {}


class LogAnalyzer:
    """
    Phân tích logs và format thành prompt cho LLM.
    """
    
    @staticmethod
    def format_for_llm_json(summary: MissionSummary) -> dict:
        """
        Format mission summary as structured JSON for LLM.
        
        Advantages over prose:
        - 50% fewer tokens
        - No parsing ambiguity
        - Better for structured analysis
        
        Returns:
            dict ready for json.dumps()
        """
        # Pre-compute patterns before sending to LLM
        death_pendulum = LogAnalyzer._detect_death_pendulum(summary.abort_events)
        location_groups = LogAnalyzer._group_aborts_by_location(
            summary.abort_events, 
            tolerance=0.3
        )
        
        return {
            "mission": {
                "type": summary.mission_type,
                "description": summary.mission_description,
                "duration_sec": round(summary.duration_seconds, 1),
                "status": summary.completion_status
            },
            "metrics": {
                "total_iterations": summary.total_iterations,
                "total_aborts": summary.total_aborts,
                "stuck_episodes": summary.stuck_episodes,
                "abort_rate_pct": round(
                    (summary.total_aborts / max(summary.total_iterations, 1)) * 100, 
                    1
                ),
                "command_success_rate_pct": round(summary.command_success_rate, 1),
                "near_miss_count": summary.near_miss_count,
                "distance_traveled_m": round(summary.distance_traveled, 2),
                "avg_speed_mps": round(summary.avg_speed, 3)
            },
            "patterns": {
                "death_pendulum": {
                    "detected": death_pendulum is not None,
                    "details": death_pendulum if death_pendulum else None
                },
                "abort_hotspots": [
                    {
                        "position": location,
                        "count": len(events),
                        "sample_angles_deg": [
                            round(e.obstacle_angle, 1) 
                            for e in events[:5]
                        ],
                        "sample_distances_m": [
                            round(e.obstacle_distance, 2) 
                            for e in events[:5]
                        ]
                    }
                    for location, events in list(location_groups.items())[:3]
                ]
            },
            "current_parameters": LogAnalyzer.extract_current_parameters()
        }
    
    @staticmethod
    def _group_aborts_by_location(
        events: List[AbortEvent],
        tolerance: float = 0.3
    ) -> Dict[str, List[AbortEvent]]:
        """
        Nhóm aborts theo vị trí (tolerance 0.3m).
        
        Returns:
            Dict[location_str, List[AbortEvent]]
        """
        import math
        
        groups = {}
        
        for event in events:
            pos = event.robot_position
            
            # Find existing group
            found_group = None
            for location_str, group_events in groups.items():
                # Parse location string back to coords
                ref_x, ref_y = map(float, location_str.strip('()').split(','))
                
                distance = math.sqrt(
                    (pos['x'] - ref_x)**2 +
                    (pos['y'] - ref_y)**2
                )
                
                if distance < tolerance:
                    found_group = location_str
                    break
            
            # Add to existing or create new group
            if found_group:
                groups[found_group].append(event)
            else:
                location_key = f"({pos['x']:.2f},{pos['y']:.2f})"
                groups[location_key] = [event]
        
        # Sort by number of events (descending)
        sorted_groups = dict(
            sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
        )
        
        return sorted_groups
    
    @staticmethod
    def _detect_death_pendulum(events: List[AbortEvent]) -> Optional[str]:
        """
        Detect death pendulum pattern.
        
        Returns:
            Description string if detected, None otherwise
        """
        if len(events) < 5:
            return None
        
        # Check last 10 events
        recent = events[-10:]
        
        # Pattern 1: Oscillating angles
        angles = [e.obstacle_angle for e in recent]
        
        # Check if alternating between positive/negative
        sign_changes = 0
        for i in range(1, len(angles)):
            if (angles[i] > 0) != (angles[i-1] > 0):
                sign_changes += 1
        
        if sign_changes >= 4:  # At least 4 sign changes in 10 events
            return (
                f"Oscillation pattern: angles alternating "
                f"{sign_changes} times in last 10 aborts. "
                f"Range: [{min(angles):.1f}°, {max(angles):.1f}°]"
            )
        
        # Pattern 2: Stuck at same position
        positions = [e.robot_position for e in recent]
        
        import math
        max_movement = 0
        for i in range(1, len(positions)):
            movement = math.sqrt(
                (positions[i]['x'] - positions[i-1]['x'])**2 +
                (positions[i]['y'] - positions[i-1]['y'])**2
            )
            max_movement = max(max_movement, movement)
        
        if max_movement < 0.05:  # Moved less than 5cm total
            return (
                f"Position stuck: max movement {max_movement:.3f}m "
                f"across {len(recent)} aborts"
            )
        
        return None
    
    @staticmethod
    def extract_current_parameters() -> Dict[str, Any]:
        """
        Trích xuất current parameters từ SafetyThresholds.
        
        Returns:
            Dict of current parameter values
        """
        from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds
        
        return {
            'ESCAPE_DURATION': SafetyThresholds.ESCAPE_DURATION,
            'ROTATION_DURATION': SafetyThresholds.ROTATION_DURATION,
            'BACKUP_DURATION': SafetyThresholds.BACKUP_DURATION,
            'CREEP_SPEED_MULTIPLIER': SafetyThresholds.CREEP_SPEED_MULTIPLIER,
            'TURN_LEFT_AGGRESSIVENESSLY': SafetyThresholds.TURN_LEFT_AGGRESSIVENESSLY,
            'TURN_RIGHT_AGGRESSIVENESSLY': SafetyThresholds.TURN_RIGHT_AGGRESSIVENESSLY,
            'FRONTIER_PREFERENCE_WEIGHT': SafetyThresholds.FRONTIER_PREFERENCE_WEIGHT,
            'STUCK_PENALTY_MULTIPLIER': SafetyThresholds.STUCK_PENALTY_MULTIPLIER,
            'STUCK_THRESHOLD': SafetyThresholds.STUCK_THRESHOLD,
            'STUCK_POSITION_TOLERANCE': SafetyThresholds.STUCK_POSITION_TOLERANCE,
            'ABORT_SPAM_THRESHOLD': SafetyThresholds.ABORT_SPAM_THRESHOLD,
        }