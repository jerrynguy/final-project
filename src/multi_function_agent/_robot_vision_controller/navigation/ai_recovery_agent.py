"""
AI Recovery Agent Module
LLM-powered intelligent recovery from stuck situations.
Uses Llama-3.2-3B for spatial reasoning and escape planning.
"""

import json
import logging
import time
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class AIRecoveryAgent:
    """
    Lightweight LLM agent for stuck recovery in explore mode.
    Only invoked when robot is detected as stuck.
    """
    
    # System prompt optimized for Llama-3.2-3B
    SYSTEM_PROMPT = """You are an expert robot navigation recovery system. Your job is to analyze stuck situations and generate safe escape commands.

**Input Format:**
- clearances: {front, left, right} distances in meters
- obstacles: list of detected obstacles with positions
- last_actions: recent navigation commands
- stuck_reason: why robot is stuck

**Output Format (JSON only, no explanation):**
{
  "action": "move_backward|rotate_left|rotate_right|move_forward",
  "linear": float (-0.3 to 0.3 m/s),
  "angular": float (-1.5 to 1.5 rad/s),
  "duration": float (0.5 to 2.5 seconds),
  "reason": "brief_tactical_explanation"
}

**Recovery Rules:**
1. If obstacle in FRONT (<0.5m): BACKUP first (linear=-0.2, duration=1.0)
2. If obstacle on ONE side: ROTATE away from it (angular=±1.0)
3. If surrounded (<0.5m all sides): ROTATE 180° (angular=1.5, duration=2.0)
4. After backup: ADD slight rotation toward clearer space
5. NEVER move forward if front clearance <0.6m

**Safety Constraints:**
- |linear| ≤ 0.3 m/s
- |angular| ≤ 1.5 rad/s
- duration ≤ 2.5s
- Prefer rotation over blind movement

**Output ONLY valid JSON. No markdown, no explanation.**"""
    
    def __init__(self):
        """Initialize AI recovery agent with preloaded Llama-3.2-3B."""
        try:
            # Get preloaded model from model manager
            from multi_function_agent._robot_vision_controller.core.models import (
                get_robot_vision_model_manager
            )
            
            model_manager = get_robot_vision_model_manager()
            self.llm = model_manager.get_ai_recovery_model()
            
            if self.llm:
                self.is_available = True
                logger.info("✅ AI Recovery Agent initialized with preloaded model")
            else:
                self.is_available = False
                logger.warning("⚠️ AI Recovery model not available, will use fallback")
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize AI Recovery Agent: {e}")
            self.is_available = False
        
        # Statistics
        self.total_invocations = 0
        self.successful_parses = 0
        self.total_inference_time = 0.0
    
    async def generate_escape_command(
        self,
        vision_context: Dict,
        last_actions: List[str],
        stuck_reason: str = "repeated_stops"
    ) -> Dict:
        """
        Generate intelligent escape command using LLM reasoning.
        
        Args:
            vision_context: Spatial analysis with clearances, obstacles
            last_actions: Recent navigation actions
            stuck_reason: Why robot is stuck
            
        Returns:
            dict: Navigation command {action, linear, angular, duration, reason}
        """
        if not self.is_available:
            return self._fallback_escape_command(vision_context)
        
        self.total_invocations += 1
        start_time = time.time()
        
        try:
            # Build concise context for LLM
            context = self._build_context(vision_context, last_actions, stuck_reason)
            
            # Query LLM
            response = await self._query_llm(context)
            
            # Parse response
            escape_cmd = self._parse_response(response)
            
            # Track performance
            inference_time = (time.time() - start_time) * 1000
            self.total_inference_time += inference_time
            
            # Update model manager stats
            try:
                from multi_function_agent._robot_vision_controller.core.models import (
                    get_robot_vision_model_manager
                )
                model_manager = get_robot_vision_model_manager()
                model_manager.update_ai_recovery_inference_time(inference_time / 1000)
            except:
                pass
            
            logger.info(
                f"[AI RECOVERY] Generated escape in {inference_time:.0f}ms: "
                f"{escape_cmd['action']} → {escape_cmd['reason']}"
            )
            
            return escape_cmd
            
        except Exception as e:
            logger.error(f"[AI RECOVERY] LLM failed: {e}, using fallback")
            return self._fallback_escape_command(vision_context)
    
    def _build_context(
        self,
        vision_context: Dict,
        last_actions: List[str],
        stuck_reason: str
    ) -> Dict:
        """
        Build minimal context for LLM (reduce token usage).
        """
        clearances = vision_context.get('clearances', {})
        obstacles = vision_context.get('obstacles', [])
        
        # Simplify obstacle data
        obstacle_summary = []
        for obs in obstacles[:3]:  # Max 3 obstacles to keep context short
            obstacle_summary.append({
                'position': obs.get('position', 'unknown'),
                'distance': round(obs.get('distance_estimate', 999), 2),
                'threat': obs.get('threat_level', 'unknown')
            })
        
        context = {
            "clearances": {
                "front": round(clearances.get('forward', 999), 2),
                "left": round(clearances.get('left', 999), 2),
                "right": round(clearances.get('right', 999), 2)
            },
            "obstacles": obstacle_summary,
            "last_actions": last_actions[-3:] if last_actions else [],
            "stuck_reason": stuck_reason
        }
        
        return context
    
    async def _query_llm(self, context: Dict) -> str:
        """
        Query LLM with context.
        """
        user_message = f"""Robot is stuck. Analyze and generate escape command.

Context:
{json.dumps(context, indent=2)}

Generate escape command (JSON only):"""
        
        messages = [
            {"role": "system", "content": self.SYSTEM_PROMPT},
            {"role": "user", "content": user_message}
        ]
        
        response = await self.llm.ainvoke(messages)
        return response.content.strip()
    
    def _parse_response(self, response_text: str) -> Dict:
        """
        Parse LLM response into navigation command.
        
        Handles markdown code blocks and validates output.
        """
        # Remove markdown code blocks
        if '```json' in response_text:
            response_text = response_text.split('```json')[1].split('```')[0].strip()
        elif '```' in response_text:
            response_text = response_text.split('```')[1].split('```')[0].strip()
        
        # Parse JSON
        try:
            escape_cmd = json.loads(response_text)
            
            # Validate required fields
            required_fields = ['action', 'linear', 'angular', 'duration', 'reason']
            if not all(field in escape_cmd for field in required_fields):
                raise ValueError(f"Missing required fields: {required_fields}")
            
            # Validate ranges
            escape_cmd['linear'] = max(-0.3, min(0.3, float(escape_cmd['linear'])))
            escape_cmd['angular'] = max(-1.5, min(1.5, float(escape_cmd['angular'])))
            escape_cmd['duration'] = max(0.5, min(2.5, float(escape_cmd['duration'])))
            
            self.successful_parses += 1
            return escape_cmd
            
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            logger.error(f"[AI RECOVERY] Parse failed: {e}")
            logger.debug(f"[AI RECOVERY] Raw response: {response_text}")
            raise
    
    def _fallback_escape_command(self, vision_context: Dict) -> Dict:
        """
        Rule-based fallback when LLM fails or unavailable.
        
        Simple heuristic: Backup if front blocked, otherwise rotate.
        """
        clearances = vision_context.get('clearances', {})
        front_clear = clearances.get('forward', 999)
        left_clear = clearances.get('left', 999)
        right_clear = clearances.get('right', 999)
        
        # Front blocked → Backup
        if front_clear < 0.5:
            return {
                'action': 'move_backward',
                'linear': -0.2,
                'angular': 0.0,
                'duration': 1.0,
                'reason': 'fallback_backup_front_blocked'
            }
        
        # Left blocked → Rotate right
        elif left_clear < right_clear:
            return {
                'action': 'rotate_right',
                'linear': 0.0,
                'angular': -1.0,
                'duration': 1.5,
                'reason': 'fallback_rotate_right_left_blocked'
            }
        
        # Right blocked → Rotate left
        else:
            return {
                'action': 'rotate_left',
                'linear': 0.0,
                'angular': 1.0,
                'duration': 1.5,
                'reason': 'fallback_rotate_left_right_blocked'
            }
    
    def get_stats(self) -> Dict:
        """Get agent performance statistics."""
        avg_inference_time = (
            self.total_inference_time / max(1, self.total_invocations)
        )
        
        return {
            'total_invocations': self.total_invocations,
            'successful_parses': self.successful_parses,
            'parse_success_rate': self.successful_parses / max(1, self.total_invocations),
            'avg_inference_time_ms': avg_inference_time,
            'is_available': self.is_available
        }