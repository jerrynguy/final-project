"""
Composite Mission Parser Module
Parses complex multi-step missions using direct HTTP to NVIDIA API.
"""

import json
import logging
import asyncio
import httpx
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger(__name__)

# Data Structures (giữ nguyên)
@dataclass
class StepConfig:
    """Configuration for a single mission step."""
    id: str
    type: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    success_conditions: List[str] = field(default_factory=list)
    next_step_if_success: Optional[str] = None
    next_step_if_failure: Optional[str] = None
    timeout: Optional[float] = None
    constraints: Dict[str, Any] = field(default_factory=dict)

@dataclass
class CompositeMissionConfig:
    """Complete composite mission configuration."""
    mission_type: str = "composite_mission"
    description: str = ""
    steps: List[StepConfig] = field(default_factory=list)
    fallback_strategy: Dict[str, str] = field(default_factory=dict)
    global_constraints: Dict[str, Any] = field(default_factory=dict)

# Composite Mission Parser (REWRITTEN)
class CompositeMissionParser:
    """Parse complex natural language commands using direct HTTP."""
    
    def __init__(self):
        """Initialize parser."""
        self.api_key = None
        self.endpoint = "https://integrate.api.nvidia.com/v1/chat/completions"
        self._system_prompt = self._load_system_prompt()
    
    def _load_system_prompt(self) -> str:
        """Load system prompt from file."""
        prompt_file = Path(__file__).parent.parent / "text" / "composite_mission_parser_prompt.txt"
        
        try:
            with open(prompt_file, 'r') as f:
                return f.read().strip()
        except FileNotFoundError:
            logger.warning(f"Prompt file not found: {prompt_file}, using fallback")
            return """You are a robot mission planner. Parse natural language into structured composite missions.

Output JSON with: mission_type, description, steps, fallback_strategy.

CRITICAL: Ignore RTSP/stream setup - focus on robot actions only."""
    
    def _initialize_api(self):
        """Initialize API credentials."""
        import os
        self.api_key = os.getenv("NVIDIA_API_KEY")
        
        if not self.api_key:
            raise ValueError("NVIDIA_API_KEY not found in environment")
        
        logger.info("[LLM INIT] Using direct HTTP client")
        logger.info(f"[LLM INIT] Endpoint: {self.endpoint}")
        logger.info(f"[LLM INIT] API Key: {self.api_key[:20]}...")
    
    async def _call_nvidia_api(self, messages: List[Dict]) -> str:
        """Call NVIDIA API directly using httpx."""
        if not self.api_key:
            self._initialize_api()
        
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": "meta/llama-3.1-70b-instruct",
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": 1000
        }
        
        timeout = httpx.Timeout(130.0, connect=10.0)
        
        logger.info("[LLM CALL] Sending HTTP request...")
        
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.post(
                self.endpoint,
                headers=headers,
                json=payload
            )
            
            response.raise_for_status()
            
            data = response.json()
            content = data["choices"][0]["message"]["content"]
            
            logger.info(f"[LLM CALL] ✅ Response received: {len(content)} chars")
            
            return content
    
    async def parse(self, user_prompt: str, builder) -> CompositeMissionConfig:
        """Parse natural language into composite mission config."""
        try:
            messages = [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": f"Parse this mission:\n{user_prompt}"}
            ]
            
            logger.info(f"[COMPOSITE PARSER] Parsing: '{user_prompt}'")
            
            # Call API with timeout
            try:
                response_text = await asyncio.wait_for(
                    self._call_nvidia_api(messages),
                    timeout=130.0
                )
            except asyncio.TimeoutError:
                logger.error("[COMPOSITE PARSER] Timeout after 30s")
                raise CompositeParsingError("LLM timeout after 30s")
            
            # Clean markdown
            if '```json' in response_text:
                response_text = response_text.split('```json')[1].split('```')[0].strip()
            elif '```' in response_text:
                response_text = response_text.split('```')[1].split('```')[0].strip()
            
            # Parse JSON
            mission_dict = json.loads(response_text)
            
            # Convert to config
            config = self._dict_to_config(mission_dict)
            
            logger.info(f"[COMPOSITE PARSER] Success: {len(config.steps)} steps")
            return config
            
        except asyncio.TimeoutError:
            raise CompositeParsingError("LLM timeout")
        except json.JSONDecodeError as e:
            logger.error(f"[COMPOSITE PARSER] JSON decode failed: {e}")
            logger.error(f"[COMPOSITE PARSER] Raw response: {response_text[:200]}")
            raise CompositeParsingError(f"Invalid JSON from LLM: {e}")
        except httpx.HTTPError as e:
            logger.error(f"[COMPOSITE PARSER] HTTP error: {e}")
            raise CompositeParsingError(f"HTTP request failed: {e}")
        except Exception as e:
            logger.error(f"[COMPOSITE PARSER] Parsing failed: {type(e).__name__}: {e}")
            raise CompositeParsingError(f"Failed to parse: {e}")
    
    def _dict_to_config(self, mission_dict: Dict) -> CompositeMissionConfig:
        """Convert parsed JSON dict to CompositeMissionConfig."""
        valid_types = [
            'explore_area', 'follow_target', 'patrol_laps', 
            'condition_check', 'directional_command'
        ]
        
        steps = []
        for step_data in mission_dict.get('steps', []):
            step_type = step_data['type']
            
            if step_type not in valid_types:
                logger.warning(f"[COMPOSITE PARSER] Skipping invalid step type: {step_type}")
                continue
            
            step = StepConfig(
                id=step_data['id'],
                type=step_data['type'],
                parameters=step_data.get('parameters', {}),
                success_conditions=step_data.get('success_conditions', []),
                next_step_if_success=step_data.get('next_step_if_success'),
                next_step_if_failure=step_data.get('next_step_if_failure'),
                timeout=step_data.get('timeout'),
                constraints=step_data.get('constraints', {})
            )
            steps.append(step)
        
        config = CompositeMissionConfig(
            mission_type=mission_dict.get('mission_type', 'composite_mission'),
            description=mission_dict.get('description', ''),
            steps=steps,
            fallback_strategy=mission_dict.get('fallback_strategy', {}),
            global_constraints=mission_dict.get('global_constraints', {})
        )
        
        self._validate_config(config)
        
        return config
    
    def _validate_config(self, config: CompositeMissionConfig):
        """Validate mission config."""
        if not config.steps:
            raise CompositeParsingError("Mission must have at least one step")
        
        step_ids = [s.id for s in config.steps]
        if len(step_ids) != len(set(step_ids)):
            raise CompositeParsingError("Step IDs must be unique")
        
        valid_types = [
            'explore_area', 'follow_target', 'patrol_laps', 
            'condition_check', 'directional_command'
        ]
        for step in config.steps:
            if step.type not in valid_types:
                raise CompositeParsingError(f"Invalid step type: {step.type}")
        
        logger.info("[VALIDATION] Composite mission config valid ✓")
    
    def is_composite_mission(self, user_prompt: str) -> bool:
        """Quick heuristic check if prompt is composite mission."""
        composite_keywords = [
            'then', 'after', 'if', 'else', 'otherwise',
            'first', 'next', 'finally', 'when',
            'and then', 'followed by', 'before',
            ',', 'turn', 'go', 'move'
        ]
        
        prompt_lower = user_prompt.lower()
        keyword_count = sum(1 for kw in composite_keywords if kw in prompt_lower)
        
        return (
            keyword_count >= 2 or
            any(seq in prompt_lower for seq in ['then', 'if', 'after that'])
        )

# Custom Exceptions
class CompositeParsingError(Exception):
    """Raised when composite mission parsing fails."""
    pass