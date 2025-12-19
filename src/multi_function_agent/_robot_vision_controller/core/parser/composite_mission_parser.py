"""
Composite Mission Parser Module
Parses complex multi-step missions from natural language using LLM.
"""

import json
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from langchain_nvidia_ai_endpoints import ChatNVIDIA

logger = logging.getLogger(__name__)

# Data Structures
@dataclass
class StepConfig:
    """Configuration for a single mission step."""
    id: str
    type: str  # explore_area, follow_target, patrol_laps, condition_check, directional_command
    parameters: Dict[str, Any] = field(default_factory=dict)
    success_conditions: List[str] = field(default_factory=list)
    next_step_if_success: Optional[str] = None
    next_step_if_failure: Optional[str] = None
    timeout: Optional[float] = None
    constraints: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ConditionConfig:
    """Configuration for conditional branching."""
    type: str 
    branch_true: str 
    branch_false: str  
    parameters: Dict[str, Any] = field(default_factory=dict) 
    timeout: float = 30.0  

@dataclass
class CompositeMissionConfig:
    """Complete composite mission configuration."""
    mission_type: str = "composite_mission"
    description: str = ""
    steps: List[StepConfig] = field(default_factory=list)
    fallback_strategy: Dict[str, str] = field(default_factory=dict)
    global_constraints: Dict[str, Any] = field(default_factory=dict)

# Composite Mission Parser
class CompositeMissionParser:
    """
    Parse complex natural language commands into structured composite missions.
    
    Supports:
    - Multi-step sequences
    - Conditional branching
    - Directional commands
    - Object detection conditions
    - Timeout handling
    """
    
    def __init__(self):
        """Initialize parser with LLM."""
        self.llm = None  # Lazy loaded
        self._system_prompt = self._load_system_prompt()
    
    def _load_system_prompt(self) -> str:
        """Load comprehensive system prompt for composite mission parsing."""
        from pathlib import Path
        
        prompt_file = Path(__file__).parent / "text" / "composite_mission_parser_prompt.txt"
        
        try:
            with open(prompt_file, 'r') as f:
                return f.read().strip()
        except FileNotFoundError:
            logger.warning(f"Prompt file not found: {prompt_file}, using fallback")
            # Fallback minimal prompt
            return """You are a robot mission planner. Parse natural language into structured composite missions.
            
    Output JSON with: mission_type, description, steps (array of step configs), fallback_strategy.
            
    Step types: explore_area, follow_target, patrol_laps, condition_check, directional_command.
            
    CRITICAL: Ignore RTSP/stream setup - focus on robot actions only."""
    
    async def parse(self, user_prompt: str, builder) -> CompositeMissionConfig:
        """
        Parse natural language into composite mission config.
        """
        try:
            # Lazy load LLM
            if self.llm is None:
                self.llm = self._initialize_llm(builder)
            
            # Build messages
            messages = [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": f"Parse this mission:\n{user_prompt}"}
            ]
            
            # Call LLM
            logger.info(f"[COMPOSITE PARSER] Parsing: '{user_prompt}'")
            response = await self.llm.ainvoke(messages)
            response_text = response.content.strip()
            
            # Clean markdown if present
            if '```json' in response_text:
                response_text = response_text.split('```json')[1].split('```')[0].strip()
            elif '```' in response_text:
                response_text = response_text.split('```')[1].split('```')[0].strip()
            
            # Parse JSON
            mission_dict = json.loads(response_text)
            
            # Validate and convert to config
            config = self._dict_to_config(mission_dict)
            
            logger.info(f"[COMPOSITE PARSER] Success: {len(config.steps)} steps")
            return config
            
        except json.JSONDecodeError as e:
            logger.error(f"[COMPOSITE PARSER] JSON decode failed: {e}")
            raise CompositeParsingError(f"Invalid JSON from LLM: {e}")
        
        except Exception as e:
            logger.error(f"[COMPOSITE PARSER] Parsing failed: {e}")
            raise CompositeParsingError(f"Failed to parse composite mission: {e}")
    
    def _initialize_llm(self, builder):
        """Initialize LLM from builder config."""
        try:
            llm_config = builder._workflow_builder.general_config.llms.get('nim_llm')
            model_name = llm_config.model_name if llm_config else "meta/llama-3.1-70b-instruct"
            temperature = llm_config.temperature if llm_config else 0.0
        except:
            model_name = "meta/llama-3.1-70b-instruct"
            temperature = 0.0
        
        return ChatNVIDIA(
            model=model_name,
            temperature=temperature,
            max_tokens=1000  # Longer for composite missions
        )
    
    def _dict_to_config(self, mission_dict: Dict) -> CompositeMissionConfig:
        """
        Convert parsed JSON dict to CompositeMissionConfig.
        """

        # THÊM: Filter out invalid step types
        valid_types = [
            'explore_area', 'follow_target', 'patrol_laps', 
            'condition_check', 'directional_command'
        ]

        # Convert steps
        steps = []
        for step_data in mission_dict.get('steps', []):
            step_type = step_data['type']

            # Skip invalid types với warning
            if step_type not in valid_types:
                logger.warning(
                    f"[COMPOSITE PARSER] Skipping invalid step type: {step_type}. "
                    f"Valid types: {valid_types}"
                )
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
        
        # Create config
        config = CompositeMissionConfig(
            mission_type=mission_dict.get('mission_type', 'composite_mission'),
            description=mission_dict.get('description', ''),
            steps=steps,
            fallback_strategy=mission_dict.get('fallback_strategy', {}),
            global_constraints=mission_dict.get('global_constraints', {})
        )
        
        # Validate
        self._validate_config(config)
        
        return config
    
    def _validate_config(self, config: CompositeMissionConfig):
        """
        Validate mission config for correctness.
        """
        if not config.steps:
            raise CompositeParsingError("Mission must have at least one step")
        
        # Check unique step IDs
        step_ids = [s.id for s in config.steps]
        if len(step_ids) != len(set(step_ids)):
            raise CompositeParsingError("Step IDs must be unique")
        
        # Check step type validity
        valid_types = [
            'explore_area', 'follow_target', 'patrol_laps', 
            'condition_check', 'directional_command'
        ]
        for step in config.steps:
            if step.type not in valid_types:
                raise CompositeParsingError(
                    f"Invalid step type: {step.type}. "
                    f"Valid types: {valid_types}"
                )
        
        # Check next_step references
        valid_refs = set(step_ids) | {'mission_complete', None}
        for step in config.steps:
            if step.next_step_if_success not in valid_refs:
                raise CompositeParsingError(
                    f"Invalid next_step reference: {step.next_step_if_success}"
                )
        
        logger.info("[VALIDATION] Composite mission config valid ✓")
    
    def is_composite_mission(self, user_prompt: str) -> bool:
        """
        Quick heuristic check if prompt is composite mission.
        """
        # Indicators of composite missions
        composite_keywords = [
            'then', 'after', 'if', 'else', 'otherwise',
            'first', 'next', 'finally', 'when',
            'and then', 'followed by', 'before',
            ',', 'turn', 'go', 'move'
        ]
        
        prompt_lower = user_prompt.lower()
        
        # Check for multiple indicators
        keyword_count = sum(1 for kw in composite_keywords if kw in prompt_lower)
        
        # Composite if 2+ keywords OR explicit sequence words
        return (
            keyword_count >= 2 or
            any(seq in prompt_lower for seq in ['then', 'if', 'after that'])
        )

# Custom Exceptions
class CompositeParsingError(Exception):
    """Raised when composite mission parsing fails."""
    pass