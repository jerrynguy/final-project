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


# =============================================================================
# Data Structures
# =============================================================================

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


# =============================================================================
# Composite Mission Parser
# =============================================================================

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
        """Load comprehensive system prompt for mission parsing."""
        return """You are an expert robot mission planner. Parse natural language commands into structured JSON missions.

**IMPORTANT PARSING RULES:**

1. **Ignore RTSP/stream URL setup** - it's handled automatically
   - "Control robot using rtsp://..." → DON'T create a step for this!
   - Start with the FIRST ACTUAL ACTION

2. **Directional commands** should use "directional_command" type:
   - "go right first" → {"type": "directional_command", "parameters": {"direction": "right"}}
   - "turn left" → {"type": "directional_command", "parameters": {"direction": "left"}}

3. **Duration parsing**:
   - "explore 120 seconds" → {"type": "explore_area", "parameters": {"duration": 120}}
   - "patrol 2 times" → {"type": "patrol_laps", "parameters": {"count": 2}}

**MISSION STRUCTURE:**
A composite mission has multiple STEPS executed sequentially or conditionally.

**STEP TYPES:**

1. **explore_area**: Autonomous exploration with SLAM
   Parameters: {duration: seconds, coverage: "full"|"partial"}
   Example: "Explore for 30 seconds"

2. **follow_target**: Track and follow detected object
   Parameters: {target_class: "person"|"bottle"|etc, min_distance: meters, max_distance: meters}
   Example: "Follow the person at 2 meters distance"

3. **patrol_laps**: Complete N laps in pattern
   Parameters: {count: int, shape: "circle"|"square", radius: meters (optional)}
   Example: "Patrol 5 laps in a circle"

4. **condition_check**: Evaluate condition and branch
   Parameters: {
     condition: {
       type: "object_detected"|"timeout"|"distance_traveled",
       target_class: string (for object_detected),
       timeout: seconds
     },
     branch_true: step_id (if condition met),
     branch_false: step_id (if condition not met)
   }
   Example: "If you see a bottle, approach it, otherwise continue exploring"

5. **directional_command**: Simple movement directive
   Parameters: {direction: "left"|"right"|"forward"|"backward", distance: meters OR duration: seconds}
   Example: "Turn left", "Go forward 3 meters"

**CONDITIONAL LOGIC:**
- Use condition_check step for branching
- Conditions: object_detected, timeout, distance_traveled
- Each condition has branch_true and branch_false

**CONSTRAINTS:**
- avoid_objects: ["chair", "table"] - objects to avoid during execution
- max_speed: float - speed limit for mission
- priority: "safety"|"speed"|"coverage" - optimization goal

**OUTPUT FORMAT (JSON only):**
{
  "mission_type": "composite_mission",
  "description": "Brief summary",
  "steps": [
    {
      "id": "step_1",
      "type": "explore_area",
      "parameters": {"duration": 60},
      "next_step_if_success": "step_2"
    },
    {
      "id": "step_2",
      "type": "condition_check",
      "parameters": {
        "condition": {
          "type": "object_detected",
          "target_class": "person",
          "timeout": 5
        },
        "branch_true": "step_3",
        "branch_false": "step_4"
      }
    },
    {
      "id": "step_3",
      "type": "follow_target",
      "parameters": {"target_class": "person", "min_distance": 2.0},
      "next_step_if_success": "mission_complete"
    }
  ],
  "fallback_strategy": {
    "on_stuck": "pause_and_retry",
    "on_error": "emergency_stop"
  },
  "global_constraints": {
    "avoid_objects": [],
    "max_speed": 0.6
  }
}

**EXAMPLES:**

Input: "Explore for 30 seconds, if you find a person follow them, otherwise patrol 5 laps"
Output:
{
  "mission_type": "composite_mission",
  "description": "Explore → conditional follow/patrol",
  "steps": [
    {"id": "explore", "type": "explore_area", "parameters": {"duration": 30}, "next_step_if_success": "check_person"},
    {"id": "check_person", "type": "condition_check", "parameters": {"condition": {"type": "object_detected", "target_class": "person", "timeout": 5}, "branch_true": "follow", "branch_false": "patrol"}},
    {"id": "follow", "type": "follow_target", "parameters": {"target_class": "person", "min_distance": 2.0, "max_distance": 3.0}, "next_step_if_success": "mission_complete"},
    {"id": "patrol", "type": "patrol_laps", "parameters": {"count": 5, "shape": "circle"}, "next_step_if_success": "mission_complete"}
  ],
  "fallback_strategy": {"on_stuck": "pause_and_retry", "on_error": "emergency_stop"}
}

Input: "Go forward 3 meters, turn left, then explore"
Output:
{
  "mission_type": "composite_mission",
  "description": "Directional sequence → explore",
  "steps": [
    {"id": "forward", "type": "directional_command", "parameters": {"direction": "forward", "distance": 3.0}, "next_step_if_success": "turn"},
    {"id": "turn", "type": "directional_command", "parameters": {"direction": "left", "duration": 2.0}, "next_step_if_success": "explore"},
    {"id": "explore", "type": "explore_area", "parameters": {"duration": 60, "coverage": "full"}, "next_step_if_success": "mission_complete"}
  ],
  "fallback_strategy": {"on_stuck": "pause_and_retry"}
}

**IMPORTANT:**
- Always assign unique step IDs
- Link steps with next_step_if_success or branch_true/false
- Include fallback_strategy
- Use "mission_complete" as final next_step
- If ambiguous, make reasonable assumptions (can be clarified later)
"""
    
    async def parse(self, user_prompt: str, builder) -> CompositeMissionConfig:
        """
        Parse natural language into composite mission config.
        
        Args:
            user_prompt: Natural language command
            builder: Workflow builder (for LLM config)
        
        Returns:
            CompositeMissionConfig: Structured mission
        
        Raises:
            CompositeParsingError: If parsing fails
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
        
        Args:
            mission_dict: Parsed JSON from LLM
        
        Returns:
            CompositeMissionConfig: Validated config object
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
        
        Args:
            config: Mission config to validate
        
        Raises:
            CompositeParsingError: If validation fails
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
        
        Args:
            user_prompt: User input
        
        Returns:
            bool: True if likely composite mission
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


# =============================================================================
# Custom Exceptions
# =============================================================================

class CompositeParsingError(Exception):
    """Raised when composite mission parsing fails."""
    pass