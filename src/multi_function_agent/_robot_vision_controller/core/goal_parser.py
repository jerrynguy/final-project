"""
Goal Parser Module
Parses natural language commands into structured Mission objects using LLM.
"""

import logging
import json
from typing import Dict, Optional
from dataclasses import dataclass, asdict
from langchain_nvidia_ai_endpoints import ChatNVIDIA

# ADDED: Import composite parser
from multi_function_agent._robot_vision_controller.core.composite_mission_parser import (
    CompositeMissionParser,
    CompositeParsingError
)

logger = logging.getLogger(__name__)


# =============================================================================
# Custom Exceptions
# =============================================================================

class MissionParsingError(Exception):
    """Raised when mission cannot be parsed from user prompt."""
    pass


class UnsupportedMissionError(Exception):
    """Raised when mission type is not supported."""
    pass


# =============================================================================
# Mission Data Structure
# =============================================================================

@dataclass
class Mission:
    """
    Structured mission object representing robot tasks.
    """
    type: str
    target_class: Optional[str] = None
    parameters: Dict = None
    description: str = ""
    
    def __post_init__(self):
        """Initialize default parameters if not provided."""
        if self.parameters is None:
            self.parameters = {}
    
    def to_dict(self) -> Dict:
        """
        Convert mission to dictionary representation.
        """
        return asdict(self)


# =============================================================================
# Mission Parser
# =============================================================================

async def parse_mission_from_prompt(user_prompt: str, builder) -> Mission:
    """
    Parse natural language command into structured Mission object using LLM.
    
    Raises:
        MissionParsingError: If LLM fails to parse the prompt
        UnsupportedMissionError: If mission type is not recognized
    """
    try:
        # Check if composite mission
        composite_parser = CompositeMissionParser()

        if composite_parser.is_composite_mission(user_prompt):
            logger.info("[GOAL PARSER] Detected composite mission")
            
            try:
                # Parse as composite
                composite_config = await composite_parser.parse(user_prompt, builder)
                
                # Wrap in Mission object
                mission = Mission(
                    type='composite_mission',
                    target_class=None,
                    parameters={'composite_config': composite_config},
                    description=composite_config.description
                )
                
                logger.info(
                    f"[GOAL PARSER] Composite mission: "
                    f"{len(composite_config.steps)} steps"
                )
                return mission
                
            except CompositeParsingError as e:
                logger.warning(f"Composite parsing failed: {e}, falling back to simple")
                # Fallback to simple parsing

        # Load system prompt from file
        from pathlib import Path
        prompt_file = Path(__file__).parent / "text" / "mission_parser_prompt.txt"
        
        try:
            with open(prompt_file, 'r') as f:
                system_prompt = f.read().strip()
        except FileNotFoundError:
            logger.warning(f"Prompt file not found: {prompt_file}, using fallback")
            # Fallback to minimal prompt if file missing
            system_prompt = "You are a robot mission parser. Convert natural language to JSON with fields: type, target_class, parameters, description. Mission types: follow_target, patrol_laps, explore_area."

        user_message = f"Parse this command:\n{user_prompt}"
        
        # Get LLM config from builder (reuse workflow config)
        try:
            llm_config = builder._workflow_builder.general_config.llms.get('nim_llm')
            model_name = llm_config.model_name if llm_config else "meta/llama-3.1-70b-instruct"
            temperature = llm_config.temperature if llm_config else 0.0
        except:
            # Fallback if config not accessible
            model_name = "meta/llama-3.1-70b-instruct"
            temperature = 0.0
            logger.warning("Using fallback LLM config")
        
        # Initialize LLM with config
        llm = ChatNVIDIA(
            model=model_name,
            temperature=temperature,
            max_tokens=100  # Keep short for mission parsing
        )
        
        # Prepare messages with system prompt
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        # Call LLM
        response = await llm.ainvoke(messages)
        response_text = response.content.strip()
        
        logger.info(f"[GOAL PARSER] LLM response: {response_text}")
        
        # Parse JSON response
        # Remove markdown code blocks if present
        if '```json' in response_text:
            response_text = response_text.split('```json')[1].split('```')[0].strip()
        elif '```' in response_text:
            response_text = response_text.split('```')[1].split('```')[0].strip()
        
        mission_dict = json.loads(response_text)
        
        # Validate mission type
        mission_type = mission_dict.get('type', '').lower()
        
        # Check if mission is unsupported
        if mission_type == 'unsupported' or mission_type not in ['follow_target', 'patrol_laps', 'explore_area']:
            raise UnsupportedMissionError(
                f"Mission type '{mission_type}' is not supported. "
                f"Available missions: follow_target, patrol_laps, explore_area"
            )
        
        # Create Mission object from parsed data
        mission = Mission(
            type=mission_type,
            target_class=mission_dict.get('target_class'),
            parameters=mission_dict.get('parameters', {}),
            description=mission_dict.get('description', '')
        )
        
        logger.info(f"[GOAL PARSER] Parsed mission: {mission.type} - {mission.description}")
        return mission
        
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse LLM JSON response: {e}")
        raise MissionParsingError(f"Could not parse mission from prompt: Invalid JSON response")
    
    except UnsupportedMissionError:
        # Re-raise to be handled by caller
        raise
    
    except Exception as e:
        logger.error(f"Mission parsing failed: {e}")
        raise MissionParsingError(f"Could not parse mission from prompt: {str(e)}")