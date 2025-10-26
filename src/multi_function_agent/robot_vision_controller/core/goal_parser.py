"""
Goal Parser Module
Parses natural language commands into structured Mission objects using LLM.
"""

import logging
import json
from typing import Dict, Optional
from dataclasses import dataclass, asdict
from langchain_nvidia_ai_endpoints import ChatNVIDIA

logger = logging.getLogger(__name__)


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
    """
    try:
        # Build system prompt with mission specifications
        system_prompt = """You are a robot mission parser. Convert natural language commands into structured JSON.

OUTPUT FORMAT (JSON only, no explanation):
{
  "type": "follow_target|patrol_laps|explore_area",
  "target_class": "person|bottle|chair|cup|car|etc (COCO class names, null if not applicable)",
  "parameters": {},
  "description": "brief description"
}

MISSION TYPES:   
1. follow_target: Track and follow a moving target (like a dog following owner)
   Parameters: {"min_distance": float (meters), "max_distance": float, "predict_on_lost": bool}
   
2. patrol_laps: Complete N laps in pattern
   Parameters: {"count": int, "shape": "circle|square|corridor"}
   
3. explore_area: General exploration
   Parameters: {"duration": int (seconds), "coverage": "full|partial"}

EXAMPLES:
"Follow the person in front of you" → {"type":"follow_target","target_class":"person","parameters":{"min_distance":1.0,"max_distance":2.5,"predict_on_lost":true},"description":"Follow moving person"}

"Go around 20 times" → {"type":"patrol_laps","target_class":null,"parameters":{"count":20,"shape":"circle"},"description":"Complete 20 circular laps"}

"Explore freely" → {"type":"explore_area","target_class":null,"parameters":{"coverage":"full"},"description":"Free exploration"}"""

        user_message = f"Parse this command:\n{user_prompt}"
        
        # Initialize LLM
        llm = ChatNVIDIA(
            model="meta/llama-3.1-70b-instruct",
            temperature=0.0,
            max_tokens=100
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
        
        # Create Mission object from parsed data
        mission = Mission(
            type=mission_dict['type'],
            target_class=mission_dict.get('target_class'),
            parameters=mission_dict.get('parameters', {}),
            description=mission_dict.get('description', '')
        )
        
        logger.info(f"[GOAL PARSER] Parsed mission: {mission.type} - {mission.description}")
        return mission
        
    except Exception as e:
        logger.error(f"Mission parsing failed: {e}")
        
        # Fallback to explore mode on parsing failure
        logger.warning("Falling back to explore mode")
        return Mission(
            type='explore_area',
            parameters={'coverage': 'full'},
            description='Free exploration (fallback)'
        )