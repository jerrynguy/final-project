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
        """Convert mission to dictionary representation."""
        return asdict(self)


# =============================================================================
# Mission Parser
# =============================================================================

async def parse_mission_from_prompt(user_prompt: str, builder) -> Mission:
    """
    Parse natural language command into structured Mission object using LLM.
    If the mission type is unrecognized or unsupported, return NOT_APPROVED instead of defaulting to explore_area.
    """
    try:
        # Build system prompt with mission specifications
        system_prompt = """You are a robot mission parser. Convert natural language commands into structured JSON.

OUTPUT FORMAT (JSON only, no explanation):
{
  "type": "follow_target|patrol_laps|explore_area|not_approved",
  "target_class": "person|bottle|chair|cup|car|etc (COCO class names, null if not applicable)",
  "parameters": {},
  "description": "brief description"
}

MISSION TYPES:   
1. follow_target: Track and follow a moving target (like a dog following owner)
   Parameters: {"min_distance": float (meters), "max_distance": float, "predict_on_lost": bool}
   
2. patrol_laps: Complete N laps in pattern
   Parameters: {"count": int, "shape": "circle|square|corridor"}
   
3. explore_area: General exploration (requires SLAM)
   Parameters: {"duration": int (seconds), "coverage": "full|partial"}

If the command does not clearly fit any of these, or refers to an unavailable feature, 
return a JSON with:
{"type":"not_approved","target_class":null,"parameters":{},"description":"Mission not approved or feature not available."}

EXAMPLES:
"Follow the person in front of you" → {"type":"follow_target","target_class":"person","parameters":{"min_distance":1.0,"max_distance":2.5,"predict_on_lost":true},"description":"Follow moving person"}
"Patrol the warehouse" → {"type":"patrol_laps","target_class":null,"parameters":{"count":3,"shape":"square"},"description":"Patrol warehouse in square route"}
"Explore freely" → {"type":"explore_area","target_class":null,"parameters":{"coverage":"full"},"description":"Free exploration"}
"Sing a song" → {"type":"not_approved","target_class":null,"parameters":{},"description":"Mission not approved or unsupported"}"""

        user_message = f"Parse this command:\n{user_prompt}"
        
        # Initialize LLM
        llm = ChatNVIDIA(
            model="meta/llama-3.1-70b-instruct",
            temperature=0.0,
            max_tokens=120
        )
        
        # Prepare messages
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        # Invoke LLM
        response = await llm.ainvoke(messages)
        response_text = response.content.strip()
        logger.info(f"[GOAL PARSER] LLM raw response: {response_text}")
        
        # Clean JSON if wrapped in markdown
        if '```json' in response_text:
            response_text = response_text.split('```json')[1].split('```')[0].strip()
        elif '```' in response_text:
            response_text = response_text.split('```')[1].split('```')[0].strip()
        
        mission_dict = json.loads(response_text)
        mission_type = mission_dict.get('type', '').lower()

        # Enforce supported missions only
        supported = {'follow_target', 'patrol_laps', 'explore_area'}
        if mission_type not in supported:
            logger.warning(f"[GOAL PARSER] Unsupported mission type '{mission_type}' detected — marking as NOT_APPROVED.")
            return Mission(
                type='not_approved',
                description='Mission not approved or feature not available.',
                parameters={},
                target_class=None
            )

        # Construct mission object
        mission = Mission(
            type=mission_type,
            target_class=mission_dict.get('target_class'),
            parameters=mission_dict.get('parameters', {}),
            description=mission_dict.get('description', '')
        )
        logger.info(f"[GOAL PARSER] Parsed mission: {mission.type} - {mission.description}")
        return mission
        
    except Exception as e:
        logger.error(f"Mission parsing failed: {e}")
        
        # Fallback: explicit NOT_APPROVED (instead of explore_area)
        logger.warning("Parsing failed — returning NOT_APPROVED mission.")
        return Mission(
            type='not_approved',
            parameters={},
            description='Mission not approved due to parsing error.',
            target_class=None
        )
