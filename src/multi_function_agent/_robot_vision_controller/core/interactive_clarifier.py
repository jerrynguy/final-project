"""
Interactive Mission Clarifier Module
Handles ambiguous missions through dialogue with user.
"""

import json
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from langchain_nvidia_ai_endpoints import ChatNVIDIA

logger = logging.getLogger(__name__)


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class Ambiguity:
    """Represents an ambiguity in mission specification."""
    type: str  # missing_parameter, multiple_options, unclear_intent, conflicting_conditions
    field: str  # Which parameter is ambiguous
    options: List[str] = field(default_factory=list)  # Possible values
    context: str = ""  # Additional context
    severity: str = "medium"  # low, medium, high


@dataclass
class ClarificationQuestion:
    """A question to ask the user."""
    question_text: str
    ambiguity: Ambiguity
    suggested_answers: List[str] = field(default_factory=list)
    question_type: str = "choice"  # choice, open_ended, yes_no


# =============================================================================
# Interactive Clarifier
# =============================================================================

class InteractiveMissionClarifier:
    """
    Detect ambiguities in parsed missions and generate clarifying questions.
    
    Features:
    - Ambiguity detection (missing params, multiple interpretations)
    - LLM-powered question generation
    - Context-aware re-parsing
    - Conversation history tracking
    """
    
    def __init__(self):
        """Initialize clarifier."""
        self.llm = None  # Lazy loaded
        self.conversation_history = []
        self.clarification_context = {}
    
    async def analyze_mission(
        self,
        user_prompt: str,
        parsed_mission: Dict,
        context: Dict
    ) -> Optional[ClarificationQuestion]:
        """
        Analyze mission for ambiguities.
        
        Args:
            user_prompt: Original user input
            parsed_mission: Parsed mission dict from LLM
            context: Current context (detections, map, etc.)
        
        Returns:
            ClarificationQuestion if ambiguous, None if clear
        """
        logger.info("[CLARIFIER] Analyzing mission for ambiguities...")
        
        # Detect ambiguities
        ambiguities = self._detect_ambiguities(parsed_mission, context)
        
        if not ambiguities:
            logger.info("[CLARIFIER] No ambiguities detected ✓")
            return None
        
        # Prioritize by severity
        ambiguities.sort(key=lambda a: {'high': 0, 'medium': 1, 'low': 2}[a.severity])
        
        # Generate question for highest priority ambiguity
        question = await self._generate_question(ambiguities[0], context)
        
        logger.info(f"[CLARIFIER] Question generated: '{question.question_text}'")
        return question
    
    def _detect_ambiguities(
        self,
        parsed_mission: Dict,
        context: Dict
    ) -> List[Ambiguity]:
        """
        Detect ambiguities in parsed mission.
        
        Returns:
            List of detected ambiguities
        """
        ambiguities = []
        
        # Check if composite mission
        is_composite = parsed_mission.get('mission_type') == 'composite_mission'
        
        if is_composite:
            steps = parsed_mission.get('steps', [])
            
            for step in steps:
                step_type = step.get('type')
                params = step.get('parameters', {})
                
                # Check follow_target ambiguities
                if step_type == 'follow_target':
                    ambiguities.extend(self._check_follow_ambiguities(params, context))
                
                # Check directional command ambiguities
                elif step_type == 'directional_command':
                    ambiguities.extend(self._check_directional_ambiguities(params))
                
                # Check condition ambiguities
                elif step_type == 'condition_check':
                    ambiguities.extend(self._check_condition_ambiguities(params, context))
        
        else:
            # Simple mission
            mission_type = parsed_mission.get('type')
            params = parsed_mission.get('parameters', {})
            
            if mission_type == 'follow_target':
                ambiguities.extend(self._check_follow_ambiguities(params, context))
            
            elif mission_type == 'patrol_laps':
                ambiguities.extend(self._check_patrol_ambiguities(params))
        
        return ambiguities
    
    def _check_follow_ambiguities(
        self,
        params: Dict,
        context: Dict
    ) -> List[Ambiguity]:
        """Check ambiguities in follow_target mission."""
        ambiguities = []
        
        # Check if target class specified
        target_class = params.get('target_class')
        if not target_class:
            ambiguities.append(Ambiguity(
                type='missing_parameter',
                field='target_class',
                context='No target specified',
                severity='high'
            ))
            return ambiguities
        
        # Check if multiple targets detected
        detected_objects = context.get('detected_objects', [])
        matching_objects = [
            obj for obj in detected_objects 
            if obj.get('class') == target_class
        ]
        
        if len(matching_objects) > 1:
            ambiguities.append(Ambiguity(
                type='multiple_options',
                field='target_selection',
                options=[f"Object {i+1}" for i in range(len(matching_objects))],
                context=f"Detected {len(matching_objects)} {target_class}(s)",
                severity='high'
            ))
        
        # Check if distance not specified
        if 'min_distance' not in params or 'max_distance' not in params:
            ambiguities.append(Ambiguity(
                type='missing_parameter',
                field='follow_distance',
                options=['1.0m', '2.0m', '3.0m'],
                context='Follow distance not specified',
                severity='medium'
            ))
        
        return ambiguities
    
    def _check_directional_ambiguities(self, params: Dict) -> List[Ambiguity]:
        """Check ambiguities in directional commands."""
        ambiguities = []
        
        direction = params.get('direction')
        distance = params.get('distance')
        duration = params.get('duration')
        
        # Both distance and duration missing
        if not distance and not duration:
            ambiguities.append(Ambiguity(
                type='missing_parameter',
                field='movement_extent',
                options=['2 seconds', '3 meters', 'until obstacle'],
                context=f"How long to move {direction}?",
                severity='medium'
            ))
        
        return ambiguities
    
    def _check_condition_ambiguities(
        self,
        params: Dict,
        context: Dict
    ) -> List[Ambiguity]:
        """Check ambiguities in condition checks."""
        ambiguities = []
        
        condition = params.get('condition', {})
        condition_type = condition.get('type')
        
        # Check object_detected condition
        if condition_type == 'object_detected':
            target_class = condition.get('target_class')
            
            if not target_class:
                ambiguities.append(Ambiguity(
                    type='missing_parameter',
                    field='condition_target',
                    context='What object to detect?',
                    severity='high'
                ))
        
        # Check timeout
        timeout = condition.get('timeout')
        if not timeout or timeout > 120:
            ambiguities.append(Ambiguity(
                type='missing_parameter',
                field='condition_timeout',
                options=['10 seconds', '30 seconds', '60 seconds'],
                context='How long to wait for condition?',
                severity='low'
            ))
        
        return ambiguities
    
    def _check_patrol_ambiguities(self, params: Dict) -> List[Ambiguity]:
        """Check ambiguities in patrol missions."""
        ambiguities = []
        
        # Check lap count
        count = params.get('count')
        if not count or count > 20:
            ambiguities.append(Ambiguity(
                type='missing_parameter',
                field='patrol_count',
                options=['3 laps', '5 laps', '10 laps'],
                context='How many patrol laps?',
                severity='medium'
            ))
        
        return ambiguities
    
    async def _generate_question(
        self,
        ambiguity: Ambiguity,
        context: Dict
    ) -> ClarificationQuestion:
        """
        Generate natural language question for ambiguity.
        
        Args:
            ambiguity: Detected ambiguity
            context: Current context
        
        Returns:
            ClarificationQuestion with human-friendly text
        """
        # Initialize LLM if needed
        if self.llm is None:
            self.llm = ChatNVIDIA(
                model="meta/llama-3.1-70b-instruct",
                temperature=0.3,
                max_tokens=150
            )
        
        # Build prompt
        prompt = f"""Generate a clear, concise clarifying question for this ambiguity:

Ambiguity Type: {ambiguity.type}
Field: {ambiguity.field}
Context: {ambiguity.context}
Options: {ambiguity.options}

Requirements:
- Ask ONE specific question
- Be conversational and friendly
- Include suggested options if available
- Keep it under 2 sentences

Output format:
Question: <your question>
Type: choice|open_ended|yes_no
Suggestions: <comma-separated options>
"""
        
        try:
            response = await self.llm.ainvoke(prompt)
            response_text = response.content.strip()
            
            # Parse response
            question_text = self._extract_field(response_text, "Question")
            question_type = self._extract_field(response_text, "Type") or "choice"
            suggestions_str = self._extract_field(response_text, "Suggestions")
            
            suggestions = []
            if suggestions_str:
                suggestions = [s.strip() for s in suggestions_str.split(',')]
            
            return ClarificationQuestion(
                question_text=question_text or f"Please specify {ambiguity.field}",
                ambiguity=ambiguity,
                suggested_answers=suggestions or ambiguity.options,
                question_type=question_type
            )
            
        except Exception as e:
            logger.error(f"Question generation failed: {e}")
            
            # Fallback to template-based question
            return self._generate_fallback_question(ambiguity)
    
    def _extract_field(self, text: str, field_name: str) -> Optional[str]:
        """Extract field from LLM response."""
        lines = text.split('\n')
        for line in lines:
            if line.startswith(f"{field_name}:"):
                return line.split(':', 1)[1].strip()
        return None
    
    def _generate_fallback_question(self, ambiguity: Ambiguity) -> ClarificationQuestion:
        """Generate template-based question as fallback."""
        
        templates = {
            'missing_parameter': f"Please specify {ambiguity.field}",
            'multiple_options': f"Which {ambiguity.field} should I choose?",
            'unclear_intent': f"What do you mean by {ambiguity.field}?",
            'conflicting_conditions': f"How should I handle {ambiguity.field}?"
        }
        
        question_text = templates.get(
            ambiguity.type, 
            f"Please clarify: {ambiguity.context}"
        )
        
        return ClarificationQuestion(
            question_text=question_text,
            ambiguity=ambiguity,
            suggested_answers=ambiguity.options,
            question_type='choice' if ambiguity.options else 'open_ended'
        )
    
    async def reparse_with_context(
        self,
        original_prompt: str,
        user_response: str,
        previous_parse: Dict,
        parser
    ) -> Dict:
        """
        Re-parse mission with clarification context.
        
        Args:
            original_prompt: Original user input
            user_response: User's answer to clarification
            previous_parse: Previously parsed mission
            parser: Mission parser to use
        
        Returns:
            Updated mission dict
        """
        logger.info(
            f"[CLARIFIER] Re-parsing with context: "
            f"'{user_response}'"
        )
        
        # Build enhanced prompt with context
        enhanced_prompt = f"""Original request: {original_prompt}

Additional clarification: {user_response}

Please parse the complete mission considering both the original request and the clarification."""
        
        # Store in conversation history
        self.conversation_history.append({
            'original': original_prompt,
            'clarification': user_response
        })
        
        # Re-parse
        try:
            if hasattr(parser, 'parse'):
                # Composite parser
                config = await parser.parse(enhanced_prompt, None)
                return config
            else:
                # Simple parser
                # Would need to modify simple parser to accept context
                return previous_parse
                
        except Exception as e:
            logger.error(f"Re-parsing failed: {e}")
            # Return original parse
            return previous_parse


# =============================================================================
# Custom Exceptions
# =============================================================================

class ClarificationError(Exception):
    """Raised when clarification process fails."""
    pass