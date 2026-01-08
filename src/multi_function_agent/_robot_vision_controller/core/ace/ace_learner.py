"""
ACE Learner Module
Sử dụng LLM để phân tích logs và đề xuất cải tiến parameters.
"""

import json
import logging
import asyncio
from typing import Dict, Any, Optional
from pathlib import Path
from langchain_nvidia_ai_endpoints import ChatNVIDIA

from multi_function_agent._robot_vision_controller.core.ace.log_analyzer import (
    MissionSummary,
    LogAnalyzer
)

logger = logging.getLogger(__name__)


class ACELearner:
    """
    Agentic Context Engineering - Learning System.
    
    Sử dụng LLM để tự động phân tích mission logs và đề xuất
    parameter adjustments cho lần chạy tiếp theo.
    """
    
    def __init__(self, llm_config: Optional[Dict] = None):
        """
        Initialize ACE learner.
        
        Args:
            llm_config: LLM configuration (from builder)
                       If None, will use defaults
        """
        self.llm_config = llm_config or {
            'model_name': 'meta/llama-3.1-70b-instruct',
            'temperature': 0.0,
            'max_tokens': 1500
        }
        
        self._llm = None
        self._system_prompt = self._load_system_prompt()
        
        logger.info("[ACE] Learner initialized with LLM config")
    
    def _load_system_prompt(self) -> str:
        """Load ACE analysis prompt từ file."""
        prompt_file = Path(__file__).parent.parent / "text" / "ace_analysis_prompt.txt"
        
        try:
            with open(prompt_file, 'r') as f:
                return f.read().strip()
        except FileNotFoundError:
            logger.warning(f"ACE prompt file not found: {prompt_file}, using fallback")
            return self._get_fallback_prompt()
    
    def _get_fallback_prompt(self) -> str:
        """Fallback prompt nếu file không tồn tại."""
        return """You are a robotics debugging assistant.
        
Analyze the mission logs and identify issues.
Return JSON with:
- diagnosis: Problem description
- root_cause: Specific cause
- recommendations: List of parameter adjustments

Focus on safety parameter issues causing "death pendulum" (oscillation/spinning).
"""
    
    def _initialize_llm(self):
        """Lazy initialize LLM."""
        if self._llm is None:
            self._llm = ChatNVIDIA(
                model=self.llm_config['model_name'],
                temperature=self.llm_config['temperature'],
                max_tokens=self.llm_config['max_tokens']
            )
            logger.info(f"[ACE] LLM initialized: {self.llm_config['model_name']}")
    
    async def analyze_mission(
        self,
        mission_summary: MissionSummary
    ) -> Optional[Dict[str, Any]]:
        """
        Phân tích mission logs bằng LLM.
        
        Args:
            mission_summary: Mission summary từ LogBuffer
        
        Returns:
            Dict with diagnosis và recommendations, hoặc None nếu thất bại
        """
        try:
            logger.info("[ACE] Starting mission analysis...")
            
            # Format logs cho LLM
            formatted_logs = LogAnalyzer.format_for_llm(mission_summary)
            
            # Thêm current parameters
            current_params = LogAnalyzer.extract_current_parameters()
            params_str = json.dumps(current_params, indent=2)
            
            user_message = f"""{formatted_logs}

=== CURRENT PARAMETERS ===
{params_str}

Please analyze the above mission logs and provide your diagnosis with parameter adjustment recommendations.
"""
            
            logger.info(f"[ACE] Formatted logs: {len(formatted_logs)} chars")
            logger.info(f"[ACE] Sending to LLM for analysis...")
            
            # Initialize LLM if needed
            self._initialize_llm()
            
            # Prepare messages
            messages = [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": user_message}
            ]
            
            # Call LLM with timeout
            response = await asyncio.wait_for(
                self._llm.ainvoke(messages),
                timeout=130.0
            )
            
            response_text = response.content.strip()
            
            logger.info(f"[ACE] LLM response received: {len(response_text)} chars")
            
            # Parse JSON response
            analysis = self._parse_llm_response(response_text)
            
            if analysis:
                logger.info("[ACE] ✅ Analysis complete")
                logger.info(f"[ACE] Diagnosis: {analysis['diagnosis']}")
                logger.info(f"[ACE] Recommendations: {len(analysis.get('recommendations', []))}")
                
                return analysis
            else:
                logger.error("[ACE] ❌ Failed to parse LLM response")
                return None
                
        except asyncio.TimeoutError:
            logger.error("[ACE] ❌ LLM timeout after 130s")
            return None
        
        except Exception as e:
            logger.error(f"[ACE] ❌ Analysis failed: {e}")
            return None
    
    def _parse_llm_response(self, response_text: str) -> Optional[Dict]:
        """
        Parse LLM JSON response.
        
        Handles markdown code blocks và malformed JSON.
        """
        try:
            # Remove markdown code blocks nếu có
            if '```json' in response_text:
                response_text = response_text.split('```json')[1].split('```')[0].strip()
            elif '```' in response_text:
                response_text = response_text.split('```')[1].split('```')[0].strip()
            
            # Parse JSON
            analysis = json.loads(response_text)
            
            # Validate structure
            required_fields = ['diagnosis', 'root_cause', 'recommendations']
            if not all(field in analysis for field in required_fields):
                logger.error(f"[ACE] Missing required fields in response")
                return None
            
            return analysis
            
        except json.JSONDecodeError as e:
            logger.error(f"[ACE] JSON parse error: {e}")
            logger.error(f"[ACE] Raw response: {response_text[:200]}")
            return None
    
    def validate_recommendations(
        self,
        recommendations: list
    ) -> tuple[list, list]:
        """
        Validate recommendations against safety bounds.
        
        Returns:
            (valid_recommendations, rejected_recommendations)
        """
        from multi_function_agent._robot_vision_controller.core.ace.parameter_manager import (
            ParameterManager
        )
        
        manager = ParameterManager()
        
        valid = []
        rejected = []
        
        for rec in recommendations:
            param_name = rec.get('parameter')
            suggested_value = rec.get('suggested_value')
            
            is_safe, reason = manager.validate_adjustment(param_name, suggested_value)
            
            if is_safe:
                valid.append(rec)
                logger.info(f"[ACE] ✅ Validated: {param_name} = {suggested_value}")
            else:
                rejected.append({
                    'recommendation': rec,
                    'rejection_reason': reason
                })
                logger.warning(f"[ACE] ❌ Rejected: {param_name} = {suggested_value} ({reason})")
        
        return valid, rejected
    
    async def learn_from_mission(
        self,
        mission_summary: MissionSummary,
        auto_apply: bool = False
    ) -> Optional[Dict]:
        """
        Complete learning workflow:
        1. Analyze logs
        2. Validate recommendations
        3. Optionally apply adjustments
        
        Args:
            mission_summary: Mission summary
            auto_apply: Tự động apply adjustments (default: False)
        
        Returns:
            Learning result với analysis + validation status
        """
        logger.info("=" * 60)
        logger.info("[ACE] POST-MISSION LEARNING STARTED")
        logger.info("=" * 60)
        
        # Step 1: Analyze
        analysis = await self.analyze_mission(mission_summary)
        
        if not analysis:
            logger.error("[ACE] Analysis failed, aborting learning")
            return None
        
        # Step 2: Validate
        recommendations = analysis.get('recommendations', [])
        
        if not recommendations:
            logger.info("[ACE] No recommendations provided")
            return {
                'analysis': analysis,
                'valid_recommendations': [],
                'rejected_recommendations': [],
                'applied': False
            }
        
        valid, rejected = self.validate_recommendations(recommendations)
        
        result = {
            'analysis': analysis,
            'valid_recommendations': valid,
            'rejected_recommendations': rejected,
            'applied': False
        }
        
        # Step 3: Apply if requested
        if auto_apply and valid:
            from multi_function_agent._robot_vision_controller.core.ace.parameter_manager import (
                ParameterManager
            )
            
            manager = ParameterManager()
            
            logger.info(f"[ACE] Applying {len(valid)} adjustments...")
            
            applied = manager.apply_adjustments(valid)
            result['applied'] = applied
            
            if applied:
                logger.info("[ACE] ✅ Adjustments applied successfully")
                manager.save_to_file()
            else:
                logger.error("[ACE] ❌ Failed to apply adjustments")
        
        logger.info("=" * 60)
        logger.info("[ACE] POST-MISSION LEARNING COMPLETE")
        logger.info("=" * 60)
        
        return result