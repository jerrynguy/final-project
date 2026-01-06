"""
ACE (Agentic Context Engineering) Module

Post-mission learning system using LLM to analyze logs and suggest
parameter improvements.

Components:
- LogBuffer: Real-time log collection
- LogAnalyzer: Format logs for LLM analysis
- ACELearner: LLM-based learning system
- ParameterManager: Safe parameter adjustment with rollback
"""

from multi_function_agent._robot_vision_controller.core.ace.log_analyzer import (
    LogBuffer,
    LogAnalyzer,
    AbortEvent,
    MissionSummary
)

from multi_function_agent._robot_vision_controller.core.ace.ace_learner import (
    ACELearner
)

from multi_function_agent._robot_vision_controller.core.ace.parameter_manager import (
    ParameterManager,
    ParameterAdjustment
)

__all__ = [
    'LogBuffer',
    'LogAnalyzer',
    'AbortEvent',
    'MissionSummary',
    'ACELearner',
    'ParameterManager',
    'ParameterAdjustment',
]