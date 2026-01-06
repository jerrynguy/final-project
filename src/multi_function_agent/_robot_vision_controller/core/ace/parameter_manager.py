"""
ACE Parameter Manager Module
Quản lý việc load/save và validate learned parameters.
"""

import json
import logging
from typing import Dict, Any, Tuple, Optional
from pathlib import Path
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


@dataclass
class ParameterAdjustment:
    """Một điều chỉnh parameter được học."""
    parameter: str
    old_value: float
    new_value: float
    reasoning: str
    confidence: float
    applied_at: float  # timestamp


class ParameterManager:
    """
    Quản lý learned parameters với safety validation.
    
    Features:
    - Load/save learned parameters
    - Validate against safety bounds
    - Rollback mechanism
    - History tracking
    """
    
    # Safety bounds (IMMUTABLE)
    SAFETY_BOUNDS = {
        'CRITICAL_ABORT': (0.15, 0.30),
        'CRITICAL_ABORT_FRONT': (0.15, 0.30),
        'CRITICAL_ABORT_SIDE': (0.10, 0.25),
        'WARNING_ZONE': (0.30, 0.70),
        'CAUTION_ZONE': (0.50, 1.20),
        'SAFE_ZONE': (0.70, 2.00),
        'RESUME_SAFE': (0.40, 1.00),
        'escape_duration': (2.0, 10.0),
    }
    
    # Parameters cho phép điều chỉnh
    ADJUSTABLE_PARAMS = set(SAFETY_BOUNDS.keys())
    
    def __init__(self, learned_params_file: Optional[str] = None):
        """
        Initialize parameter manager.
        
        Args:
            learned_params_file: Path to learned_parameters.json
                                If None, uses default location
        """
        if learned_params_file is None:
            # Default: src/multi_function_agent/configs/learned_parameters.json
            base_dir = Path(__file__).parent.parent.parent
            learned_params_file = str(base_dir / "configs" / "learned_parameters.json")
        
        self.learned_params_file = Path(learned_params_file)
        self.adjustment_history = []
        
        # Ensure config dir exists
        self.learned_params_file.parent.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"[PARAM MGR] Using file: {self.learned_params_file}")
    
    def validate_adjustment(
        self,
        param_name: str,
        new_value: float
    ) -> Tuple[bool, str]:
        """
        Validate xem adjustment có an toàn không.
        
        Returns:
            (is_valid: bool, reason: str)
        """
        # Check if parameter allowed
        if param_name not in self.ADJUSTABLE_PARAMS:
            return False, f"Parameter '{param_name}' not adjustable"
        
        # Check bounds
        bounds = self.SAFETY_BOUNDS.get(param_name)
        if not bounds:
            return False, f"No bounds defined for '{param_name}'"
        
        min_val, max_val = bounds
        
        if not (min_val <= new_value <= max_val):
            return False, f"Value {new_value} out of bounds [{min_val}, {max_val}]"
        
        return True, "OK"
    
    def apply_adjustments(
        self,
        recommendations: list
    ) -> bool:
        """
        Apply list of parameter adjustments.
        
        Args:
            recommendations: List of dicts from ACE learner
                            Each with: parameter, suggested_value, reasoning
        
        Returns:
            Success status
        """
        import time
        
        applied = []
        
        for rec in recommendations:
            param_name = rec['parameter']
            new_value = rec['suggested_value']
            
            # Validate
            is_valid, reason = self.validate_adjustment(param_name, new_value)
            
            if not is_valid:
                logger.error(f"[PARAM MGR] Cannot apply {param_name}: {reason}")
                continue
            
            # Get old value
            old_value = self._get_current_value(param_name)
            
            # Create adjustment record
            adjustment = ParameterAdjustment(
                parameter=param_name,
                old_value=old_value,
                new_value=new_value,
                reasoning=rec.get('reasoning', 'No reason provided'),
                confidence=rec.get('confidence', 0.0),
                applied_at=time.time()
            )
            
            self.adjustment_history.append(adjustment)
            applied.append(adjustment)
            
            logger.info(
                f"[PARAM MGR] ✅ Applied: {param_name} "
                f"{old_value:.3f} → {new_value:.3f}"
            )
        
        if not applied:
            logger.warning("[PARAM MGR] No adjustments applied")
            return False
        
        # Update SafetyThresholds in runtime
        self._update_runtime_thresholds(applied)
        
        return True
    
    def _get_current_value(self, param_name: str) -> float:
        """Get current value of parameter from SafetyThresholds."""
        from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds
        
        return getattr(SafetyThresholds, param_name, 0.0)
    
    def _update_runtime_thresholds(self, adjustments: list):
        """
        Update SafetyThresholds class attributes in runtime.
        
        ⚠️ WARNING: This modifies global state!
        """
        from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds
        
        for adjustment in adjustments:
            setattr(SafetyThresholds, adjustment.parameter, adjustment.new_value)
            logger.info(
                f"[PARAM MGR] Runtime updated: "
                f"SafetyThresholds.{adjustment.parameter} = {adjustment.new_value:.3f}"
            )
    
    def save_to_file(self):
        """
        Save learned parameters to JSON file.
        
        Format:
        {
          "version": 1,
          "last_updated": timestamp,
          "adjustments": [
            {
              "parameter": "CRITICAL_ABORT",
              "value": 0.25,
              "reasoning": "...",
              "confidence": 0.85,
              "applied_at": timestamp
            }
          ]
        }
        """
        import time
        
        # Get latest values for each parameter
        latest_adjustments = {}
        for adj in self.adjustment_history:
            latest_adjustments[adj.parameter] = adj
        
        data = {
            "version": 1,
            "last_updated": time.time(),
            "adjustments": [
                {
                    "parameter": adj.parameter,
                    "value": adj.new_value,
                    "reasoning": adj.reasoning,
                    "confidence": adj.confidence,
                    "applied_at": adj.applied_at
                }
                for adj in latest_adjustments.values()
            ]
        }
        
        with open(self.learned_params_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        logger.info(f"[PARAM MGR] ✅ Saved to {self.learned_params_file}")
    
    def load_from_file(self) -> bool:
        """
        Load learned parameters from file và apply.
        
        Returns:
            Success status
        """
        if not self.learned_params_file.exists():
            logger.info("[PARAM MGR] No learned parameters file found")
            return False
        
        try:
            with open(self.learned_params_file, 'r') as f:
                data = json.load(f)
            
            adjustments = data.get('adjustments', [])
            
            if not adjustments:
                logger.info("[PARAM MGR] No adjustments in file")
                return False
            
            logger.info(
                f"[PARAM MGR] Loading {len(adjustments)} learned parameters..."
            )
            
            # Apply each adjustment
            for adj_data in adjustments:
                param_name = adj_data['parameter']
                new_value = adj_data['value']
                
                # Validate
                is_valid, reason = self.validate_adjustment(param_name, new_value)
                
                if not is_valid:
                    logger.warning(
                        f"[PARAM MGR] Skipping {param_name}: {reason}"
                    )
                    continue
                
                # Apply
                from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds
                
                old_value = getattr(SafetyThresholds, param_name, 0.0)
                setattr(SafetyThresholds, param_name, new_value)
                
                logger.info(
                    f"[PARAM MGR] ✅ Loaded: {param_name} "
                    f"{old_value:.3f} → {new_value:.3f}"
                )
            
            return True
            
        except Exception as e:
            logger.error(f"[PARAM MGR] Failed to load: {e}")
            return False
    
    def get_adjustment_history(self) -> list:
        """Get all adjustments made in current session."""
        return [asdict(adj) for adj in self.adjustment_history]
    
    def rollback_last(self) -> bool:
        """
        Rollback last adjustment.
        
        Returns:
            Success status
        """
        if not self.adjustment_history:
            logger.warning("[PARAM MGR] No adjustments to rollback")
            return False
        
        last_adj = self.adjustment_history.pop()
        
        # Restore old value
        from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyThresholds
        
        setattr(SafetyThresholds, last_adj.parameter, last_adj.old_value)
        
        logger.info(
            f"[PARAM MGR] ⏮️  Rolled back: {last_adj.parameter} "
            f"{last_adj.new_value:.3f} → {last_adj.old_value:.3f}"
        )
        
        # Update file
        self.save_to_file()
        
        return True
    
    def rollback_all(self):
        """Rollback all adjustments made in session."""
        while self.adjustment_history:
            self.rollback_last()
        
        logger.info("[PARAM MGR] ⏮️  All adjustments rolled back")