"""
Composite Mission Module
State machine executor for multi-step missions with conditional branching.
"""

import os
import time
import logging
from enum import Enum
from typing import Dict, List, Optional, Any

from multi_function_agent._robot_vision_controller.core.mission_controller.missions.base_mission import (
    BaseMission, MissionConfig
)
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.explore_mission import ExploreMission
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.follow_mission import FollowMission
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.patrol_mission import PatrolMission

logger = logging.getLogger(__name__)


class MissionTransitionError(Exception):
    """Raised when transition between steps fails validation."""
    pass

# Mission State Enumeration
class CompositeState(Enum):
    """States for composite mission execution."""
    INIT = "init"
    EXECUTING = "executing"
    CHECKING_CONDITION = "checking_condition"
    TRANSITIONING = "transitioning"
    COMPLETE = "complete"
    ERROR = "error"


# =============================================================================
# Step Executors
# =============================================================================

class StepExecutor:
    """Base class for step execution."""
    
    def __init__(self, step_config: Dict):
        self.config = step_config
        self.start_time = None
        self.completed = False
    
    def start(self):
        """Start step execution."""
        self.start_time = time.time()
        self.completed = False
    
    def update(self, detected_objects, robot_pos, frame_info) -> Dict:
        """Update step state. Override in subclasses."""
        raise NotImplementedError
    
    def check_completion(self) -> bool:
        """Check if step is complete. Override in subclasses."""
        raise NotImplementedError
    
    def get_directive(self) -> str:
        """Get navigation directive. Override in subclasses."""
        raise NotImplementedError


class DirectionalCommandExecutor(StepExecutor):
    """Executor for simple directional commands."""
    
    def __init__(self, step_config: Dict):
        super().__init__(step_config)
        self.direction = step_config['parameters'].get('direction', 'forward')
        self.distance = step_config['parameters'].get('distance')
        self.duration = step_config['parameters'].get('duration', 2.0)
        
        self.start_position = None
        self.distance_traveled = 0.0
    
    def start(self):
        super().start()
        logger.info(f"[DIRECTIONAL] Starting: {self.direction}, duration={self.duration}s")
    
    def update(self, detected_objects, robot_pos, frame_info) -> Dict:
        """Track movement progress."""
        if robot_pos and self.start_position is None:
            self.start_position = (robot_pos['x'], robot_pos['y'])
        
        # Calculate distance traveled
        if robot_pos and self.start_position:
            import numpy as np
            dx = robot_pos['x'] - self.start_position[0]
            dy = robot_pos['y'] - self.start_position[1]
            self.distance_traveled = np.sqrt(dx**2 + dy**2)
        
        return {
            'distance_traveled': self.distance_traveled,
            'direction': self.direction
        }
    
    def check_completion(self) -> bool:
        """Check if command completed."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        # Complete by distance (if specified)
        if self.distance and self.distance_traveled >= self.distance:
            logger.info(f"[DIRECTIONAL] Complete: {self.distance_traveled:.2f}m traveled")
            return True
        
        # Complete by duration
        if elapsed >= self.duration:
            logger.info(f"[DIRECTIONAL] Complete: {elapsed:.1f}s elapsed")
            return True
        
        return False
    
    def get_directive(self) -> str:
        """Get navigation directive."""
        return f'directional_{self.direction}'


class ConditionCheckExecutor(StepExecutor):
    """Executor for conditional branching."""
    
    def __init__(self, step_config: Dict):
        super().__init__(step_config)
        self.condition = step_config['parameters']['condition']
        self.branch_true = step_config['parameters']['branch_true']
        self.branch_false = step_config['parameters']['branch_false']
        self.timeout = self.condition.get('timeout', 30.0)
        
        self.condition_met = None
        self.check_start_time = None
    
    def start(self):
        super().start()
        self.check_start_time = time.time()
        logger.info(f"[CONDITION] Checking: {self.condition['type']}")
    
    def update(self, detected_objects, robot_pos, frame_info) -> Dict:
        """Evaluate condition."""
        condition_type = self.condition['type']
        
        if condition_type == 'object_detected':
            target_class = self.condition.get('target_class')
            
            if detected_objects:
                # Check if target detected
                detected = any(
                    obj.get('class') == target_class 
                    for obj in detected_objects
                )
                
                if detected:
                    self.condition_met = True
                    logger.info(f"[CONDITION] ✓ Detected: {target_class}")
                    return {'condition_met': True, 'target_class': target_class}
        
        elif condition_type == 'timeout':
            # Timeout condition always returns false (let timeout handle it)
            pass
        
        elif condition_type == 'distance_traveled':
            threshold = self.condition.get('distance', 5.0)
            # Would need distance tracking
            pass
        
        # Check timeout
        elapsed = time.time() - self.check_start_time
        if elapsed >= self.timeout:
            self.condition_met = False
            logger.info(f"[CONDITION] ✗ Timeout after {elapsed:.1f}s")
        
        return {'condition_met': self.condition_met}
    
    def check_completion(self) -> bool:
        """Condition check completes when result determined."""
        return self.condition_met is not None
    
    def get_next_step(self) -> str:
        """Get next step ID based on condition result."""
        if self.condition_met:
            return self.branch_true
        else:
            return self.branch_false
    
    def get_directive(self) -> str:
        """Directive during condition check."""
        return 'condition_checking'


# =============================================================================
# Composite Mission
# =============================================================================

class CompositeMission(BaseMission):
    """
    Execute multi-step missions with conditional logic.
    
    State Machine:
    INIT → EXECUTING → CHECKING_CONDITION → TRANSITIONING → EXECUTING → ... → COMPLETE
    """
    
    def __init__(self, config: MissionConfig):
        """Initialize composite mission."""
        # Extract composite config FIRST
        self.composite_config = config.parameters.get('composite_config')
        if not self.composite_config:
            raise ValueError("Composite mission requires composite_config")
        
        self._pre_validate_requirements()

        # Initialize attributes BEFORE super().__init__()
        self.steps = {step.id: step for step in self.composite_config.steps}
        self.current_step_id = self.composite_config.steps[0].id
        self.current_executor = None
        self.sub_mission = None
        self.execution_context = {}
        self.composite_state = CompositeState.INIT
        
        # NOW call parent constructor
        super().__init__(config)
        
        logger.info(
            f"[COMPOSITE] Initialized: {len(self.steps)} steps, "
            f"starting with '{self.current_step_id}'"
        )

    def _pre_validate_requirements(self):
        """
        Pre-validate requirements for ALL steps before mission starts.
        Ensures SLAM/map setup is done BEFORE control loop begins.
        """
        from multi_function_agent._robot_vision_controller.core.mission_controller.mission_validator.mission_validator import (
            MissionValidator,
            MissionRequirementsError
        )
        
        for step in self.composite_config.steps:
            step_type = step.type
            
            # Check explore steps (need SLAM confirmation)
            if step_type == 'explore_area':
                logger.info(f"[PRE-VALIDATION] Step '{step.id}' requires SLAM")
                try:
                    MissionValidator.validate_explore_mission()
                    logger.info(f"[PRE-VALIDATION] ✅ SLAM confirmed for '{step.id}'")
                except MissionRequirementsError as e:
                    raise MissionTransitionError(
                        f"Composite mission cannot start: Step '{step.id}' requires SLAM. {e}"
                    )
            
            # Check patrol steps (need map)
            elif step_type == 'patrol_laps':
                logger.info(f"[PRE-VALIDATION] Step '{step.id}' requires map")
                try:
                    self._validate_patrol_requirements()
                    logger.info(f"[PRE-VALIDATION] ✅ Map found for '{step.id}'")
                except MissionRequirementsError as e:
                    # Map might not exist yet (will be created by explore step)
                    # Check if explore step comes before patrol
                    explore_steps = [s for s in self.composite_config.steps if s.type == 'explore_area']
                    if not explore_steps:
                        raise MissionTransitionError(
                            f"Step '{step.id}' requires map but no explore step found. {e}"
                        )
                    else:
                        logger.warning(f"[PRE-VALIDATION] Map not found yet - will be created by explore step")

    def _validate_step_requirements(self, step_id: str):
        """
        Validate requirements for a specific step before starting.
        
        Args:
            step_id: ID of step to validate
            
        Raises:
            MissionTransitionError: If requirements not met
        """
        step_config = self.steps[step_id]
        step_type = step_config.type
        
        logger.info(f"[VALIDATION] Checking requirements for step '{step_id}' ({step_type})")
        
        # Import validator
        from multi_function_agent._robot_vision_controller.core.mission_controller.mission_validator.mission_validator import (
            MissionValidator,
            MissionRequirementsError
        )
        
        try:
            if step_type == 'explore_area':
                # Validate SLAM availability
                MissionValidator.validate_explore_mission()
            
            elif step_type == 'patrol_laps':
                # Validate map exists
                self._validate_patrol_requirements()
            
            elif step_type == 'follow_target':
                # Validate target class
                target_class = step_config.parameters.get('target_class')
                MissionValidator.validate_follow_mission(target_class=target_class)
            
            logger.info(f"[VALIDATION] ✅ Step '{step_id}' requirements met")
            
        except MissionRequirementsError as e:
            raise MissionTransitionError(
                f"Step '{step_id}' ({step_type}) requirements not met: {e}"
            )
    
    def _validate_explore_completion(self):
        """
        Validate explore step completed successfully with map saved.
        
        Raises:
            MissionTransitionError: If map not found or invalid
        """
        # Check map files exist
        map_path = os.path.expanduser("~/my_map")
        yaml_file = f"{map_path}.yaml"
        pgm_file = f"{map_path}.pgm"
        
        if not (os.path.exists(yaml_file) and os.path.exists(pgm_file)):
            raise MissionTransitionError(
                "Explore step completed but map not found. "
                "SLAM may have failed or not been started. "
                f"Expected files: {yaml_file}, {pgm_file}"
            )
        
        # Validate file sizes (sanity check)
        yaml_size = os.path.getsize(yaml_file)
        pgm_size = os.path.getsize(pgm_file)
        
        if yaml_size < 100 or pgm_size < 1000:
            raise MissionTransitionError(
                f"Map files too small (YAML={yaml_size}B, PGM={pgm_size}B). "
                f"Map may be incomplete or corrupted."
            )
        
        logger.info(
            f"[VALIDATION] ✅ Explore map validated: "
            f"YAML={yaml_size}B, PGM={pgm_size}B"
        )
    
    def _validate_patrol_requirements(self):
        """
        Validate patrol step has required map file.
        
        Raises:
            MissionTransitionError: If map not found
        """
        map_path = os.path.expanduser("~/my_map")
        yaml_file = f"{map_path}.yaml"
        
        if not os.path.exists(yaml_file):
            raise MissionTransitionError(
                f"Patrol requires map file at {yaml_file}. "
                f"Run explore mission first or create map manually:\n"
                f"  1. Start SLAM: ros2 launch slam_toolbox online_async_launch.py\n"
                f"  2. Drive around: ros2 run turtlebot3_teleop teleop_keyboard\n"
                f"  3. Save map: ros2 run nav2_map_server map_saver_cli -f ~/my_map"
            )
        
        logger.info(f"[VALIDATION] ✅ Patrol map found: {yaml_file}")
    
    def _initialize_state(self) -> Dict:
        """Initialize composite mission state."""
        return {
            'composite_state': CompositeState.INIT.value,
            'current_step_id': self.current_step_id,
            'current_step_type': self.steps[self.current_step_id].type,
            'completed_steps': [],
            'execution_context': {}
        }
    
    def _update_state(
        self,
        detected_objects: List[Dict] = None,
        robot_pos: Dict = None,
        frame_info: Dict = None,
        frame = None,
        vision_analyzer = None,
        full_lidar_scan = None
    ) -> Dict:
        """Execute state machine logic."""
        
        if self.composite_state == CompositeState.INIT:
            self._start_current_step()
            self.composite_state = CompositeState.EXECUTING
        
        elif self.composite_state == CompositeState.EXECUTING:
            # Execute current step
            step_result = self._execute_current_step(
                detected_objects, robot_pos, frame_info,
                frame, vision_analyzer, full_lidar_scan
            )
            
            # Check completion
            if self._is_current_step_complete():
                self.state['completed_steps'].append(self.current_step_id)
                
                # Determine next step
                next_step_id = self._determine_next_step()
                
                if next_step_id == 'mission_complete':
                    self.composite_state = CompositeState.COMPLETE
                    logger.info("[COMPOSITE] Mission complete ✓")
                else:
                    self.composite_state = CompositeState.TRANSITIONING
        
        elif self.composite_state == CompositeState.TRANSITIONING:
            # Transition to next step
            next_step_id = self._determine_next_step()
            self._transition_to_step(next_step_id)
            self.composite_state = CompositeState.EXECUTING
        
        elif self.composite_state == CompositeState.COMPLETE:
            # Mission finished
            self.state['completed'] = True
        
        # Update state info
        self.state['composite_state'] = self.composite_state.value
        self.state['current_step_id'] = self.current_step_id
        self.state['current_step_type'] = self.steps[self.current_step_id].type
        self.state['progress'] = len(self.state['completed_steps']) / len(self.steps)
        
        return self.state
    
    def _start_current_step(self):
        """Start execution of current step."""
        step_config = self.steps[self.current_step_id]
        step_type = step_config.type
        
        logger.info(f"[COMPOSITE] Starting step '{self.current_step_id}' ({step_type})")
        
        try:
            self._validate_step_requirements(self.current_step_id)
        except MissionTransitionError as e:
            logger.error(f"[COMPOSITE] Step validation failed: {e}")
            self.composite_state = CompositeState.ERROR
            raise

        # MODIFIED: Import and use dedicated step classes
        if step_type in ['explore_area', 'follow_target', 'patrol_laps']:
            # Delegate to existing mission classes
            self._start_sub_mission(step_config)
        
        elif step_type == 'directional_command':
            # UPDATED: Use DirectionalCommandMission instead of executor
            from multi_function_agent._robot_vision_controller.core.mission_controller.missions.directional_command_step import DirectionalCommandMission
            
            mission_config = MissionConfig(
                type='directional_command',
                description=f"Directional: {step_config.parameters.get('direction')}",
                parameters=step_config.parameters
            )
            self.sub_mission = DirectionalCommandMission(mission_config)
            logger.info("[COMPOSITE] Directional command sub-mission started")
        
        elif step_type == 'condition_check':
            # UPDATED: Use ConditionCheckMission instead of executor
            from multi_function_agent._robot_vision_controller.core.mission_controller.missions.condition_check_step import ConditionCheckMission
            
            mission_config = MissionConfig(
                type='condition_check',
                description=f"Condition: {step_config.parameters.get('condition', {}).get('type')}",
                parameters=step_config.parameters
            )
            self.sub_mission = ConditionCheckMission(mission_config)
            logger.info("[COMPOSITE] Condition check sub-mission started")
        
        else:
            logger.error(f"Unknown step type: {step_type}")
            self.composite_state = CompositeState.ERROR

    def _start_sub_mission(self, step_config):
        """Start a sub-mission (explore/follow/patrol)."""
        step_type = step_config.type
        
        # Create mission config
        mission_config = MissionConfig(
            type=step_type,
            description=f"Composite step: {step_config.id}",
            parameters=step_config.parameters,
            target_class=step_config.parameters.get('target_class')
        )
        
        # Instantiate mission
        if step_type == 'explore_area':
            self.sub_mission = ExploreMission(mission_config)
        elif step_type == 'follow_target':
            self.sub_mission = FollowMission(mission_config)
        elif step_type == 'patrol_laps':
            self.sub_mission = PatrolMission(mission_config)
        
        logger.info(f"[COMPOSITE] Sub-mission started: {step_type}")
    
    def _execute_current_step(
        self,
        detected_objects,
        robot_pos,
        frame_info,
        frame,
        vision_analyzer,
        full_lidar_scan
    ) -> Dict:
        """Execute current step and return result."""
        
        # Execute sub-mission
        if self.sub_mission:
            return self.sub_mission.process_frame(
                detected_objects, robot_pos, frame_info,
                frame, vision_analyzer, full_lidar_scan
            )
        
        # Execute custom executor
        elif self.current_executor:
            return self.current_executor.update(
                detected_objects, robot_pos, frame_info
            )
        
        return {}
    
    def _is_current_step_complete(self) -> bool:
        """Check if current step is complete."""
        
        if self.sub_mission:
            return self.sub_mission.check_completion()
        
        elif self.current_executor:
            return self.current_executor.check_completion()
        
        return False
    
    def _determine_next_step(self) -> str:
        """Determine next step ID based on current step result."""
        current_step = self.steps[self.current_step_id]

        from multi_function_agent._robot_vision_controller.core.mission_controller.missions.condition_check_step import ConditionCheckMission
        
        # MODIFIED: Check if condition check step
        if isinstance(self.sub_mission, ConditionCheckMission):
            next_step = self.sub_mission.get_next_step_id()
            logger.info(f"[COMPOSITE] Condition → next: '{next_step}'")
            return next_step
        
        # Default: use next_step_if_success
        next_step = current_step.next_step_if_success
        
        if not next_step:
            # No explicit next step → mission complete
            return 'mission_complete'
        
        return next_step
    
    def _transition_to_step(self, step_id: str):
        """Transition to next step WITH POST-VALIDATION."""
        current_step = self.steps[self.current_step_id]
        
        logger.info(f"[COMPOSITE] Transition: '{self.current_step_id}' → '{step_id}'")
        
        # Post-step validation
        try:
            if current_step.type == 'explore_area':
                # Just finished explore → ensure map saved
                logger.info("[COMPOSITE] Validating explore completion...")
                self._validate_explore_completion()
        
        except MissionTransitionError as e:
            logger.error(f"[COMPOSITE] Post-step validation failed: {e}")
            self.composite_state = CompositeState.ERROR
            raise
        
        # Cleanup current step
        self.sub_mission = None
        self.current_executor = None
        
        # Set new step
        self.current_step_id = step_id
        
        # Start next step (with pre-validation inside _start_current_step)
        self._start_current_step()
    
    def _check_completion(self) -> bool:
        """Check if entire composite mission is complete."""
        return self.composite_state == CompositeState.COMPLETE
    
    def _get_directive(self) -> str:
        """Get current navigation directive."""
        
        # Delegate to sub-mission
        if self.sub_mission:
            return self.sub_mission.get_directive()
        
        # Delegate to executor
        elif self.current_executor:
            return self.current_executor.get_directive()
        
        # Fallback
        return 'composite_executing'