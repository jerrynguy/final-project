import json
import numpy as np
from pydantic import Field
import time
import logging
import asyncio
from typing import AsyncGenerator, Dict, Any, Optional, Tuple

from multi_function_agent._robot_vision_controller.utils.log.error_handlers import ErrorHandlers
from multi_function_agent._robot_vision_controller.utils.log.output_formatter import OutputFormatter
from multi_function_agent._robot_vision_controller.utils.log.performance_logger import PerformanceLogger
from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyValidator, SafetyThresholds

from multi_function_agent._robot_vision_controller.perception.slam_controller import SLAMController
from multi_function_agent._robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
from multi_function_agent._robot_vision_controller.perception.rtsp_stream_handler import RTSPStreamHandler
from multi_function_agent._robot_vision_controller.perception.robot_vision_analyzer import RobotVisionAnalyzer

from multi_function_agent._robot_vision_controller.navigation.navigation_reasoner import NavigationReasoner
from multi_function_agent._robot_vision_controller.navigation.robot_controller_interface import RobotControllerInterface
try:
    from multi_function_agent._robot_vision_controller.navigation.nav2_interface import NavigationState
    NAV2_STATE_AVAILABLE = True
except ImportError:
    NavigationState = None
    NAV2_STATE_AVAILABLE = False

from multi_function_agent._robot_vision_controller.core.query_extractor import QueryExtractor
from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
from multi_function_agent._robot_vision_controller.core.mission_controller.mission_controller import (
    MissionController,
    MissionRequirementsError
)
from multi_function_agent._robot_vision_controller.core.mission_controller.missions.composite_mission import (
    MissionTransitionError 
)
from multi_function_agent._robot_vision_controller.core.parser.goal_parser import (
    MissionParsingError,
    UnsupportedMissionError
)
from multi_function_agent._robot_vision_controller.core.models import (
    get_robot_vision_model_manager,
    preload_robot_vision_model,
    is_yolo_ready,
)
from multi_function_agent._robot_vision_controller.core.ace.log_analyzer import (
    LogBuffer, LogAnalyzer
)
from multi_function_agent._robot_vision_controller.core.ace.ace_learner import ACELearner
from multi_function_agent._robot_vision_controller.core.ace.parameter_manager import ParameterManager
from multi_function_agent.register import RobotVisionConfig

from aiq.builder.builder import Builder # type: ignore
from aiq.builder.function_info import FunctionInfo # type: ignore
from aiq.cli.register_workflow import register_function # type: ignore
from aiq.data_models.function import FunctionBaseConfig # type: ignore

try:
    import rclpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

logger = logging.getLogger(__name__)

# Global Initialization
stream_handler = RTSPStreamHandler()
safety_validator = SafetyValidator()
model_manager = get_robot_vision_model_manager()
query_extractor = QueryExtractor()

logger.info("Initializing ROS2 node...")
ros2_node = get_ros2_node()
logger.info("✅ ROS2 node ready")

logger.info("Starting robot vision model preloading...")
preload_success = preload_robot_vision_model()

if preload_success:
    logger.info("✅ Robot vision model preloaded")
else:
    logger.error("❌ Failed to preload robot vision model")

# Main Controller Function
async def _robot_vision_controller(
    config: RobotVisionConfig, 
    builder: Builder
) -> AsyncGenerator[FunctionInfo, None]:
    """Main robot vision controller function."""
    
    robot_config = query_extractor.debugging_robot_config_resolution(config, builder, logger)
    stream_url = robot_config["stream_url"]
    control_mode = robot_config["control_mode"]
    navigation_goal = robot_config["navigation_goal"]
    safety_level = robot_config["safety_level"]
    
    if not stream_url or not stream_url.strip():
        yield FunctionInfo.from_fn(
            ErrorHandlers.no_stream_url(),
            description="No stream URL provided"
        )
        return
    
    is_stream_valid, stream_error = await stream_handler.validate_stream(stream_url)
    if not is_stream_valid:
        yield FunctionInfo.from_fn(
            ErrorHandlers.invalid_stream(stream_url, stream_error),
            description="Invalid stream connection"
        )
        return
    
    try:
        logger.info(f"🎯 Starting: {control_mode} | Goal: {navigation_goal} | Safety: {safety_level}")
        
        navigation_reasoner = NavigationReasoner(
            safety_level=safety_level, 
            max_speed=robot_config["max_speed"]
        )

        exploration_boost = robot_config.get("exploration_speed_boost", 1.0)
        navigation_reasoner.set_exploration_boost(exploration_boost)

        robot_interface = RobotControllerInterface()
        vision_analyzer = RobotVisionAnalyzer(robot_controller=robot_interface)
        safety_monitor = LidarSafetyMonitor()

        logger.info("✅ AI Recovery system initialized for explore mode")

        nav2_ready = False
        if robot_interface.use_nav2:
            nav2_ready = await robot_interface.wait_for_nav2_ready(timeout=10.0)
            logger.info(f"Nav2: {'✅ Ready' if nav2_ready else '⚠️ Fallback to manual'}")
        
        connection_success = await robot_interface.connect()
        if not connection_success:
            logger.warning("⚠️ Robot controller connection failed")

        # Startup safety check
        initial_scan = robot_interface.lidar_data
        if initial_scan:
            startup_monitor = LidarSafetyMonitor()
            min_dist = startup_monitor.get_min_distance(initial_scan)
            if min_dist < SafetyThresholds.WARNING_ZONE:
                logger.warning(f"[STARTUP] Backing away from obstacle ({min_dist:.2f}m)")
                back_away_cmd = {
                    "action": "move_backward",
                    "parameters": {
                        "linear_velocity": -0.2,
                        "angular_velocity": 0.0,
                        "duration": 2.0
                    },
                    "reason": "startup_back_away"
                }
                await robot_interface.execute_command(back_away_cmd)
                await asyncio.sleep(0.5)

        # Parse mission
        user_prompt = builder._workflow_builder.general_config.front_end.input_query[0]
        try:
            # Parse mission (might be composite)
            mission_controller = await MissionController.from_prompt(user_prompt, builder)
            parsed_mission = mission_controller.mission.to_dict()
            
            # Get current context for clarification
            # Capture frame for object detection context
            await stream_handler.start_stream(stream_url)
            await asyncio.sleep(1.0)  # Wait for first frame
            
            frame = await stream_handler.get_latest_frame()
            context = {
                'detected_objects': [],
                'slam_map': None,
                'robot_pos': None
            }
            
            # Detect objects if YOLO ready
            if frame is not None and is_yolo_ready():
                # Quick detection for context (non-blocking)
                try:
                    detected = vision_analyzer.detect_target_objects(
                        frame, 
                        'person',  # Default check for common objects
                        confidence_threshold=0.6
                    )
                    context['detected_objects'] = detected
                except Exception as e:
                    logger.debug(f"Context detection skipped: {e}")

        except UnsupportedMissionError as e:
            error_msg = (
                f"❌ Mission Not Supported\n\n"
                f"{str(e)}\n\n"
                f"Supported missions:\n"
                f"  • follow_target: 'Follow the person'\n"
                f"  • patrol_laps: 'Patrol 5 laps'\n"
                f"  • explore_area: 'Explore freely'\n"
            )
            logger.error(error_msg)
            yield FunctionInfo.from_fn(lambda x: error_msg, description="Unsupported mission")
            return
        
        except MissionRequirementsError as e:
            logger.error(f"❌ Mission requirements not met: {e}")
            yield FunctionInfo.from_fn(lambda x: str(e), description="Requirements not met")
            return
        
        except MissionParsingError as e:
            logger.error(f"❌ Mission parsing failed: {e}")
            yield FunctionInfo.from_fn(lambda x: str(e), description="Parsing failed")
            return
    
        # Create ACE components
        log_buffer = LogBuffer(max_size=200)
        
        # Get LLM config from builder (reuse workflow config)
        try:
            llm_config = builder._workflow_builder.general_config.llms.get('nim_llm')
            ace_llm_config = {
                'model_name': llm_config.model_name if llm_config else "meta/llama-3.1-70b-instruct",
                'temperature': 0.0,  # ACE needs deterministic output
                'max_tokens': 1500
            }
        except:
            # Fallback if config not accessible
            ace_llm_config = {
                'model_name': "meta/llama-3.1-70b-instruct",
                'temperature': 0.0,
                'max_tokens': 1500
            }
            logger.warning("[ACE] Using fallback LLM config")
        
        ace_learner = ACELearner(llm_config=ace_llm_config)
        param_manager = ParameterManager()
        
        # Load learned parameters from previous missions
        logger.info("=" * 60)
        logger.info("[ACE] CHECKING FOR LEARNED PARAMETERS")
        logger.info("=" * 60)
        
        loaded = param_manager.load_from_file()
        if loaded:
            logger.info("[ACE] ✅ Applied previous learning")
        else:
            logger.info("[ACE] ℹ️  No previous learning found, using defaults")
        
        logger.info("=" * 60)
        
        # Set mission info in log buffer
        log_buffer.set_mission_info(
            mission_type=mission_controller.mission.type,
            description=mission_controller.mission.description
        )
        
        # Initialize SLAM for explore mission
        slam_controller = None
        if mission_controller.has_explore_step():
            logger.info("[SLAM] Starting mapping (composite mission aware)...")
            slam_controller = SLAMController()
            
            slam_started = slam_controller.start_slam()
            if not slam_started:
                error_msg = "❌ SLAM Failed to Start"
                logger.error(error_msg)
                yield FunctionInfo.from_fn(lambda x: error_msg, description="SLAM failed")
                return
        
        control_start_time = time.time()
        control_results = await run_robot_control_loop(
            stream_url,
            vision_analyzer,
            navigation_reasoner,
            robot_interface,
            navigation_goal,
            safety_validator,
            safety_monitor,
            mission_controller,
            slam_controller=slam_controller,
            nav2_ready=nav2_ready,
            max_iterations=None,
            log_buffer=log_buffer,
            ace_learner=ace_learner
        )

        # Stop SLAM and save map
        if slam_controller and slam_controller.is_running:
            slam_controller.stop_slam()
        
        # Format output
        formatted_output = OutputFormatter.format_control_results(
            control_results, 
            stream_url, 
            control_mode, 
            mission_controller.mission
        )

        if slam_controller:
            slam_stats = slam_controller.get_mapping_stats()
            formatted_output += f"\n\n=== SLAM Mapping Results ===\n"
            formatted_output += f"Duration: {slam_stats['duration']:.1f}s\n"
            formatted_output += f"Map saved: {slam_stats['map_exists']}\n"
        
        control_time = time.time() - control_start_time
        model_manager.update_inference_time(control_time)
        
        print("\n===== ROBOT VISION CONTROL OUTPUT =====\n")
        print(formatted_output)
        
        async def success_control(dummy: str) -> str:
            return formatted_output
        
        try:
            yield FunctionInfo.from_fn(
                success_control,
                description=f"Robot vision control completed: {control_mode} mode",
            )
        finally:
            logger.info("✅ Mission completed")
                    
    except Exception as e:
        logger.error(f"❌ Control error: {e}")
        yield FunctionInfo.from_fn(
            ErrorHandlers.control_error(e, stream_url, is_yolo_ready()),
            description="Robot control failed"
        )
        return

def _mission_directive_to_nav2_goal(
    directive: str,
    robot_pos: Dict,
    vision_analysis: Dict
) -> Optional[Tuple[float, float, float]]:
    """Convert mission directive to Nav2 goal (x, y, theta)."""
    if robot_pos is None:
        return None
    
    current_x = robot_pos['x']
    current_y = robot_pos['y']
    current_theta = robot_pos.get('theta', 0.0)
    
    import numpy as np
    
    def relative_to_absolute(distance: float, angle_offset: float = 0.0):
        target_theta = current_theta + angle_offset
        goal_x = current_x + distance * np.cos(target_theta)
        goal_y = current_y + distance * np.sin(target_theta)
        return (goal_x, goal_y, target_theta)
    
    if directive == 'track_follow':
        return relative_to_absolute(1.0, 0.0)
    elif directive == 'track_approach':
        return relative_to_absolute(0.5, 0.0)
    elif directive.startswith('track_search_'):
        search_dir = directive.split('_')[-1]
        if search_dir == 'left':
            return relative_to_absolute(1.5, np.pi/2)
        elif search_dir == 'right':
            return relative_to_absolute(1.5, -np.pi/2)
        elif search_dir == 'forward':
            return relative_to_absolute(1.5, 0.0)
        return None
    elif directive.startswith('patrol_'):
        return relative_to_absolute(1.0, 0.3)
    elif directive.startswith('explore_'):
        import random
        angle_offset = random.uniform(-np.pi, np.pi)
        distance = random.uniform(2.0, 3.0)
        return relative_to_absolute(distance, angle_offset)
    
    return None

# Control Loop
async def run_robot_control_loop(
    stream_url: str,
    vision_analyzer: RobotVisionAnalyzer,
    navigation_reasoner: NavigationReasoner,
    robot_interface: RobotControllerInterface,
    navigation_goal: str,
    safety_validator: SafetyValidator,
    safety_monitor: LidarSafetyMonitor,
    mission_controller: MissionController, 
    slam_controller: Optional[SLAMController] = None,
    nav2_ready: bool = False,
    max_iterations: int = None,
    log_buffer: Optional['LogBuffer'] = None,  
    ace_learner: Optional['ACELearner'] = None 
) -> Dict[str, Any]:
    """Execute main robot control loop."""
    
    results = {
        "iterations": 0,
        "commands_sent": [],
        "obstacles_detected": [],
        "navigation_decisions": [],
        "final_status": "unknown",
    }
    
    stream_started = await stream_handler.start_stream(stream_url)
    if not stream_started:
        raise ValueError("Could not start stream")
    
    logger.info(f"🎯 Mission started: {mission_controller.mission.description}")
    
    iteration = 0
    last_vision_update = 0
    cached_vision_analysis = None

    navigation_decision = {
        'action': 'init',
        'parameters': {'linear_velocity': 0.0, 'angular_velocity': 0.0, 'duration': 0.1},
        'reason': 'initialization'
    }

    while max_iterations is None or iteration < max_iterations:
        try:
            # STEP 0: Frame Acquisition
            lidar_snapshot = robot_interface.lidar_data
            frame = await stream_handler.get_latest_frame()
            if frame is None:
                await asyncio.sleep(0.5)
                continue
            
            iteration += 1
            PerformanceLogger.log_iteration_start(iteration)

            # Log robot position for tracking
            robot_pos = robot_interface.ros_node.get_robot_pose()
            if robot_pos:
                logger.info(
                    f"[POSITION] x={robot_pos['x']:.3f}m, "
                    f"y={robot_pos['y']:.3f}m, "
                    f"yaw={robot_pos['theta']:.3f}rad ({np.degrees(robot_pos['theta']):.1f}°)"
                )
            else:
                logger.warning("[POSITION] No odometry data available")

            # STEP 1: Critical Abort Check (SINGLE LAYER)
            abort_result = safety_monitor.check_critical_abort(
                lidar_snapshot,
                robot_pos=robot_pos
            )

            if abort_result['abort']:
                logger.error(
                    f"[CRITICAL ABORT] Obstacle at {abort_result['min_distance']:.3f}m"
                )

                # Handle deadlock/timeout from escape system
                if abort_result.get('request_nav2_rescue'):
                    logger.error("=" * 60)
                    logger.error("[DEADLOCK DETECTED] Cannot escape locally")
                    logger.error("=" * 60)
                    logger.error(
                        f"Obstacle clearances: {abort_result.get('clearances', {})}"
                    )
                    logger.error("Mission will abort - manual intervention may be needed")
                    
                    results["final_status"] = "deadlock_cannot_escape"
                    break

                await robot_interface.execute_command(abort_result['command'])
                results["navigation_decisions"].append({
                    'action': 'critical_abort',
                    'distance': abort_result['min_distance'],
                    'reason': abort_result.get('reason', 'critical')
                })
                await asyncio.sleep(0.1)
                continue

            # STEP 2: Check if escape is taking too long
            if safety_monitor.should_abort_mission():
                logger.error(
                    "[ESCAPE FAILED] Robot stuck after multiple escape attempts. "
                    "Mission will abort."
                )
                results["final_status"] = "stuck_after_escape_timeout"
                break

            # STEP 3: Vision Analysis (Cached at 2Hz)
            # Vision analysis (cached at 2Hz)
            current_time = time.time()
            
            if (cached_vision_analysis is None or 
                current_time - last_vision_update > 0.5):
                
                mission = mission_controller.mission
                vision_analysis = await vision_analyzer.analyze_with_mission(
                    frame, 
                    navigation_goal,
                    mission_type=mission.type,
                    target_class=mission.target_class if hasattr(mission, 'target_class') else None
                )
                
                cached_vision_analysis = vision_analysis
                last_vision_update = current_time
                
                detected_objects = vision_analysis.get('detected_objects', [])
                if detected_objects:
                    PerformanceLogger.log_mission_status(
                        mission.type, len(detected_objects),
                        mission.target_class if hasattr(mission, 'target_class') else None
                    )
            else:
                vision_analysis = cached_vision_analysis
                detected_objects = vision_analysis.get('detected_objects', [])
            
            obstacles = vision_analysis.get("obstacles", [])
            PerformanceLogger.log_vision_analysis(vision_analysis, obstacles)
            results["obstacles_detected"].extend(obstacles)
            
            # STEP 4: Mission State Update 
            # Mission update
            robot_pos = robot_interface.ros_node.get_robot_pose()
            if robot_pos is None and hasattr(robot_interface, 'robot_status'):
                robot_pos = {
                    'x': robot_interface.robot_status.position_x,
                    'y': robot_interface.robot_status.position_y,
                    'theta': getattr(robot_interface.robot_status, 'theta', 0.0)
                }
            
            frame_info = {
                'width': frame.shape[1],
                'height': frame.shape[0]
            } if frame is not None else None

            #Extract full LiDAR scan from vision analyzer if available
            full_lidar_scan = vision_analysis.get('full_lidar_scan', None)

            if slam_controller and slam_controller.is_running:
                slam_controller.maybe_auto_save()

            try:
                mission_result = mission_controller.process_frame(
                    detected_objects=detected_objects,
                    robot_pos=robot_pos,
                    frame_info=frame_info,
                    frame=frame,
                    vision_analyzer=vision_analyzer,
                    full_lidar_scan=full_lidar_scan
                )
                
            except MissionTransitionError as e:
                # Composite mission transition failed
                logger.error("=" * 60)
                logger.error("❌ MISSION TRANSITION FAILED")
                logger.error("=" * 60)
                logger.error(f"Error: {e}")
                logger.error("")
                logger.error("Mission aborted. Check requirements and try again.")
                logger.error("=" * 60)
                
                results["final_status"] = f"transition_error: {str(e)}"
                break
            
            # STEP 5: Mission Completion Check
            # Check mission completion
            if mission_result['completed']:
                logger.info(f"✅ Mission complete: {mission_controller.mission.description}")
                
                results["final_status"] = "mission_completed"
                break

            # STEP 6: Nav2 Goal Planning (Patrol/Follow Only)
            # Navigation decision
            mission_directive = mission_result['directive']

            can_use_nav2 = (
                nav2_ready and 
                robot_pos is not None and
                robot_interface.nav2_interface and
                not robot_interface.nav2_interface.is_navigating() and
                mission_controller.get_current_mission_type() != 'explore_area'
            )

            if can_use_nav2:
                nav2_goal = _mission_directive_to_nav2_goal(
                    mission_directive,
                    robot_pos,
                    vision_analysis
                )
                
                if nav2_goal:
                    goal_x, goal_y, goal_theta = nav2_goal
                    logger.info(f"[NAV2] Goal: ({goal_x:.2f}, {goal_y:.2f})")
                    
                    nav2_success = await robot_interface.send_nav2_goal(
                        x=goal_x,
                        y=goal_y,
                        theta=goal_theta,
                        blocking=False
                    )
                    
                    if nav2_success:
                        results["navigation_decisions"].append({
                            'action': 'nav2_goal',
                            'parameters': {'x': goal_x, 'y': goal_y, 'theta': goal_theta},
                            'directive': mission_directive
                        })
                        
                        await asyncio.sleep(0.1)
                        continue
                    else:
                        can_use_nav2 = False

            # STEP 7: Manual Navigation Decision (Fallback)
            # Manual control fallback
            if not can_use_nav2 or not nav2_goal:
                navigation_decision = navigation_reasoner.decide_next_action(
                    vision_analysis,
                    robot_pos=robot_pos,
                    spatial_detector=vision_analyzer.spatial_detector,
                    mission_directive=mission_directive  # CHANGED: Removed lidar_override param
                )

                if log_buffer:
                    log_buffer.log_iteration(
                        iteration=iteration,
                        robot_pos=robot_pos,
                        vision_analysis=vision_analysis,
                        navigation_decision=navigation_decision,
                        abort_info=abort_result if abort_result.get('abort') else None
                    )
                
                PerformanceLogger.log_navigation_decision(navigation_decision)
                
                if not safety_validator.validate_movement_command(navigation_decision):
                    logger.warning("[SAFETY] Command rejected")
                    navigation_decision = {
                        "action": "stop",
                        "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1},
                        "reason": "safety_override"
                    }
                
                results["navigation_decisions"].append(navigation_decision)

                # STEP 8: Command Execution
                command_success = await robot_interface.execute_command(navigation_decision)
                PerformanceLogger.log_command_result(command_success)
                
                if command_success:
                    results["commands_sent"].append(navigation_decision)
                
                await asyncio.sleep(0.05)

            results["iterations"] = iteration
            await asyncio.sleep(0.05)
            
        except Exception as e:                
            logger.error(f"❌ Loop error: {e}")
            results["final_status"] = f"error: {e}"
            try:
                stop_cmd = {
                    "action": "stop",
                    "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1}
                }
                await robot_interface.execute_command(stop_cmd)
            except:
                pass
            break
        
    # Only run ACE if log_buffer and ace_learner available
    if log_buffer and ace_learner:
        logger.info("=" * 60)
        logger.info("[ACE] STARTING POST-MISSION LEARNING")
        logger.info("=" * 60)
        
        # Generate mission summary
        mission_summary = log_buffer.get_mission_summary(
            completion_status=results.get("final_status", "unknown")
        )
        
        logger.info(f"[ACE] Mission summary:")
        logger.info(f"  - Iterations: {mission_summary.total_iterations}")
        logger.info(f"  - Aborts: {mission_summary.total_aborts}")
        logger.info(f"  - Stuck episodes: {mission_summary.stuck_episodes}")
        
        # Analyze and learn (async, with error handling)
        try:
            logger.info("[ACE] Calling LLM for analysis...")
            
            learning_result = await ace_learner.learn_from_mission(
                mission_summary=mission_summary,
                auto_apply=False  # Manual review recommended for safety
            )
            
            if learning_result:
                analysis = learning_result['analysis']
                valid_recs = learning_result['valid_recommendations']
                rejected = learning_result['rejected_recommendations']
                
                # Display results
                logger.info("=" * 60)
                logger.info("[ACE] LEARNING RESULTS")
                logger.info("=" * 60)
                
                logger.info(f"\n📋 DIAGNOSIS:")
                logger.info(f"  {analysis['diagnosis']}")
                
                logger.info(f"\n🔍 ROOT CAUSE:")
                logger.info(f"  {analysis['root_cause']}")
                
                logger.info(f"\n⚠️  SEVERITY: {analysis.get('severity', 'unknown').upper()}")
                
                death_pendulum = analysis.get('death_pendulum_detected', False)
                logger.info(f"\n🚨 DEATH PENDULUM DETECTED: {death_pendulum}")
                
                logger.info(f"\n📊 RECOMMENDATIONS: {len(valid_recs)} valid, {len(rejected)} rejected")
                
                # Show valid recommendations
                if valid_recs:
                    logger.info("\n" + "=" * 60)
                    logger.info("💡 RECOMMENDED PARAMETER ADJUSTMENTS:")
                    logger.info("=" * 60)
                    
                    for i, rec in enumerate(valid_recs, 1):
                        logger.info(f"\n{i}. {rec['parameter']}")
                        logger.info(f"   Current value:  {rec['current_value']:.3f}")
                        logger.info(f"   Suggested:      {rec['suggested_value']:.3f}")
                        logger.info(f"   Confidence:     {rec['confidence']:.0%}")
                        logger.info(f"   Reasoning:")
                        # Word wrap reasoning
                        reasoning = rec['reasoning']
                        import textwrap
                        wrapped = textwrap.fill(reasoning, width=60, initial_indent='     ', subsequent_indent='     ')
                        logger.info(wrapped)
                    
                    logger.info("\n" + "=" * 60)
                    logger.info("📝 TO APPLY THESE ADJUSTMENTS:")
                    logger.info("=" * 60)
                    logger.info("Option 1 - Review and apply manually:")
                    logger.info("  python -m multi_function_agent.ace_apply --review")
                    logger.info("  python -m multi_function_agent.ace_apply --apply")
                    logger.info("")
                    logger.info("Option 2 - Auto-apply on next run:")
                    logger.info("  Parameters are saved to learned_parameters.json")
                    logger.info("  Will be loaded automatically next time")
                    logger.info("=" * 60)
                else:
                    logger.info("\n✅ No adjustments needed - parameters look good!")
                
                # Show rejected if any
                if rejected:
                    logger.info("\n⚠️  REJECTED RECOMMENDATIONS:")
                    for i, rej in enumerate(rejected, 1):
                        rec = rej['recommendation']
                        reason = rej['rejection_reason']
                        logger.info(f"  {i}. {rec['parameter']}: {reason}")
                
                # Additional notes
                if analysis.get('additional_notes'):
                    logger.info(f"\n💭 ADDITIONAL NOTES:")
                    logger.info(f"  {analysis['additional_notes']}")
                
                logger.info("\n" + "=" * 60)
                logger.info("[ACE] POST-MISSION LEARNING COMPLETE")
                logger.info("=" * 60)
                
            else:
                logger.warning("[ACE] ⚠️  Learning failed - no analysis generated")
        
        except asyncio.TimeoutError:
            logger.error("[ACE] ❌ LLM timeout - analysis took too long")
        
        except Exception as e:
            logger.error(f"[ACE] ❌ Learning error: {e}")
            import traceback
            logger.debug(traceback.format_exc())

    await stream_handler.stop_stream()
    
    if results["final_status"] == "unknown":
        results["final_status"] = "completed"
    
    return results