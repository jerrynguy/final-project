import numpy as np
from pydantic import Field
import time
import logging
import asyncio
from typing import AsyncGenerator, Dict, Any, Optional, Tuple

from multi_function_agent._robot_vision_controller.utils.log.error_handlers import ErrorHandlers
from multi_function_agent._robot_vision_controller.utils.log.output_formatter import OutputFormatter
from multi_function_agent._robot_vision_controller.utils.log.performance_logger import PerformanceLogger
from multi_function_agent._robot_vision_controller.utils.safety_checks import SafetyValidator

from multi_function_agent._robot_vision_controller.perception.slam_controller import SLAMController
from multi_function_agent._robot_vision_controller.perception.lidar_monitor import LidarSafetyMonitor
from multi_function_agent._robot_vision_controller.perception.rtsp_stream_handler import RTSPStreamHandler
from multi_function_agent._robot_vision_controller.perception.robot_vision_analyzer import RobotVisionAnalyzer

from multi_function_agent._robot_vision_controller.navigation.stuck_detector import StuckDetector
from multi_function_agent._robot_vision_controller.navigation.ai_recovery_agent import AIRecoveryAgent
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
from multi_function_agent._robot_vision_controller.core.goal_parser import (
    MissionParsingError,
    UnsupportedMissionError
)
from multi_function_agent._robot_vision_controller.core.models import (
    get_robot_vision_model_manager,
    preload_robot_vision_model,
    is_yolo_ready,
)
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

# =============================================================================
# Global Initialization
# =============================================================================

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

# =============================================================================
# Main Controller Function
# =============================================================================

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
        stuck_detector = StuckDetector(
            window_size=3,
            displacement_threshold=0.05
        )
        ai_recovery_agent = AIRecoveryAgent()

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
            if min_dist < 0.5:
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
            mission_controller = await MissionController.from_prompt(user_prompt, builder)
        
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
        
        # Initialize SLAM for explore mission
        slam_controller = None
        if mission_controller.mission.type == 'explore_area':
            logger.info("[SLAM] Starting mapping...")
            slam_controller = SLAMController(map_save_path="~/my_map")
            
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
            stuck_detector,
            ai_recovery_agent, 
            slam_controller=slam_controller,
            nav2_ready=nav2_ready,
            max_iterations=None,
        )

        # Stop SLAM and save map
        if slam_controller and slam_controller.is_running:
            logger.info("[SLAM] Saving final map...")
            slam_controller.stop_slam(save_final_map=True)
            
            is_valid, reason = slam_controller.verify_map_quality()
            if is_valid:
                logger.info(f"✅ Map created: ~/my_map.yaml")
                control_results['slam_map_created'] = True
            else:
                logger.warning(f"⚠️ Map quality issue: {reason}")
                control_results['slam_map_created'] = False
        
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
        if 'slam_controller' in locals() and slam_controller and slam_controller.is_running:
            slam_controller.stop_slam(save_final_map=True)

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

# =============================================================================
# Nav2 Continuous Safety Monitor
# =============================================================================

async def _nav2_continuous_safety_monitor(
    robot_interface,
    safety_monitor: LidarSafetyMonitor,
    results: Dict
) -> None:
    """Background task for continuous Nav2 safety monitoring at 20Hz."""
    
    try:
        while robot_interface.nav2_interface.is_navigating():
            lidar_data = robot_interface.lidar_data
            
            if lidar_data is None:
                logger.warning("[NAV2] No LIDAR - forcing stop")
                robot_interface.cancel_nav2_navigation()
                await robot_interface.execute_command({
                    'action': 'stop',
                    'parameters': {'linear_velocity': 0.0, 'angular_velocity': 0.0, 'duration': 0.1},
                    'reason': 'nav2_monitor_no_lidar'
                })
                break
            
            min_dist = safety_monitor.get_min_distance(lidar_data)
            
            if min_dist < safety_monitor.CRITICAL_DISTANCE:
                logger.error(f"[NAV2 ABORT] Obstacle at {min_dist:.2f}m")
                robot_interface.cancel_nav2_navigation()
                
                stop_cmd = {
                    'action': 'stop',
                    'parameters': {'linear_velocity': 0.0, 'angular_velocity': 0.0, 'duration': 0.1},
                    'reason': f'nav2_abort_{min_dist:.2f}m'
                }
                await robot_interface.execute_command(stop_cmd)
                
                results["navigation_decisions"].append({
                    'action': 'nav2_abort',
                    'reason': f'critical_{min_dist:.2f}m'
                })
                break
            
            await asyncio.sleep(0.05)
    
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error(f"[NAV2 MONITOR] Error: {e}")
        try:
            robot_interface.cancel_nav2_navigation()
        except:
            pass

# =============================================================================
# Control Loop
# =============================================================================
async def run_robot_control_loop(
    stream_url: str,
    vision_analyzer: RobotVisionAnalyzer,
    navigation_reasoner: NavigationReasoner,
    robot_interface: RobotControllerInterface,
    navigation_goal: str,
    safety_validator: SafetyValidator,
    safety_monitor: LidarSafetyMonitor,
    mission_controller: MissionController, 
    stuck_detector: StuckDetector,
    ai_recovery_agent: AIRecoveryAgent, 
    slam_controller: Optional[SLAMController] = None,
    nav2_ready: bool = False,
    max_iterations: int = None,
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
    last_slam_save = 0

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

            # STEP 1: Nav2 Active Navigation Safety
            # Check Nav2 safety
            if nav2_ready and robot_interface.nav2_interface:
                if robot_interface.nav2_interface.is_navigating():
                    safe = robot_interface.check_nav2_safety(lidar_override=lidar_snapshot)
                    if not safe:
                        logger.warning("[NAV2] Safety abort")
                        results["navigation_decisions"].append({
                            'action': 'abort_nav2',
                            'reason': 'safety_critical'
                        })
                        continue
            
            # STEP 2: Emergency LIDAR Safety Veto
            # Safety check
            safety_result = await safety_monitor.handle_safety_override(
                lidar_snapshot, robot_interface, results
            )
            
            if safety_result['veto']:
                asyncio.create_task(
                    robot_interface.execute_command(safety_result['command'])
                )
                results["commands_sent"].append(safety_result['command'])
                await asyncio.sleep(0.05)
                continue

            # STEP 3: Stuck Detection & AI Recovery
            # Stuck detection and ai recovery
            if mission_controller.mission.type == 'explore_area':
                robot_pos = robot_interface.ros_node.get_robot_pose()
                if robot_pos:
                    #Get last action
                    last_action = (
                        results["navigation_decisions"][-1].get('action','unknown')
                        if results["navigation_decisions"] else 'unknown'
                    )

                    # Check if stuck
                    is_stuck = stuck_detector.update(robot_pos, last_action)

                    if is_stuck:
                        logger.warning("[STUCK] Robot appears to be stuck, invoking AI recovery")

                        # Get recent action history
                        recent_actions = [
                            d.get('action','unknown') for d in results["navigation_decisions"][-5:]
                        ]

                        # Generate AI escape command
                        try:
                            escape_command = await ai_recovery_agent.generate_escape_command(
                                vision_context = vision_analysis,
                                last_actions= recent_actions,
                                stuck_reason="position_not_changing"
                            )

                            logger.info(f"[AI RECOVERY] Executing escape command: {escape_command}")

                            # Convert to navigation decision format
                            navigation_decision = {
                                'action': escape_command['action'],
                                'parameters': {
                                    'linear_velocity': escape_command['linear'],
                                    'angular_velocity': escape_command['angular'],
                                    'duration': escape_command['duration']
                                },
                                'confidence': 0.95,
                                'reason': f"ai_recovery_{escape_command['reason']}"
                            }

                            command_success = await robot_interface.execute_command(navigation_decision)

                            if command_success:
                                results["navigation_decisions"].append(navigation_decision)
                                results["commands_sent"].append(navigation_decision)

                                # Reset stuck detector after successful recovery
                                stuck_detector.reset()
                                logger.info("✅ [AI RECOVERY] Escape executed, resuming exploration")

                            await asyncio.sleep(0.1)
                            continue
                        except Exception as e:
                            logger.error(f"[AI RECOVERY] Failed to generate/execute escape command: {e}")
                            # Proceed with normal navigation if recovery fails

            # STEP 4: Vision Analysis (Cached at 2Hz)
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
            
            # STEP 5: Mission State Update & SLAM Auto-Save
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
            
            mission_result = mission_controller.process_frame(
                detected_objects=detected_objects,
                robot_pos=robot_pos,
                frame_info=frame_info
            )

            # SLAM auto-save
            if slam_controller and slam_controller.is_running:
                current_time = time.time()
                if current_time - last_slam_save >= 10.0:
                    saved = slam_controller.auto_save_map()
                    if saved:
                        last_slam_save = current_time
            
            # STEP 6: Mission Completion Check
            # Check mission completion
            if mission_result['completed']:
                logger.info(f"✅ Mission complete: {mission_controller.mission.description}")
                
                if slam_controller and slam_controller.is_running:
                    slam_controller.save_map()

                # Log AI recovery stats
                if ai_recovery_agent.total_invocations > 0:
                    stats = ai_recovery_agent.get_stats()
                    logger.info(
                        f"[AI RECOVERY] Invoked {ai_recovery_agent.total_invocations} times, "
                        f"Success rate: {stats['parse_success_rate']:.1%}, "
                        f"Avg latency: {stats['avg_inference_time_ms']:.0f}ms"
                    )
                
                stuck_stats = stuck_detector.get_stats()
                logger.info(
                    f"Total checks: {stuck_stats['total_checks']}, "
                    f"Stuck events: {stuck_stats['stuck_events']}, "
                    f"Stuck rate: {stuck_stats['stuck_rate']:.1%}"
                )
                
                results["final_status"] = "mission_completed"
                break

            # STEP 7: Nav2 Goal Planning (Patrol/Follow Only)
            # Navigation decision
            mission_directive = mission_result['directive']

            can_use_nav2 = (
                nav2_ready and 
                robot_pos is not None and
                robot_interface.nav2_interface and
                not robot_interface.nav2_interface.is_navigating() and
                mission_controller.mission.type != 'explore_area'
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
                        monitor_task = asyncio.create_task(
                             _nav2_continuous_safety_monitor(
                                 robot_interface,
                                 safety_monitor,
                                 results
                             )
                         )

                        results["navigation_decisions"].append({
                            'action': 'nav2_goal',
                            'parameters': {'x': goal_x, 'y': goal_y, 'theta': goal_theta},
                            'directive': mission_directive
                        })
                        
                        await asyncio.sleep(0.1)
                        continue
                    else:
                        can_use_nav2 = False

            # STEP 8: Manual Navigation Decision (Fallback)
            # Manual control fallback
            if not can_use_nav2 or not nav2_goal:
                navigation_decision = navigation_reasoner.decide_next_action(
                    vision_analysis,
                    robot_pos=robot_pos,
                    spatial_detector=vision_analyzer.spatial_detector,
                    lidar_override=None,
                    mission_directive=mission_directive
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
                
                # STEP 9: Pre-Execution Safety Check
                # Pre-execution safety check
                critical_threshold = getattr(safety_monitor, 'CRITICAL_DISTANCE', 0.4)
                if lidar_snapshot is not None:
                    min_dist = safety_monitor.get_min_distance(lidar_snapshot)
                    
                    if min_dist < critical_threshold:
                        PerformanceLogger.log_safety_abort(min_dist, navigation_decision['action'])
                        
                        navigation_decision = {
                            "action": "stop",
                            "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.05},
                            "reason": f"pre_abort_{min_dist:.2f}m"
                        }
                        
                        await robot_interface.execute_command(navigation_decision)
                        continue
                else:
                    navigation_decision = {
                        "action": "stop",
                        "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1},
                        "reason": "no_lidar"
                    }

                # STEP 10: Command Execution
                command_success = await robot_interface.execute_command(navigation_decision)
                PerformanceLogger.log_command_result(command_success)
                
                if command_success:
                    results["commands_sent"].append(navigation_decision)
                
                await asyncio.sleep(0.05)

            results["iterations"] = iteration
            await asyncio.sleep(0.05)
            
        except asyncio.CancelledError:
            if slam_controller and slam_controller.is_running:
                slam_controller.save_map()
            results["final_status"] = "interrupted"
            break
            
        except Exception as e:
            if slam_controller and slam_controller.is_running:
                slam_controller.save_map()
                
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
    
    await stream_handler.stop_stream()
    
    if results["final_status"] == "unknown":
        results["final_status"] = "completed"
    
    return results