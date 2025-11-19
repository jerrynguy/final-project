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

from multi_function_agent._robot_vision_controller.navigation.navigation_reasoner import NavigationReasoner
from multi_function_agent._robot_vision_controller.navigation.robot_controller_interface import RobotControllerInterface
try:
    from multi_function_agent._robot_vision_controller.navigation.nav2_interface import NavigationState
    NAV2_STATE_AVAILABLE = True
except ImportError:
    NavigationState = None
    NAV2_STATE_AVAILABLE = False
# ==================================
from multi_function_agent._robot_vision_controller.navigation.robot_controller_interface import RobotControllerInterface

from multi_function_agent._robot_vision_controller.core.query_extractor import QueryExtractor
from multi_function_agent._robot_vision_controller.core.ros2_node import get_ros2_node
from multi_function_agent._robot_vision_controller.core.mission_controller import (
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

# Initialize ROS2 node early (singleton pattern)
logger.info("Initializing ROS2 node...")
ros2_node = get_ros2_node()
logger.info("✅ ROS2 node ready")

logger.info("Starting robot vision model preloading...")
preload_success = preload_robot_vision_model()

if preload_success:
    logger.info("Robot vision model preloaded successfully!")
else:
    logger.error("Failed to preload robot vision model")

# =============================================================================
# Main Controller Function
# =============================================================================

async def _robot_vision_controller(
    config: RobotVisionConfig, 
    builder: Builder
) -> AsyncGenerator[FunctionInfo, None]:
    """Main robot vision controller function."""
    
    # Resolve configuration
    robot_config = query_extractor.debugging_robot_config_resolution(config, builder, logger)
    stream_url = robot_config["stream_url"]
    control_mode = robot_config["control_mode"]
    navigation_goal = robot_config["navigation_goal"]
    safety_level = robot_config["safety_level"]
    
    # Validate stream URL
    if not stream_url or not stream_url.strip():
        yield FunctionInfo.from_fn(
            ErrorHandlers.no_stream_url(),
            description="No stream URL provided"
        )
        return
    
    # Validate stream connection
    is_stream_valid, stream_error = await stream_handler.validate_stream(stream_url)
    if not is_stream_valid:
        yield FunctionInfo.from_fn(
            ErrorHandlers.invalid_stream(stream_url, stream_error),
            description="Invalid stream connection"
        )
        return
    
    try:
        logger.info(f"Starting robot vision control: {stream_url}")
        logger.info(f"Control mode: {control_mode}, Goal: {navigation_goal}, Safety: {safety_level}")
        
        # CHANGED: Removed BLIP2 model loading
        logger.info("Using YOLO-only vision pipeline (BLIP2 removed)")
        
        # ROS2 already initialized globally via get_ros2_node()
        logger.info("Using global ROS2 node instance")
        
        # Initialize core components
        navigation_reasoner = NavigationReasoner(
            safety_level=safety_level, 
            max_speed=robot_config["max_speed"]
        )

        # Apply exploration speed boost if in config
        exploration_boost = robot_config.get("exploration_speed_boost", 1.0)
        navigation_reasoner.set_exploration_boost(exploration_boost)

        robot_interface = RobotControllerInterface()
        vision_analyzer = RobotVisionAnalyzer(robot_controller=robot_interface)
        safety_monitor = LidarSafetyMonitor()

        nav2_ready = False
        if robot_interface.use_nav2:
            logger.info("Initializing Nav2...")
            nav2_ready = await robot_interface.wait_for_nav2_ready(timeout=10.0)
            
            if nav2_ready:
                logger.info("Nav2 ready for hybrid navigation")
            else:
                logger.warning("Nav2 not ready - using manual control fallback")
        else:
            logger.info("Nav2 disabled - using manual control only")
        
        # Connect to robot controller
        connection_success = await robot_interface.connect()
        if not connection_success:
            logger.warning("Could not connect to robot controller")

        # STARTUP ESCAPE: Back away if spawned too close to obstacle
        logger.info("[STARTUP] Checking initial position safety...")
        initial_scan = robot_interface.lidar_data

        if initial_scan:
            startup_monitor = LidarSafetyMonitor()
            min_dist = startup_monitor.get_min_distance(initial_scan)
            if min_dist < 0.5:  # Too close to obstacle
                logger.warning(f"[STARTUP] Too close to obstacle ({min_dist:.2f}m) - backing away")
                back_away_cmd = {
                    "action": "move_backward",
                    "parameters": {
                        "linear_velocity": -0.2,
                        "angular_velocity": 0.0,
                        "duration": 2.0
                    },
                    "reason": "startup_back_away"
                }
                escape_success = await robot_interface.execute_command(back_away_cmd)
                if escape_success:
                    logger.info("[STARTUP ESCAPE] ✅ Successfully moved away from obstacle")
                    await asyncio.sleep(0.5)  # Settle time
                else:
                    logger.warning("[STARTUP ESCAPE] ⚠️  Escape failed, proceeding with caution")
            else:
                logger.info(f"[STARTUP] ✅ Safe initial position (clearance: {min_dist:.2f}m)")
        else:
            logger.warning("[STARTUP] No LiDAR data, skipping safety check")
        
        # Parse mission using MissionController.from_prompt()
        user_prompt = builder._workflow_builder.general_config.front_end.input_query[0]
        try:
            mission_controller = await MissionController.from_prompt(user_prompt, builder)
        
        except UnsupportedMissionError as e:
            error_msg = (
                f"❌ Mission Not Supported\n\n"
                f"{str(e)}\n\n"
                f"Your prompt: '{user_prompt}'\n\n"
                f"Supported missions:\n"
                f"  • follow_target: 'Follow the person', 'Track the dog'\n"
                f"  • patrol_laps: 'Patrol 5 laps', 'Go around 10 times'\n"
                f"  • explore_area: 'Explore freely' (SLAM required - not yet available)\n"
            )
            logger.error(error_msg)
            
            yield FunctionInfo.from_fn(
                lambda x: error_msg,
                description="Mission type not supported"
            )
            return
        
        except MissionRequirementsError as e:
            error_msg = (
                f"❌ Mission Requirements Not Met\n\n"
                f"{str(e)}\n\n"
                f"Your mission: '{mission_controller.mission.type if 'mission_controller' in locals() else 'unknown'}'\n"
            )
            logger.error(error_msg)
            
            yield FunctionInfo.from_fn(
                lambda x: error_msg,
                description="Mission requirements not met"
            )
            return
        
        except MissionParsingError as e:
            error_msg = (
                f"❌ Mission Parsing Failed\n\n"
                f"{str(e)}\n\n"
                f"Your prompt: '{user_prompt}'\n\n"
                f"Please rephrase your command. Examples:\n"
                f"  • 'Follow the person in front'\n"
                f"  • 'Patrol 5 circular laps'\n"
                f"  • 'Track the dog'\n"
            )
            logger.error(error_msg)
            
            yield FunctionInfo.from_fn(
                lambda x: error_msg,
                description="Could not parse mission"
            )
            return
        
        # Initialize SLAM if explore mission 
        slam_controller = None
        if mission_controller.mission.type == 'explore_area':
            logger.info("[SLAM] Initializing for exploration mission...")
            slam_controller = SLAMController(map_save_path="~/my_map")
            
            slam_started = slam_controller.start_slam()
            if not slam_started:
                error_msg = (
                    "❌ SLAM Failed to Start\n\n"
                    "Could not start SLAM Toolbox. Please check:\n"
                    "1. ROS2 environment sourced\n"
                    "2. slam_toolbox installed: sudo apt install ros-humble-slam-toolbox\n"
                    "3. Gazebo simulation running\n"
                )
                logger.error(error_msg)
                
                yield FunctionInfo.from_fn(
                    lambda x: error_msg,
                    description="SLAM initialization failed"
                )
                return
            
            logger.info("[SLAM] Mapping started successfully")
        
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
        )

        # Stop SLAM if it was running 
        if slam_controller and slam_controller.is_running:
            logger.info("[SLAM] Stopping and saving final map...")
            slam_controller.stop_slam(save_final_map=True)
            
            # Verify map quality
            is_valid, reason = slam_controller.verify_map_quality()
            if is_valid:
                logger.info(f"[SLAM] ✅ Map created successfully: ~/my_map.yaml")
                control_results['slam_map_created'] = True
                control_results['slam_map_path'] = slam_controller.map_save_path
            else:
                logger.warning(f"[SLAM] ⚠️ Map quality issue: {reason}")
                control_results['slam_map_created'] = False
                control_results['slam_error'] = reason
        
        # Format output
        formatted_output = OutputFormatter.format_control_results(
            control_results, 
            stream_url, 
            control_mode, 
            mission_controller.mission
        )

        # Add SLAM info to output if applicable
        if slam_controller:
            slam_stats = slam_controller.get_mapping_stats()
            formatted_output += f"\n\n=== SLAM Mapping Results ===\n"
            formatted_output += f"Duration: {slam_stats['duration']:.1f}s\n"
            formatted_output += f"Map saved: {slam_stats['map_exists']}\n"
            if slam_stats['map_exists']:
                formatted_output += f"Map path: {slam_stats['map_path']}.yaml\n"
                formatted_output += f"Map size: {slam_stats['yaml_size']}B (YAML), {slam_stats['pgm_size']}B (PGM)\n"   
        
        # Update performance metrics
        control_time = time.time() - control_start_time
        model_manager.update_inference_time(control_time)
        model_manager.cleanup_gpu_memory()
        
        # Display results
        print("\n===== ROBOT VISION CONTROL OUTPUT =====\n")
        print(formatted_output)
        
        stream_stats = stream_handler.get_stream_stats()
        print(f"\n{OutputFormatter.format_stream_stats(stream_stats)}")
        
        # Return success result
        async def success_control(dummy: str) -> str:
            return formatted_output
        
        try:
            yield FunctionInfo.from_fn(
                success_control,
                description=f"Robot vision control completed: {control_mode} mode",
            )
        finally:
            logger.info("Mission completed, cleaning up...")
                    
    except Exception as e:
        # Emergency cleanup
        if 'slam_controller' in locals() and slam_controller and slam_controller.is_running:
            logger.warning("[SLAM] Emergency shutdown due to error")
            slam_controller.stop_slam(save_final_map=True)

        logger.error(f"Error in robot vision control: {e}")
        
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
    """
    Convert mission directive to Nav2 goal (x, y, theta).
    Uses proper coordinate transformation from robot frame to map frame.
    """
    if robot_pos is None:
        logger.warning("No robot position for Nav2 goal")
        return None
    
    current_x = robot_pos['x']
    current_y = robot_pos['y']
    current_theta = robot_pos.get('theta', 0.0)
    
    import numpy as np
    
    def relative_to_absolute(distance: float, angle_offset: float = 0.0):
        """Transform relative (distance, angle_offset) to absolute (x, y, theta)."""
        target_theta = current_theta + angle_offset
        goal_x = current_x + distance * np.cos(target_theta)
        goal_y = current_y + distance * np.sin(target_theta)
        return (goal_x, goal_y, target_theta)
    
    # Tracking directives
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
    
    # Patrol directives
    elif directive.startswith('patrol_'):
        return relative_to_absolute(1.0, 0.3)
    
    # Exploration
    elif directive.startswith('explore_'):
        import random
        angle_offset = random.uniform(-np.pi, np.pi)
        distance = random.uniform(2.0, 3.0)
        return relative_to_absolute(distance, angle_offset)
    
    # Manual-only directives
    return None

# =============================================================================
# Nav2 Continuous Safety Monitor
# =============================================================================

async def _nav2_continuous_safety_monitor(
    robot_interface,
    safety_monitor: LidarSafetyMonitor,
    results: Dict
) -> None:
    """
    Background task for continuous Nav2 safety monitoring.
    Runs at 20Hz while Nav2 is navigating.
    """
    logger.info("[NAV2 MONITOR] Starting continuous safety monitor")
    
    try:
        while robot_interface.nav2_interface.is_navigating():
            # Get fresh LIDAR data
            lidar_data = robot_interface.lidar_data
            
            if lidar_data is None:
                logger.warning("[NAV2 MONITOR] No LIDAR data, forcing stop")
                robot_interface.cancel_nav2_navigation()
                await robot_interface.execute_command({
                    'action': 'stop',
                    'parameters': {'linear_velocity': 0.0, 'angular_velocity': 0.0, 'duration': 0.1},
                    'reason': 'nav2_monitor_no_lidar'
                })
                break
            
            # Check safety
            min_dist = safety_monitor.get_min_distance(lidar_data)
            
            if min_dist < safety_monitor.CRITICAL_DISTANCE:
                logger.error(
                    f"[NAV2 MONITOR] CRITICAL DISTANCE {min_dist:.2f}m! "
                    f"Aborting Nav2 navigation"
                )
                
                # Cancel Nav2 goal
                robot_interface.cancel_nav2_navigation()
                
                # Execute emergency stop
                stop_cmd = {
                    'action': 'stop',
                    'parameters': {
                        'linear_velocity': 0.0,
                        'angular_velocity': 0.0,
                        'duration': 0.1
                    },
                    'reason': f'nav2_monitor_abort_{min_dist:.2f}m'
                }
                await robot_interface.execute_command(stop_cmd)
                
                results["navigation_decisions"].append({
                    'action': 'nav2_abort',
                    'reason': f'critical_distance_{min_dist:.2f}m',
                    'distance': min_dist
                })
                
                break
            
            elif min_dist < safety_monitor.WARNING_DISTANCE:
                logger.debug(
                    f"[NAV2 MONITOR] Warning distance {min_dist:.2f}m "
                    f"(Nav2 costmap should handle this)"
                )
            
            # Monitor at 20Hz
            await asyncio.sleep(0.05)
    
    except asyncio.CancelledError:
        logger.info("[NAV2 MONITOR] Monitor cancelled")
    
    except Exception as e:
        logger.error(f"[NAV2 MONITOR] Monitor error: {e}")
        # Emergency: cancel Nav2 on error
        try:
            robot_interface.cancel_nav2_navigation()
        except:
            pass
    
    finally:
        logger.info("[NAV2 MONITOR] Continuous safety monitor stopped")

# =============================================================================
# Control Loop - REFACTORED
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
    
    # Start video stream
    stream_started = await stream_handler.start_stream(stream_url)
    if not stream_started:
        raise ValueError("Could not start continuous stream")
    
    logger.info("Real-time streaming started")
    logger.info(f"[MISSION] Started: {mission_controller.mission.description}")
    
    iteration = 0
    last_vision_update = 0
    cached_vision_analysis = None
    last_slam_save = 0

    while max_iterations is None or iteration < max_iterations:
        try:
            lidar_snapshot = robot_interface.lidar_data
            # Get latest camera frame
            frame = await stream_handler.get_latest_frame()
            if frame is None:
                await asyncio.sleep(0.5)
                continue
            
            iteration += 1
            PerformanceLogger.log_iteration_start(iteration)

            # PRIORITY 0: ABORT NAV2 IF CRITICAL (NON-BLOCKING CHECK)
            if nav2_ready and robot_interface.nav2_interface:
                if robot_interface.nav2_interface.is_navigating():
                    safe = robot_interface.check_nav2_safety(lidar_override=lidar_snapshot)
                    if not safe:
                        logger.warning("[NAV2 ABORTED] Safety override")
                        results["navigation_decisions"].append({
                            'action': 'abort_nav2',
                            'reason': 'safety_critical'
                        })
                        continue
            
            # PRIORITY 1: SAFETY CHECK (BLOCKING)
            safety_result = await safety_monitor.handle_safety_override(
                lidar_snapshot, robot_interface, results
            )
            
            if safety_result['veto']:
                # Execute escape NON-BLOCKING
                asyncio.create_task(
                    robot_interface.execute_command(safety_result['command'])
                )
                results["commands_sent"].append(safety_result['command'])
                
                # Continue immediately
                await asyncio.sleep(0.05)
                continue

            # PRIORITY 2: VISION ANALYSIS 
            current_time = time.time()
            
            # Only run YOLO at 2Hz (every 0.5s)
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
                # Use cached analysis
                vision_analysis = cached_vision_analysis
                detected_objects = vision_analysis.get('detected_objects', [])
            
            obstacles = vision_analysis.get("obstacles", [])
            PerformanceLogger.log_vision_analysis(vision_analysis, obstacles)
            results["obstacles_detected"].extend(obstacles)
            
            # PRIORITY 3: MISSION STATE UPDATE 
            # Get robot position from ROS2 node directly
            robot_pos = robot_interface.ros_node.get_robot_pose()

            # Fallback to robot_status if needed
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
            
            # Use process_frame() for unified update
            mission_result = mission_controller.process_frame(
                detected_objects=detected_objects,
                robot_pos=robot_pos,
                frame_info=frame_info
            )

            if slam_controller and slam_controller.is_running:
                current_time = time.time()
                if current_time - last_slam_save >= 5.0:  # Save every 5s
                    saved = slam_controller.auto_save_map()
                    if saved:
                        last_slam_save = current_time
                        logger.info("[SLAM] Map auto-saved")
            
            # Check for mission completion
            if mission_result['completed']:
                logger.info(f"[MISSION COMPLETE] {mission_controller.mission.description}")
                
                # Save final SLAM map if explore mission
                if slam_controller and slam_controller.is_running:
                    logger.info("[SLAM] Saving final map...")
                    slam_controller.save_map()
                
                results["final_status"] = "mission_completed"
                break

            # PRIORITY 4: NAVIGATION DECISION - HYBRID NAV2/MANUAL
            mission_directive = mission_result['directive']
            logger.info(f"[MISSION] Directive: {mission_directive}")

            # Determine if we can use Nav2 for this directive
            can_use_nav2 = (
                nav2_ready and 
                robot_pos is not None and
                robot_interface.nav2_interface and
                not robot_interface.nav2_interface.is_navigating()
            )

            if can_use_nav2:
                # Try to convert directive to Nav2 goal
                nav2_goal = _mission_directive_to_nav2_goal(
                    mission_directive,
                    robot_pos,
                    vision_analysis
                )
                
                if nav2_goal:
                    # Use Nav2 for navigation
                    goal_x, goal_y, goal_theta = nav2_goal
                    logger.info(f"[NAV2] Sending goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.2f})")
                    
                    # ============================================================
                    # PRIORITY 5a: EXECUTE VIA NAV2 (NON-BLOCKING)
                    # ============================================================
                    nav2_success = await robot_interface.send_nav2_goal(
                        x=goal_x,
                        y=goal_y,
                        theta=goal_theta,
                        blocking=False  # Non-blocking - let Nav2 handle path
                    )
                    
                    if nav2_success:
                        # Spawn continuous safety monitor
                        monitor_task = asyncio.create_task(
                             _nav2_continuous_safety_monitor(
                                 robot_interface,
                                 safety_monitor,
                                 results
                             )
                         )
                        logger.info("[NAV2] Continuous safety monitor started")

                        results["navigation_decisions"].append({
                            'action': 'nav2_goal',
                            'parameters': {'x': goal_x, 'y': goal_y, 'theta': goal_theta},
                            'directive': mission_directive,
                            'confidence': 0.95
                        })
                        
                        # Monitor Nav2 state with safety checks
                        nav2_state = robot_interface.get_nav2_state()
                        logger.info(f"[NAV2] State: {nav2_state}")
                        
                        # Continue loop - Nav2 handles execution
                        await asyncio.sleep(0.1)  # Longer interval for Nav2
                        continue
                    else:
                        logger.warning("[NAV2] Goal rejected - falling back to manual")
                        can_use_nav2 = False

            # Fallback to manual control if Nav2 not used
            if not can_use_nav2 or not nav2_goal:
                logger.info("[MANUAL] Using manual navigation")
                
                # Original manual navigation logic
                navigation_decision = navigation_reasoner.decide_next_action(
                    vision_analysis,
                    robot_pos=robot_pos,
                    spatial_detector=vision_analyzer.spatial_detector,
                    lidar_override=None,
                    mission_directive=mission_directive
                )
                
                PerformanceLogger.log_navigation_decision(navigation_decision)
                
                # Validate safety
                if not safety_validator.validate_movement_command(navigation_decision):
                    logger.warning("  SAFETY OVERRIDE: Command rejected")
                    navigation_decision = {
                        "action": "stop",
                        "parameters": {
                            "linear_velocity": 0.0,
                            "angular_velocity": 0.0,
                            "duration": 0.1
                        },
                        "reason": "safety_override",
                    }
                
                results["navigation_decisions"].append(navigation_decision)
                
                # ============================================================
                # PRIORITY 5b: EXECUTE MANUAL COMMAND (BLOCKING with ABORT)
                # ============================================================
                
                # Final safety check with FRESH lidar data
                if lidar_snapshot is not None:
                    min_dist = safety_monitor.get_min_distance(lidar_snapshot)
                    
                    if min_dist < safety_monitor.CRITICAL_DISTANCE:
                        PerformanceLogger.log_safety_abort(min_dist, navigation_decision['action'])
                        
                        navigation_decision = {
                            "action": "stop",
                            "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1},
                            "reason": f"pre_execution_abort_{min_dist:.2f}m",
                        }
                        
                        await robot_interface.execute_command(navigation_decision)
                        continue
                    
                    elif min_dist < safety_monitor.WARNING_DISTANCE:
                        PerformanceLogger.log_safety_warning(min_dist)
                        
                        # Scale down velocity
                        params = navigation_decision.get('parameters', {})
                        scale = (min_dist - safety_monitor.CRITICAL_DISTANCE) / \
                                (safety_monitor.WARNING_DISTANCE - safety_monitor.CRITICAL_DISTANCE)
                        scale = max(0.3, min(1.0, scale))
                        
                        params['linear_velocity'] = params.get('linear_velocity', 0.0) * scale
                        params['angular_velocity'] = params.get('angular_velocity', 0.0) * scale
                        navigation_decision['parameters'] = params
                else:
                    logger.warning("[PRE-EXECUTION] No LIDAR data, forcing STOP")
                    navigation_decision = {
                        "action": "stop",
                        "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1},
                        "reason": "no_lidar_data",
                    }

                command_success = await robot_interface.execute_command(navigation_decision)
                
                PerformanceLogger.log_command_result(command_success)
                
                if command_success:
                    results["commands_sent"].append(navigation_decision)
                
                # Small delay between iterations
                await asyncio.sleep(0.05)

            results["iterations"] = iteration
            
            # Small delay between iterations
            await asyncio.sleep(0.05)
            
        except asyncio.CancelledError:
            # Save SLAM map on interruption
            if slam_controller and slam_controller.is_running:
                logger.info("[SLAM] Saving map before interruption...")
                slam_controller.save_map()
            
            results["final_status"] = "user_interrupted"
            logger.info("Control loop interrupted")
            break
            
        except Exception as e:
            # Emergency save on error
            if slam_controller and slam_controller.is_running:
                logger.warning("[SLAM] Emergency save due to error")
                slam_controller.save_map()
                
            logger.error(f"Control loop error: {e}")
            results["final_status"] = f"error: {e}"
            # Emergency stop on error
            try:
                stop_cmd = {
                    "action": "stop",
                    "parameters": {"linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.1}
                }
                await robot_interface.execute_command(stop_cmd)
            except:
                pass
            break
    
    # Cleanup
    await stream_handler.stop_stream()
    logger.info(f"Control loop stopped after {results['iterations']} iterations")
    
    if results["final_status"] == "unknown":
        results["final_status"] = "completed"
    
    return results