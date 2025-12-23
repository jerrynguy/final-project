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

def _calculate_nav2_escape_goal(
    robot_pos: Dict,
    clearances: Dict,
    slam_controller: Optional[SLAMController],
    stuck_type: str
) -> Optional[Dict]:
    """
    Calculate Nav2 escape goal using CLEARANCE-BASED direction.
    
    Strategy:
    1. Find direction with MAX clearance (front/left/right/rear)
    2. Set goal 2.5m away in that direction
    3. Validate with SLAM map if available
    """
    if robot_pos is None:
        logger.error("[NAV2 GOAL] No robot position")
        return None
    
    current_x = robot_pos['x']
    current_y = robot_pos['y']
    current_theta = robot_pos.get('theta', 0.0)
    
    import numpy as np
    
    # ✅ FIX: Use clearance-based direction instead of current heading
    # Build direction map with (clearance, angle_offset)
    directions = {
        'front': (clearances.get('forward', 0), 0.0),          # 0° (straight)
        'left': (clearances.get('left', 0), np.pi/2),         # 90° (left)
        'right': (clearances.get('right', 0), -np.pi/2),      # -90° (right)
        'rear': (clearances.get('rear', 0), np.pi)            # 180° (back)
    }
    
    # Choose direction with MAX clearance
    best_dir_name, (best_clearance, angle_offset) = max(
        directions.items(), 
        key=lambda x: x[1][0]
    )
    
    logger.info(
        f"[NAV2 GOAL] Best escape direction: {best_dir_name} "
        f"(clearance: {best_clearance:.2f}m)"
    )
    
    # Calculate goal position
    escape_theta = current_theta + angle_offset
    
    # Normalize angle to [-pi, pi]
    while escape_theta > np.pi:
        escape_theta -= 2 * np.pi
    while escape_theta < -np.pi:
        escape_theta += 2 * np.pi
    
    escape_distance = 2.5  # meters
    goal_x = current_x + escape_distance * np.cos(escape_theta)
    goal_y = current_y + escape_distance * np.sin(escape_theta)
    
    logger.info(
        f"[NAV2 GOAL] Calculated escape: "
        f"Current: ({current_x:.2f}, {current_y:.2f}, {np.degrees(current_theta):.0f}°) "
        f"→ Goal: ({goal_x:.2f}, {goal_y:.2f}, {np.degrees(escape_theta):.0f}°)"
    )
    
    # Strategy 2: Validate with SLAM map if available
    if slam_controller and slam_controller.is_running:
        try:
            from multi_function_agent._robot_vision_controller.core.ros2_node.ros2_node import get_ros2_node
            ros_node = get_ros2_node()
            slam_map = ros_node.get_slam_map()
            
            if slam_map:
                # Check if goal position is in free space
                is_free = _check_map_position_free(
                    goal_x, goal_y, slam_map
                )
                
                if not is_free:
                    logger.warning(
                        f"[NAV2 GOAL] Goal ({goal_x:.2f}, {goal_y:.2f}) "
                        f"not in free space, trying alternatives..."
                    )
                    
                    # Try other directions in order of clearance
                    sorted_dirs = sorted(
                        directions.items(), 
                        key=lambda x: x[1][0], 
                        reverse=True
                    )[1:]  # Skip best (already tried)
                    
                    for alt_name, (alt_clear, alt_offset) in sorted_dirs:
                        alt_theta = current_theta + alt_offset
                        
                        # Normalize
                        while alt_theta > np.pi:
                            alt_theta -= 2 * np.pi
                        while alt_theta < -np.pi:
                            alt_theta += 2 * np.pi
                        
                        alt_x = current_x + escape_distance * np.cos(alt_theta)
                        alt_y = current_y + escape_distance * np.sin(alt_theta)
                        
                        if _check_map_position_free(alt_x, alt_y, slam_map):
                            logger.info(
                                f"[NAV2 GOAL] Found free alternative: {alt_name} "
                                f"({alt_x:.2f}, {alt_y:.2f})"
                            )
                            goal_x, goal_y, escape_theta = alt_x, alt_y, alt_theta
                            break
                    else:
                        logger.error("[NAV2 GOAL] No free space found in any direction")
                        return None
        
        except Exception as e:
            logger.warning(f"[NAV2 GOAL] Could not validate with map: {e}")
    
    return {
        'x': goal_x,
        'y': goal_y,
        'theta': escape_theta
    }


def _check_map_position_free(
    x: float, 
    y: float, 
    slam_map: Dict
) -> bool:
    """
    Check if position (x, y) is in free space on SLAM map.
    
    Note: Currently simplified validation (bounds check only).
    TODO: Add occupancy grid check when full map data available.
    """
    try:
        # Convert world coordinates to grid coordinates
        resolution = slam_map.get('resolution', 0.05)
        origin_x = slam_map['origin']['x']
        origin_y = slam_map['origin']['y']
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Check bounds
        width = slam_map.get('width', 0)
        height = slam_map.get('height', 0)
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            logger.warning(
                f"[MAP CHECK] Position out of map bounds: "
                f"grid=({grid_x},{grid_y}), map=({width}x{height})"
            )
            return False
        
        # TODO: Check actual occupancy data when available
        # For now, assume position is free if within bounds
        logger.debug(
            f"[MAP CHECK] Position within bounds: "
            f"world=({x:.2f},{y:.2f}), grid=({grid_x},{grid_y})"
        )
        
        return True
        
    except Exception as e:
        logger.error(f"[MAP CHECK] Error: {e}")
        return False
    
async def _execute_nav2_rescue(
    robot_interface: 'RobotControllerInterface',
    goal: Dict,
    timeout: float,
    safety_monitor: 'LidarSafetyMonitor'
) -> bool:
    """
    Execute Nav2 rescue with safety monitoring.
    
    Strategy:
    1. Send goal to Nav2
    2. Monitor progress every 0.5s
    3. Abort if critical obstacle detected
    4. Timeout after specified duration
    """
    logger.info("[NAV2 RESCUE] Starting rescue navigation...")
    
    # Send Nav2 goal (non-blocking)
    success = await robot_interface.send_nav2_goal(
        x=goal['x'],
        y=goal['y'],
        theta=goal['theta'],
        blocking=False
    )
    
    if not success:
        logger.error("[NAV2 RESCUE] Failed to send goal")
        return False
    
    # Monitor progress
    start_time = time.time()
    last_log_time = start_time
    
    while time.time() - start_time < timeout:
        # Check Nav2 state
        nav2_state = robot_interface.get_nav2_state()
        
        if nav2_state == 'succeeded':
            logger.info("[NAV2 RESCUE] ✅ Goal reached!")
            return True
        
        if nav2_state in ['failed', 'cancelled']:
            logger.error(f"[NAV2 RESCUE] ❌ Navigation {nav2_state}")
            return False
        
        # Safety check: abort if critical obstacle
        lidar_data = robot_interface.lidar_data
        if lidar_data:
            abort_result = safety_monitor.check_critical_abort(
                lidar_data,
                robot_pos=robot_interface.ros_node.get_robot_pose()
            )
            
            if abort_result['abort']:
                logger.error(
                    f"[NAV2 RESCUE] Safety abort at "
                    f"{abort_result['min_distance']:.3f}m"
                )
                robot_interface.cancel_nav2_navigation()
                
                # Execute safety command
                await robot_interface.execute_command(abort_result['command'])
                await asyncio.sleep(0.5)
                return False
        
        # Log progress every 2s
        current_time = time.time()
        if current_time - last_log_time > 2.0:
            elapsed = current_time - start_time
            logger.info(
                f"[NAV2 RESCUE] Progress: {elapsed:.1f}s/{timeout:.1f}s, "
                f"state={nav2_state}"
            )
            last_log_time = current_time
        
        await asyncio.sleep(0.5)
    
    # Timeout
    logger.error(f"[NAV2 RESCUE] ⏱️ Timeout after {timeout:.1f}s")
    robot_interface.cancel_nav2_navigation()
    return False

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
                await robot_interface.execute_command(abort_result['command'])
                results["navigation_decisions"].append({
                    'action': 'critical_abort',
                    'distance': abort_result['min_distance'],
                    'reason': abort_result.get('reason', 'critical')
                })
                await asyncio.sleep(0.1)
                continue

            elif abort_result['command'] is not None:
                await robot_interface.execute_command(abort_result['command'])
                await asyncio.sleep(0.1)
                continue

            # STEP 2: Nav2 Rescue Check
            if nav2_ready and safety_monitor.should_trigger_nav2_rescue():
                logger.error(
                    "[NAV2 RESCUE] Force escape failed → Activating global planner"
                )
                
                # Get current clearances from last abort
                clearances = abort_result.get('clearances', {}) if abort_result else {}
                
                # Calculate escape goal
                escape_goal = _calculate_nav2_escape_goal(
                    robot_pos=robot_pos,
                    clearances=clearances,
                    slam_controller=slam_controller,
                    stuck_type='force_escape_timeout'
                )
                
                if escape_goal:
                    logger.warning(
                        f"[NAV2 RESCUE] Sending global escape goal: "
                        f"({escape_goal['x']:.2f}, {escape_goal['y']:.2f})"
                    )
                    
                    rescue_success = await _execute_nav2_rescue(
                        robot_interface=robot_interface,
                        goal=escape_goal,
                        timeout=30.0,
                        safety_monitor=safety_monitor
                    )
                    
                    if rescue_success:
                        # Reset safety monitor
                        safety_monitor.reset_after_nav2_rescue()
                        
                        logger.info("[NAV2 RESCUE] ✅ Escaped successfully, resuming mission")
                        results["navigation_decisions"].append({
                            'action': 'nav2_rescue_success',
                            'reason': 'global_planner_escape'
                        })
                        
                        # Skip vision/navigation for this iteration
                        await asyncio.sleep(0.5)
                        continue
                    else:
                        logger.error(
                            "[NAV2 RESCUE] ❌ Failed after 30s, mission abort"
                        )
                        results["final_status"] = "nav2_rescue_failed"
                        break
                else:
                    logger.error("[NAV2 RESCUE] Could not calculate escape goal, mission abort")
                    results["final_status"] = "no_escape_goal"
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
    
    await stream_handler.stop_stream()
    
    if results["final_status"] == "unknown":
        results["final_status"] = "completed"
    
    return results