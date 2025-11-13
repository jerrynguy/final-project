#!/usr/bin/env python3
"""
Standalone Integration Test Script
Tests ROS2 Bridge → Vision → Navigation → Mission layers independently.
"""

import sys
import time
import asyncio
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)

# Add project to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))


class IntegrationTester:
    """Orchestrates multi-layer integration tests."""
    
    def __init__(self):
        self.results = {}
        self.ros2_node = None
        self.robot_interface = None
    
    # =========================================================================
    # LAYER 1: ROS2 Bridge Test
    # =========================================================================
    def test_ros2_bridge(self) -> bool:
        """Test ROS2 subprocess bridge connectivity."""
        logger.info("\n" + "="*60)
        logger.info("LAYER 1: ROS2 BRIDGE TEST")
        logger.info("="*60)
        
        try:
            from multi_function_agent._robot_vision_controller.core.ros2_node import get_ros2_node
            
            logger.info("Initializing ROS2 bridge...")
            self.ros2_node = get_ros2_node()
            
            logger.info("Waiting 3s for daemon startup...")
            time.sleep(3)
            
            # Test LIDAR
            scan = self.ros2_node.get_scan()
            if scan and hasattr(scan, 'ranges'):
                lidar_points = len(scan.ranges)
                logger.info(f"✅ LIDAR: {lidar_points} points")
                self.results['lidar'] = True
            else:
                logger.error("❌ LIDAR: No data")
                self.results['lidar'] = False
                return False
            
            # Test Odometry
            odom = self.ros2_node.get_odom()
            if odom:
                logger.info(f"✅ Odom: {odom['position']}")
                self.results['odom'] = True
            else:
                logger.error("❌ Odom: No data")
                self.results['odom'] = False
                return False
            
            # Test Robot Pose
            pose = self.ros2_node.get_robot_pose()
            if pose:
                logger.info(f"✅ Pose: x={pose['x']:.2f}, y={pose['y']:.2f}, θ={pose['theta']:.2f}")
                self.results['pose'] = True
            else:
                logger.error("❌ Pose: No data")
                self.results['pose'] = False
                return False
            
            logger.info("\n✅ LAYER 1 PASSED: ROS2 Bridge operational\n")
            return True
            
        except Exception as e:
            logger.error(f"❌ LAYER 1 FAILED: {e}")
            self.results['ros2_bridge'] = False
            return False
    
    # =========================================================================
    # LAYER 2: Nav2 Connection Test
    # =========================================================================
    # Line 295-335: REPLACE toàn bộ test_nav2_connection()

    async def test_nav2_connection(self) -> bool:
        """Test Nav2 interface availability."""
        logger.info("\n" + "="*60)
        logger.info("LAYER 2: NAV2 CONNECTION TEST")
        logger.info("="*60)
        
        try:
            from multi_function_agent._robot_vision_controller.navigation.robot_controller_interface import RobotControllerInterface
            
            self.robot_interface = RobotControllerInterface()
            
            # ✅ FIX: THÊM AWAIT CONNECT()
            logger.info("Connecting to robot controller...")
            connected = await self.robot_interface.connect()
            
            if not connected:
                logger.warning("⚠️  Robot controller connection failed")
                self.results['robot_connected'] = False
            else:
                logger.info("✅ Robot controller connected")
                self.results['robot_connected'] = True
            
            # Check if Nav2 enabled
            if not self.robot_interface.use_nav2:
                logger.warning("⚠️  Nav2 disabled in config")
                self.results['nav2_enabled'] = False
                logger.info("\n✅ LAYER 2 PASSED: Nav2 interface checked\n")
                return True  # Not a failure
            
            # Check Nav2 ready
            logger.info("Waiting for Nav2 (10s timeout)...")
            nav2_ready = await self.robot_interface.wait_for_nav2_ready(timeout=10.0)
            
            if nav2_ready:
                logger.info("✅ Nav2 ready")
                self.results['nav2_ready'] = True
            else:
                logger.warning("⚠️  Nav2 not ready (timeout)")
                self.results['nav2_ready'] = False
            
            logger.info("\n✅ LAYER 2 PASSED: Nav2 interface checked\n")
            return True
            
        except Exception as e:
            logger.error(f"❌ LAYER 2 FAILED: {e}")
            self.results['nav2_connection'] = False
            return False
    
    # =========================================================================
    # LAYER 3: Movement Command Test
    # =========================================================================
    async def test_movement_commands(self) -> bool:
        """Test basic robot movement commands."""
        logger.info("\n" + "="*60)
        logger.info("LAYER 3: MOVEMENT COMMAND TEST")
        logger.info("="*60)
        
        if not self.robot_interface:
            logger.error("❌ Robot interface not initialized")
            return False
        
        try:
            # Test 1: Stop command
            logger.info("Test 3.1: Sending STOP command...")
            stop_cmd = {
                'action': 'stop',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.0,
                    'duration': 0.1
                }
            }
            success = await self.robot_interface.execute_command(stop_cmd)
            if success:
                logger.info("✅ STOP command sent")
                self.results['cmd_stop'] = True
            else:
                logger.error("❌ STOP command failed")
                self.results['cmd_stop'] = False
            
            time.sleep(0.5)
            
            # Test 2: Forward command (0.5s)
            logger.info("Test 3.2: Sending FORWARD command (0.5s)...")
            forward_cmd = {
                'action': 'move_forward',
                'parameters': {
                    'linear_velocity': 0.2,
                    'angular_velocity': 0.0,
                    'duration': 0.5
                }
            }
            success = await self.robot_interface.execute_command(forward_cmd)
            if success:
                logger.info("✅ FORWARD command sent")
                self.results['cmd_forward'] = True
            else:
                logger.error("❌ FORWARD command failed")
                self.results['cmd_forward'] = False
            
            time.sleep(0.5)
            
            # Test 3: Rotate command (0.5s)
            logger.info("Test 3.3: Sending ROTATE command (0.5s)...")
            rotate_cmd = {
                'action': 'rotate_left',
                'parameters': {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.5,
                    'duration': 0.5
                }
            }
            success = await self.robot_interface.execute_command(rotate_cmd)
            if success:
                logger.info("✅ ROTATE command sent")
                self.results['cmd_rotate'] = True
            else:
                logger.error("❌ ROTATE command failed")
                self.results['cmd_rotate'] = False
            
            time.sleep(0.5)
            
            # Final stop
            await self.robot_interface.execute_command(stop_cmd)
            
            logger.info("\n✅ LAYER 3 PASSED: Movement commands functional\n")
            return True
            
        except Exception as e:
            logger.error(f"❌ LAYER 3 FAILED: {e}")
            self.results['movement_commands'] = False
            return False
    
    # =========================================================================
    # LAYER 4: Mission Parsing Test
    # =========================================================================
    async def test_mission_parsing(self) -> bool:
        """Test mission parsing from natural language."""
        logger.info("\n" + "="*60)
        logger.info("LAYER 4: MISSION PARSING TEST")
        logger.info("="*60)
        
        try:
            from multi_function_agent._robot_vision_controller.core.goal_parser import Mission
            from multi_function_agent._robot_vision_controller.core.mission_controller import MissionController
            
            # Mock builder (simplified)
            class MockBuilder:
                class MockWorkflowBuilder:
                    class MockGeneralConfig:
                        class MockFrontEnd:
                            input_query = ["Run wide automatically in 60 seconds"]
                        front_end = MockFrontEnd()
                    general_config = MockGeneralConfig()
                _workflow_builder = MockWorkflowBuilder()
            
            mock_builder = MockBuilder()
            
            # Test prompt
            test_prompt = "Run wide automatically in 60 seconds"
            logger.info(f"Parsing prompt: '{test_prompt}'")
            
            mission_controller = await MissionController.from_prompt(test_prompt, mock_builder)
            
            logger.info(f"✅ Mission Type: {mission_controller.mission.type}")
            logger.info(f"✅ Description: {mission_controller.mission.description}")
            logger.info(f"✅ Parameters: {mission_controller.mission.parameters}")
            
            self.results['mission_parsing'] = True
            
            # Test directive generation
            directive = mission_controller.get_directive()
            logger.info(f"✅ Initial Directive: {directive}")
            
            logger.info("\n✅ LAYER 4 PASSED: Mission parsing functional\n")
            return True
            
        except Exception as e:
            logger.error(f"❌ LAYER 4 FAILED: {e}")
            self.results['mission_parsing'] = False
            return False
    
    # =========================================================================
    # LAYER 5: Nav2 Goal Test (if available)
    # =========================================================================
    async def test_nav2_goal(self) -> bool:
        """Test sending Nav2 navigation goal."""
        logger.info("\n" + "="*60)
        logger.info("LAYER 5: NAV2 GOAL TEST")
        logger.info("="*60)
        
        if not self.robot_interface:
            logger.warning("⚠️  Skipping - robot interface not available")
            return True
        
        if not self.results.get('nav2_ready', False):
            logger.warning("⚠️  Skipping - Nav2 not ready")
            return True
        
        try:
            # Get current position
            pose = self.ros2_node.get_robot_pose()
            if not pose:
                logger.error("❌ Cannot get robot pose")
                return False
            
            # Goal: 1m forward
            goal_x = pose['x'] + 1.0
            goal_y = pose['y']
            goal_theta = pose['theta']
            
            logger.info(f"Sending Nav2 goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.2f})")
            
            success = await self.robot_interface.send_nav2_goal(
                x=goal_x,
                y=goal_y,
                theta=goal_theta,
                blocking=False
            )
            
            if success:
                logger.info("✅ Nav2 goal accepted")
                
                # Monitor for 3 seconds
                for i in range(6):
                    await asyncio.sleep(0.5)
                    state = self.robot_interface.get_nav2_state()
                    logger.info(f"  Nav2 State: {state}")
                
                # Cancel goal
                logger.info("Canceling Nav2 goal...")
                self.robot_interface.cancel_nav2_navigation()
                
                self.results['nav2_goal'] = True
                logger.info("\n✅ LAYER 5 PASSED: Nav2 goal functional\n")
                return True
            else:
                logger.error("❌ Nav2 goal rejected")
                self.results['nav2_goal'] = False
                return False
            
        except Exception as e:
            logger.error(f"❌ LAYER 5 FAILED: {e}")
            self.results['nav2_goal'] = False
            return False
    
    # =========================================================================
    # Summary
    # =========================================================================
    def print_summary(self):
        """Print test results summary."""
        logger.info("\n" + "="*60)
        logger.info("TEST SUMMARY")
        logger.info("="*60)
        
        total = len(self.results)
        passed = sum(1 for v in self.results.values() if v)
        
        for test, result in self.results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            logger.info(f"{status}: {test}")
        
        logger.info("-"*60)
        logger.info(f"Total: {passed}/{total} tests passed")
        logger.info("="*60 + "\n")
        
        return passed == total
    
    # =========================================================================
    # Cleanup
    # =========================================================================
    async def cleanup(self):
        """Cleanup resources."""
        logger.info("Cleaning up...")
        
        try:
            if self.robot_interface:
                await self.robot_interface.execute_command({
                    'action': 'stop',
                    'parameters': {'linear_velocity': 0.0, 'angular_velocity': 0.0, 'duration': 0.1}
                })
        except Exception as e:
            logger.debug(f"Cleanup command failed: {e}")
        
        try:
            from multi_function_agent._robot_vision_controller.core.ros2_node import shutdown_ros2
            shutdown_ros2()
        except Exception as e:
            logger.debug(f"Shutdown failed: {e}")


# =============================================================================
# Main Test Runner
# =============================================================================

async def run_tests():
    """Run all integration tests."""
    tester = IntegrationTester()
    
    try:
        # Layer 1: ROS2 Bridge
        if not tester.test_ros2_bridge():
            logger.error("CRITICAL: ROS2 bridge failed - cannot continue")
            return False
        
        # Layer 2: Nav2 Connection
        await tester.test_nav2_connection()
        
        # Layer 3: Movement Commands
        await tester.test_movement_commands()
        
        # Layer 4: Mission Parsing
        await tester.test_mission_parsing()
        
        # Layer 5: Nav2 Goals (optional)
        await tester.test_nav2_goal()
        
        # Summary
        all_passed = tester.print_summary()
        
        return all_passed
        
    finally:
        await tester.cleanup()


if __name__ == "__main__":
    logger.info("Starting Integration Tests...")
    
    try:
        result = asyncio.run(run_tests())
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        logger.info("\nTests interrupted by user")
        sys.exit(130)
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)