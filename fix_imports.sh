#!/bin/bash
# Auto-fix ROS2 imports to use stubs fallback

ROBOT_DIR="$HOME/nemo-agent-toolkit/examples/multi_function_agent/src/multi_function_agent/robot_vision_controller"

# Create stubs file
cat > "$ROBOT_DIR/utils/ros2_stubs.py" << 'PYTHON_EOF'
"""
ROS2 message stubs for Python 3.11 compatibility.
Actual ROS2 communication happens via subprocess bridge.
"""

class _MockMessage:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

# geometry_msgs.msg
class Vector3(_MockMessage):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

class Twist(_MockMessage):
    def __init__(self, linear=None, angular=None):
        self.linear = linear or Vector3()
        self.angular = angular or Vector3()

class Point(_MockMessage):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

class Quaternion(_MockMessage):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w

class Pose(_MockMessage):
    def __init__(self, position=None, orientation=None):
        self.position = position or Point()
        self.orientation = orientation or Quaternion()

class PoseStamped(_MockMessage):
    def __init__(self):
        self.header = type('Header', (), {'frame_id': 'map'})()
        self.pose = Pose()

# sensor_msgs.msg
class LaserScan(_MockMessage):
    def __init__(self):
        self.ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

# nav_msgs.msg
class Odometry(_MockMessage):
    pass

class OccupancyGrid(_MockMessage):
    pass

class Path(_MockMessage):
    pass

# std_msgs.msg
class String(_MockMessage):
    pass

class Bool(_MockMessage):
    def __init__(self, data=False):
        self.data = data

# nav2_msgs.action
class NavigateToPose:
    Goal = _MockMessage
    Result = _MockMessage
    Feedback = _MockMessage
PYTHON_EOF

echo "✅ Created ros2_stubs.py"

# Function to fix imports in a file
fix_file() {
    local file="$1"
    echo "Processing: $file"
    
    # Backup
    cp "$file" "$file.bak"
    
    # Fix geometry_msgs
    sed -i 's/^from geometry_msgs\.msg import \(.*\)$/try:\n    from geometry_msgs.msg import \1\nexcept ImportError:\n    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import \1/' "$file"
    
    # Fix sensor_msgs
    sed -i 's/^from sensor_msgs\.msg import \(.*\)$/try:\n    from sensor_msgs.msg import \1\nexcept ImportError:\n    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import \1/' "$file"
    
    # Fix nav_msgs
    sed -i 's/^from nav_msgs\.msg import \(.*\)$/try:\n    from nav_msgs.msg import \1\nexcept ImportError:\n    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import \1/' "$file"
    
    # Fix std_msgs
    sed -i 's/^from std_msgs\.msg import \(.*\)$/try:\n    from std_msgs.msg import \1\nexcept ImportError:\n    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import \1/' "$file"
    
    # Fix nav2_msgs
    sed -i 's/^from nav2_msgs\.action import \(.*\)$/try:\n    from nav2_msgs.action import \1\nexcept ImportError:\n    from multi_function_agent.robot_vision_controller.utils.ros2_stubs import \1/' "$file"
}

# Fix all files
fix_file "$ROBOT_DIR/utils/movement_commands.py"
fix_file "$ROBOT_DIR/utils/safety_checks.py"
fix_file "$ROBOT_DIR/navigation/robot_controller_interface.py"
fix_file "$ROBOT_DIR/navigation/nav2_interface.py"
fix_file "$ROBOT_DIR/perception/spatial_detector.py"
fix_file "$ROBOT_DIR/core/ros2_node.py"

echo "✅ All files fixed! Backups saved as .bak"
echo "To restore: for f in $ROBOT_DIR/**/*.py.bak; do mv \$f \${f%.bak}; done"
