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