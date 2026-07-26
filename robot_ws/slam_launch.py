#!/usr/bin/env python3
"""Launch: static TF base_link->laser + slam_toolbox (online async)"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        # x y z yaw pitch roll base_link laser (lidar ở tâm robot, không xoay)
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser'],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([static_tf, slam])