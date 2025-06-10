#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    package_name = 'inspection_cell_description'
    
    urdf_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        'test.urdf.xacro'
    ])
    
    robot_description = Command(['xacro ', urdf_file])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])