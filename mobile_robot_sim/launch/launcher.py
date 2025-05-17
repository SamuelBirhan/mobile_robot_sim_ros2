import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Simulator Node
        Node(
            package='mobile_robot_sim',
            executable='simulator',
            name='mobile_robot_simulator',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mobile_robot_sim'),
                    'config',
                    'simulator_params.yaml'
                ])
            ],
        ),
        
        # Controller Node
        Node(
            package='mobile_robot_sim',
            executable='controller',
            name='mobile_robot_controller',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mobile_robot_sim'),
                    'config',
                    'controller_params.yaml'
                ])
            ],
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('mobile_robot_sim'),
                    'rviz',
                    'rviz_config.rviz'
                ])
            ]
        ),
    ])
