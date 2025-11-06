from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='waypoint_navigator.py',
            name='waypoint_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
