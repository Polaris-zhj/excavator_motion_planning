from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="excavator_sim_viz",
            executable="trajectory_player",
            name="excavator_trajectory_player",
            output="screen",
            parameters=[{"publish_rate_hz": 50.0}],
        )
    ])
