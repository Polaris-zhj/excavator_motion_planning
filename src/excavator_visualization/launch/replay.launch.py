"""Offline replay: publish a saved trajectory and its measured state stream.

No simulator, controller or planner is started. The replay node is the only
authoritative /joint_states source in this mode.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_domain_id = os.environ.get("ROS_DOMAIN_ID", "77")
    model_file = PathJoinSubstitution([
        FindPackageShare("excavator_description"),
        "urdf",
        LaunchConfiguration("model"),
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("excavator_visualization"),
        "config",
        "visualization.rviz",
    ])
    robot_description = ParameterValue(
        Command(["xacro ", model_file]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument(
            "model", default_value="excavator.urdf.xacro",
            description="URDF xacro under excavator_description/urdf "
                        "(excavator_box.urdf.xacro for the box template)"),
        DeclareLaunchArgument(
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain for every node started here"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        DeclareLaunchArgument("trajectory_file"),
        DeclareLaunchArgument("replay_rate", default_value="1.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[
                ("robot_description", "/excavator/robot_description")],
            output="screen",
        ),
        Node(
            package="excavator_visualization",
            executable="trajectory_replay_node",
            name="excavator_trajectory_replay",
            parameters=[{
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "replay_rate": LaunchConfiguration("replay_rate"),
                "loop": LaunchConfiguration("loop"),
                "publish_joint_states": True,
            }],
            output="screen",
        ),
        Node(
            package="excavator_visualization",
            executable="trajectory_visualizer_node",
            name="excavator_trajectory_visualizer",
            parameters=[{
                "robot_description": robot_description,
                "occupancy_enabled": True,
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            output="screen",
        ),
    ])
