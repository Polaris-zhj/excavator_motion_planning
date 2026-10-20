"""Headless visualization data node (no RViz).

Useful for headless tests and for feeding an already running RViz instance.
Starts no controller or simulator.
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
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="false"),
        DeclareLaunchArgument("occupancy_enabled", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[
                ("robot_description", "/excavator/robot_description")],
            condition=IfCondition(
                LaunchConfiguration("start_robot_state_publisher")),
            output="screen",
        ),
        Node(
            package="excavator_visualization",
            executable="trajectory_visualizer_node",
            name="excavator_trajectory_visualizer",
            parameters=[{
                "robot_description": robot_description,
                "frame_id": "base_link",
                "tip_link": "bucket_tip",
                "occupancy_enabled": LaunchConfiguration("occupancy_enabled"),
            }],
            output="screen",
        ),
        Node(
            package="excavator_visualization",
            executable="target_marker_node",
            name="excavator_target_marker",
            parameters=[{
                "robot_description": robot_description,
                "frame_id": "base_link",
                "tip_link": "bucket_tip",
            }],
            output="screen",
        ),
    ])
