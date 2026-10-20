"""Headless kinematic simulation: model, TF and trajectory controller.

This launch file starts no planning node and no RViz. Start the planner
separately with excavator_planning/planning_server.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
    return LaunchDescription([
        DeclareLaunchArgument(
            "model", default_value="excavator.urdf.xacro",
            description="URDF xacro under excavator_description/urdf "
                        "(excavator_box.urdf.xacro for the box template)"),
        DeclareLaunchArgument(
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain; must match the other excavator nodes"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(
                Command(["xacro ", model_file]), value_type=str)}],
            remappings=[
                ("robot_description", "/excavator/robot_description")],
            output="screen",
        ),
        Node(
            package="excavator_sim_viz",
            executable="trajectory_controller_node",
            name="excavator_trajectory_controller",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", model_file]), value_type=str),
                "joint_names": [
                    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"
                ],
                "initial_positions": [0.0, -0.35, 0.45, -0.4],
                "control_rate_hz": 100.0,
                "max_collision_joint_step": 0.03,
                "require_collision_check": True,
                "allow_simulator_initialization": True,
            }],
        ),
    ])
