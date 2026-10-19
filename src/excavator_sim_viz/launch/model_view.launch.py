"""Simulation demo: model, TF, trajectory controller, visualization and RViz.

The planning module is intentionally not started here. Start it separately:
  ros2 launch excavator_planning planning_server.launch.py
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
        "excavator.urdf.xacro",
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
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain for every node started here; keeps the "
                        "model isolated from other robots on the network"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_visualization", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            # Project-private topic, immune to other robots' /robot_description.
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
                "robot_description": robot_description,
                "joint_names": [
                    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"
                ],
                "initial_positions": [0.0, -0.35, 0.45, -0.4],
                "control_rate_hz": 100.0,
                "max_collision_joint_step": 0.03,
                "require_collision_check": True,
                "allow_simulator_initialization": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="excavator_visualization",
            executable="trajectory_visualizer_node",
            name="excavator_trajectory_visualizer",
            condition=IfCondition(LaunchConfiguration("start_visualization")),
            parameters=[{
                "robot_description": robot_description,
                "frame_id": "base_link",
                "tip_link": "bucket_tip",
                "occupancy_enabled": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
        Node(
            package="excavator_visualization",
            executable="target_marker_node",
            name="excavator_target_marker",
            condition=IfCondition(LaunchConfiguration("start_visualization")),
            parameters=[{
                "robot_description": robot_description,
                "frame_id": "base_link",
                "tip_link": "bucket_tip",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
