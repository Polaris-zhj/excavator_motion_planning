"""Independent visualization launch.

Starts no trajectory controller and no simulator. A static joint-state
publisher is included by default so that the robot model is visible without a
simulator or replay source. When running together with
excavator_sim_viz/model_view.launch.py, set both
start_robot_state_publisher:=false and start_static_joint_states:=false so that
only one authoritative /joint_states source and TF publisher exist.
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
    # All nodes of this project must share one domain. The default ROS domain
    # (0) is usually shared with other robots/PCs on the same LAN; their
    # /robot_description messages would otherwise be picked up by RViz.
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
            description="ROS 2 domain for every node started here; keeps the "
                        "model isolated from other robots on the network"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "start_static_joint_states", default_value="true"),
        DeclareLaunchArgument("occupancy_enabled", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            # Publish on a project-private topic: other robots on the same
            # network may publish their own /robot_description.
            remappings=[
                ("robot_description", "/excavator/robot_description")],
            condition=IfCondition(
                LaunchConfiguration("start_robot_state_publisher")),
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="static_joint_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[
                ("robot_description", "/excavator/robot_description")],
            condition=IfCondition(
                LaunchConfiguration("start_static_joint_states")),
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
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
