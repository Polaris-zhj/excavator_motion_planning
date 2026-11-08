"""All-in-one demo: simulation, controller, visualization, RViz and planner.

The modules remain independently launchable; this file only composes the
standard launch files for convenience. Use planner_class to select a planner
plugin, for example:
  ros2 launch excavator_sim_viz demo.launch.py \
    planner_class:=excavator_planner_rrtstar/RrtStarPlanner
Use time_parameterizer_class to switch between the baseline and TOPP-RA:
  ros2 launch excavator_sim_viz demo.launch.py time_parameterizer_class:=toppra
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_domain_id = os.environ.get("ROS_DOMAIN_ID", "77")
    model_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("excavator_sim_viz"), "launch",
                "model_view.launch.py"
            ])
        ]),
        launch_arguments={
            "domain_id": LaunchConfiguration("domain_id"),
            "model": LaunchConfiguration("model"),
            "collision_backend": LaunchConfiguration("collision_backend"),
            "collision_model_file": LaunchConfiguration("collision_model_file"),
            "dh_parameters_file": LaunchConfiguration("dh_parameters_file"),
            "start_collision_test": LaunchConfiguration(
                "start_collision_test"),
        }.items())
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("excavator_planning"), "launch",
                "planning_server.launch.py"
            ])
        ]),
        launch_arguments={
            "planner_class": LaunchConfiguration("planner_class"),
            "time_parameterizer_class": LaunchConfiguration(
                "time_parameterizer_class"),
            "toppra_grid_points": LaunchConfiguration("toppra_grid_points"),
            "domain_id": LaunchConfiguration("domain_id"),
            "model": LaunchConfiguration("model"),
            "kinematics_backend": LaunchConfiguration("kinematics_backend"),
            "collision_backend": LaunchConfiguration("collision_backend"),
            "collision_model_file": LaunchConfiguration("collision_model_file"),
            "dh_parameters_file": LaunchConfiguration("dh_parameters_file"),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument(
            "model", default_value="excavator.urdf.xacro",
            description="URDF xacro under excavator_description/urdf "
                        "(excavator_box.urdf.xacro for the box template)"),
        DeclareLaunchArgument(
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain; must match the other excavator nodes"),
        DeclareLaunchArgument(
            "planner_class",
            default_value="excavator_planning/LinearPlannerPlugin"),
        DeclareLaunchArgument(
            "time_parameterizer_class", default_value="simple",
            description="time parameterization algorithm: simple or toppra"),
        DeclareLaunchArgument(
            "toppra_grid_points", default_value="200",
            description="TOPP-RA discretization intervals (samples = N + 1)"),
        DeclareLaunchArgument(
            "kinematics_backend", default_value="dh"),
        DeclareLaunchArgument(
            "collision_backend", default_value="fcl"),
        DeclareLaunchArgument(
            "start_collision_test", default_value="true",
            description="Start the interactive collision-test node and "
                        "bucket-tip jog marker"),
        DeclareLaunchArgument(
            "dh_parameters_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("excavator_description"),
                "config", "excavator_dh.yaml"])),
        DeclareLaunchArgument(
            "collision_model_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("excavator_description"),
                "config", "excavator_collision.yaml"])),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        model_view,
        planning,
    ])
