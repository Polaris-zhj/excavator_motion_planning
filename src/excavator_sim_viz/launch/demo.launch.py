"""All-in-one demo: simulation, controller, visualization, RViz and planner.

The modules remain independently launchable; this file only composes the
standard launch files for convenience. Use planner_class to select a planner
plugin, for example:
  ros2 launch excavator_sim_viz demo.launch.py \
    planner_class:=excavator_planner_rrtstar/RrtStarPlanner
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
            "domain_id": LaunchConfiguration("domain_id"),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument(
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain; must match the other excavator nodes"),
        DeclareLaunchArgument(
            "planner_class",
            default_value="excavator_planning/LinearPlannerPlugin"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        model_view,
        planning,
    ])
