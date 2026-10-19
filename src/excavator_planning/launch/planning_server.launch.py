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
        "excavator.urdf.xacro",
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "domain_id", default_value=default_domain_id,
            description="ROS 2 domain; must match the other excavator nodes"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        DeclareLaunchArgument(
            "planner_class",
            default_value="excavator_planning/LinearPlannerPlugin",
            description="pluginlib class of the path planner"),
        DeclareLaunchArgument("planner_max_iterations", default_value="2000"),
        DeclareLaunchArgument("planner_step_size", default_value="0.25"),
        DeclareLaunchArgument("planner_goal_bias", default_value="0.10"),
        DeclareLaunchArgument("planner_rewire_radius", default_value="0.60"),
        DeclareLaunchArgument("planner_goal_tolerance", default_value="0.15"),
        DeclareLaunchArgument("planner_random_seed", default_value="42"),
        Node(
            package="excavator_planning",
            executable="planning_server",
            name="planning_server",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", model_file]), value_type=str),
                "joint_names": [
                    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"
                ],
                "planner_class": LaunchConfiguration("planner_class"),
                "planner_max_iterations": ParameterValue(
                    LaunchConfiguration("planner_max_iterations"),
                    value_type=int),
                "planner_step_size": ParameterValue(
                    LaunchConfiguration("planner_step_size"),
                    value_type=float),
                "planner_goal_bias": ParameterValue(
                    LaunchConfiguration("planner_goal_bias"), value_type=float),
                "planner_rewire_radius": ParameterValue(
                    LaunchConfiguration("planner_rewire_radius"),
                    value_type=float),
                "planner_goal_tolerance": ParameterValue(
                    LaunchConfiguration("planner_goal_tolerance"),
                    value_type=float),
                "planner_random_seed": ParameterValue(
                    LaunchConfiguration("planner_random_seed"),
                    value_type=int),
                "max_joint_step": 0.05,
                "max_collision_joint_step": 0.03,
                "max_detour_attempts": 12,
                "simplify_path": True,
                "smooth_path": False,
                "smoothing_iterations": 1,
                "smoothing_alpha": 0.2,
                "max_acceleration": [1.0, 0.6, 0.8, 1.0],
            }],
        )
    ])
