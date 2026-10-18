from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = PathJoinSubstitution([
        FindPackageShare("excavator_description"),
        "urdf",
        "excavator.urdf.xacro",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("excavator_sim_viz"),
        "config",
        "excavator.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", model_file]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
        Node(
            package="excavator_sim_viz",
            executable="trajectory_player",
            name="excavator_trajectory_player",
            output="screen",
        ),
        Node(
            package="excavator_sim_viz",
            executable="trajectory_executor",
            name="excavator_trajectory_executor",
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
