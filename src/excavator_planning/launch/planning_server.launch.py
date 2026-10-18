from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = PathJoinSubstitution([
        FindPackageShare("excavator_description"),
        "urdf",
        "excavator.urdf.xacro",
    ])
    return LaunchDescription([
        Node(
            package="excavator_planning",
            executable="planning_server",
            name="planning_server",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro ", model_file]),
                "joint_names": [
                    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"
                ],
                "max_joint_step": 0.05,
                "max_collision_joint_step": 0.03,
                "max_acceleration": [1.0, 0.6, 0.8, 1.0],
            }],
        )
    ])
