from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda").to_moveit_configs()

    cartesian_demo = Node(
        name="cartesian_tutorial",
        package="move_group_examples",
        executable="cartesian_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([cartesian_demo])
