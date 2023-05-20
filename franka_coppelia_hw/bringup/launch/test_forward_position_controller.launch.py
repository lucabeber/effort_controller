from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("franka_coppelia_hw"),
            "config",
            "target_test_forward_position_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="publisher_forward_position_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )
