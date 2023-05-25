from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import xacro
import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

distro = os.environ['ROS_DISTRO']
if distro == 'humble' or distro == 'galactic':
    spawner = "spawner"
else:  # foxy
    spawner = "spawner.py"


def generate_launch_description():
    use_sim_time = True
    # Declare arguments
    declared_arguments = []
    description_package = get_package_share_directory('franka_coppelia_hw')
    urdf_path = os.path.join(description_package, "urdf", "panda.urdf")
    initial_joint_controllers = os.path.join(
        description_package, "config", "franka_controllers_coppelia.yaml")

    franka_xacro_file = os.path.join(get_package_share_directory(
        'franka_coppelia_hw'), 'urdf', 'panda_arm.urdf.xacro')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', 'false',
            ' robot_ip:=', 'xxx.yyy.zzz.www', ' use_fake_hardware:=', 'true',
            ' fake_sensor_commands:=', 'false'])

    rviz_file = os.path.join(get_package_share_directory('franka_coppelia_hw'), 'rviz',
                             'visualize_franka.rviz')
    robot_controllers = initial_joint_controllers

    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    urdf_path = os.path.join(description_package,"urdf","panda.urdf")
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description} , robot_controllers],
        # prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
        remappings=[
            ('cartesian_impedance_controller/target_frame', 'target_frame'),
            ('cartesian_impedance_controller/target_wrench', 'target_wrench'),
            ('motion_control_handle/target_frame', 'target_frame'),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_impedance_controller",
                   "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["motion_control_handle", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #output="both",
        parameters=[{"robot_description": robot_description}],
    )

    # Visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        parameters=[{"use_sim_time": use_sim_time}]
    )




    # Nodes to start
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        cartesian_impedance_controller_spawner,
        motion_control_handle_spawner,
        robot_state_publisher,
        rviz
    ]

    return LaunchDescription(declared_arguments + nodes)





