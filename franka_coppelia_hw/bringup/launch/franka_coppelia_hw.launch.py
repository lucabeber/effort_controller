# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import xacro,os
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
    urdf_path = os.path.join(description_package,"urdf","panda.urdf")
    initial_joint_controllers = os.path.join(description_package,"config", "franka_controllers_coppelia.yaml")
    print(initial_joint_controllers)
    
    # franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
    #                                 'panda_arm.urdf.xacro')
    # robot_description = Command(
    #    [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper])

    rviz_file = os.path.join(get_package_share_directory('franka_coppelia_hw'), 'rviz',
                             'visualize_franka.rviz')
    print(rviz_file)
    robot_controllers = initial_joint_controllers
    robot_description = {"robot_description": open(urdf_path, 'r').read()}
             
    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers,{"use_sim_time": use_sim_time}],
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
        remappings=[
            ('cartesian_impedance_controller/target_frame', 'target_frame'),
            ('cartesian_impedance_controller/target_wrench', 'target_wrench'),
            ]
    )

    # Joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_impedance_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )    
    
    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": use_sim_time}],
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
        robot_state_publisher,
        rviz
    ]

    return LaunchDescription(declared_arguments + nodes)
