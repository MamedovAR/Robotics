# import launch
# from launch.substitutions import Command, LaunchConfiguration
# import launch_ros
# import os
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.actions import IncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import TimerAction

# def generate_launch_description():
#     pkg_share = launch_ros.substitutions.FindPackageShare(package='notsam').find('notsam')
#     default_model_path = os.path.join(pkg_share, 'src/description/sam.urdf')
#     default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     pkg_project_bringup = get_package_share_directory('notsam')
#     pkg_project_description = get_package_share_directory('notsam')

#     # Setup to launch the simulator and Gazebo world
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
#         launch_arguments={
#             'gz_args': '-r empty.sdf'
#         }.items(),
#     )

#     robot_state_publisher_node = launch_ros.actions.Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
#     )
#     joint_state_publisher_node = launch_ros.actions.Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
#     )
#     joint_state_publisher_gui_node = launch_ros.actions.Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         name='joint_state_publisher_gui',
#         condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
#     )
#     rviz_node = launch_ros.actions.Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', LaunchConfiguration('rvizconfig')],
#     )
    
#     rqt_robot_steering = launch_ros.actions.Node(
#         package='rqt_robot_steering',
#         executable='rqt_robot_steering'
#     )

#     # Spawn robot
#     create = launch_ros.actions.Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=['-name', 'robot',
#                    '-topic', 'robot_description',
#                    '-x', '0.0',
#                    '-y', '0.0',
#                    '-z', '0.1',
#                 ],
#         output='screen',
#     )

#     # Bridge ROS topics and Gazebo messages for establishing communication
#     bridge = launch_ros.actions.Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#             'config_file': os.path.join("notsam", 'config', 'ros_gz_example_bridge.yaml'),
#             'qos_overrides./tf_static.publisher.durability': 'transient_local',
#         }],
#         output='screen'
#     )

#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
#                                             description='Flag to enable joint_state_publisher_gui'),
#         launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
#                                             description='Absolute path to robot urdf file'),
#         launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
#                                             description='Absolute path to rviz config file'),
#         joint_state_publisher_node,
#         joint_state_publisher_gui_node,
#         robot_state_publisher_node,
#         rviz_node,
#         rqt_robot_steering,
#         bridge,
#         gz_sim,
#         TimerAction(
#             period=5.0,
#             actions=[create])
#     ])

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.actions import Node
# from launch.actions import TimerAction


# def generate_launch_description():
#     # Configure ROS nodes for launch

#     # Setup project paths
#     pkg_project_bringup = get_package_share_directory('notsam')
#     pkg_project_description = get_package_share_directory('notsam')
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

#     # # Load the URDF file from "description" package
#     # urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'robot.urdf')
#     # with open(urdf_file, 'r') as infp:
#     #     robot_desc = infp.read()

#     urdf_path  =  os.path.join(pkg_project_description, 'src/description', 'sam.sdf')
#     robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

#     # Setup to launch the simulator and Gazebo world
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
#         launch_arguments={'gz_args': "-r empty.sdf"}.items(),
#     )

#     # Spawn robot
#     create = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=['-name', 'robot',
#                    '-topic', 'robot_description',
#                    '-x', '0.0',
#                    '-y', '0.0',
#                    '-z', '0.1',
#                 ],
#         output='screen',
#     )

#     # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='both',
#         parameters=[
#             {'robot_description': robot_desc},
#             {'frame_prefix': "robot/"}
#         ]
#     )

#     # Visualize in RViz
#     rviz = Node(
#        package='rviz2',
#        executable='rviz2',
#        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
#        condition=IfCondition(LaunchConfiguration('rviz'))
#     )

#     # Bridge ROS topics and Gazebo messages for establishing communication
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#             'config_file': os.path.join(pkg_project_bringup, 'config', 'robot_bridge.yaml'),
#             'qos_overrides./tf_static.publisher.durability': 'transient_local',
#         }],
#         output='screen'
#     )

#     return LaunchDescription([
#         gz_sim,
#         DeclareLaunchArgument('rviz', default_value='true',
#                               description='Open RViz.'),
#         bridge,
#         robot_state_publisher,
#         rviz,
#         TimerAction(
#             period=5.0,
#             actions=[create])
#     ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction
import launch_ros.substitutions


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='notsam').find('notsam')
    default_model_path = os.path.join(pkg_share, 'urdf/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('notsam')
    pkg_project_description = get_package_share_directory('notsam')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the URDF file from "description" package
    # urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'robot.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    urdf_path  =  os.path.join(pkg_project_description, 'urdf', 'sam_bot_description.urdf')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-r empty.sdf"}.items(),
    )

    # Spawn robot
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1',
                ],
        output='screen',
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"}
        ]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering'
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'urdf_config.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'rviz', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        # DeclareLaunchArgument(name='model', default_value=default_model_path,
        #                                     description='Absolute path to robot urdf file'),
        # DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        # joint_state_publisher_gui_node,
        robot_state_publisher,
        # joint_state_publisher_node,
        # rviz_node,
        # rviz,
        # 
        bridge,
        gz_sim,
        TimerAction(
            period=1.0,
            actions=[create, rqt_robot_steering]),
            
    ])
