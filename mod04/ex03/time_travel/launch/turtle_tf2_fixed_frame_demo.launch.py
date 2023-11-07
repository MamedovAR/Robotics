import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('time_travel'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )

    return LaunchDescription([
        DeclareLaunchArgument("delay", default_value="5", description="Delay"),
        demo_nodes,
        Node(
            package='time_travel',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_frame_tf2_broadcaster',
        ),
    ])