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
            get_package_share_directory('learning_tf2_cpp'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        launch_arguments={'target_frame': 'carrot1'}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument("radius", default_value="5.0", description="Radius of carrot's rotation"),
        DeclareLaunchArgument("direction_of_rotation", default_value="-1", description="Rotation direction (1 - clockwise, -1 - counterclockwise)"),
        demo_nodes,
        Node(
            package='learning_tf2_cpp',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
    ])