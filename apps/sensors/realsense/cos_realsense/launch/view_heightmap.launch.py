"""Load a generated heightmap world in Gazebo.

Run after capture_heightmap.launch.py has produced a world file:
  ros2 launch cos_realsense view_heightmap.launch.py
  ros2 launch cos_realsense view_heightmap.launch.py world:=/path/to/heightmap.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration('world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py']),
        launch_arguments={'world': world}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(os.environ.get('ELFIN5_OUTPUT_DIR', 'outputs'), 'realsense', 'heightmap.world')),
        gazebo,
    ])
