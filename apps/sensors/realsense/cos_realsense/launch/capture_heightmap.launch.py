"""Capture a point cloud stream and bake it into a Gazebo heightmap.

Real camera:
  ros2 launch cos_realsense capture_heightmap.launch.py

Simulated camera (end-to-end self test without hardware):
  ros2 launch cos_realsense capture_heightmap.launch.py camera_source:=sim z_min:=-0.6 z_max:=0.6

Common parameters (forwarded to heightmap_builder_node):
  num_frames, resolution, x_min/x_max, y_min/y_max, z_min/z_max, output_dir
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cos_realsense')

    camera_source = LaunchConfiguration('camera_source')

    declared = [
        DeclareLaunchArgument('camera_source', default_value='real',
                              description='real | sim'),
        DeclareLaunchArgument('num_frames', default_value='100'),
        DeclareLaunchArgument('resolution', default_value='0.01'),
        DeclareLaunchArgument('x_min', default_value='-0.2'),
        DeclareLaunchArgument('x_max', default_value='3.0'),
        DeclareLaunchArgument('y_min', default_value='-1.6'),
        DeclareLaunchArgument('y_max', default_value='1.6'),
        DeclareLaunchArgument('z_min', default_value='-0.6'),
        DeclareLaunchArgument('z_max', default_value='0.3'),
        DeclareLaunchArgument('output_dir',
                              default_value=os.path.join(os.environ.get('ELFIN5_OUTPUT_DIR', 'outputs'), 'realsense')),
    ]

    real_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py']),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", camera_source, "' == 'real'"])))

    sim_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim_pointcloud.launch.py')),
        condition=IfCondition(PythonExpression(["'", camera_source, "' == 'sim'"])))

    builder = Node(
        package='cos_realsense',
        executable='heightmap_builder_node',
        parameters=[{
            'num_frames': LaunchConfiguration('num_frames'),
            'resolution': LaunchConfiguration('resolution'),
            'x_min': LaunchConfiguration('x_min'),
            'x_max': LaunchConfiguration('x_max'),
            'y_min': LaunchConfiguration('y_min'),
            'y_max': LaunchConfiguration('y_max'),
            'z_min': LaunchConfiguration('z_min'),
            'z_max': LaunchConfiguration('z_max'),
            'output_dir': LaunchConfiguration('output_dir'),
        }],
        output='screen')

    return LaunchDescription(declared + [real_camera, sim_camera, builder])
