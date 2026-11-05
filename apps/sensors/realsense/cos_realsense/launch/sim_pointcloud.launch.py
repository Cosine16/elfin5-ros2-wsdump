"""Simulated D435i: Gazebo world + xacro model + depth camera plugin + RViz.

Run:
  ros2 launch cos_realsense sim_pointcloud.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('cos_realsense')
    xacro_file = os.path.join(pkg_share, 'urdf', 'd435i.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'demo.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'pointcloud.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'd435i',
                   '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen')

    stats_node = Node(
        package='cos_realsense',
        executable='pointcloud_stats_node',
        output='screen')

    # Reproject the depth image: the gazebo plugin's own point cloud is
    # broken in ros-foxy-gazebo-plugins 3.5.3 (see depth_projector_node.py).
    projector_node = Node(
        package='cos_realsense',
        executable='depth_projector_node',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_camera,
        stats_node,
        projector_node,
        rviz,
    ])
