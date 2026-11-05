"""Real D435i: realsense2_camera driver with point cloud + RViz.

Run:
  ros2 launch cos_realsense real_pointcloud.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cos_realsense')
    rviz_config = os.path.join(pkg_share, 'rviz', 'pointcloud.rviz')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py']),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            # Uncomment to enable the IMU:
            # 'enable_gyro': 'true',
            # 'enable_accel': 'true',
            # 'unite_imu_method': '2',
        }.items())

    stats_node = Node(
        package='cos_realsense',
        executable='pointcloud_stats_node',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([realsense, stats_node, rviz])
