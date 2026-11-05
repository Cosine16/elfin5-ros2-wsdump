#!/usr/bin/python3

# rs_camera.launch.py:
# 实相机(RealSense D435i)验证用: 启动 realsense2_camera + rviz。
#
# 前置: sudo apt install ros-foxy-realsense2-camera
# 本文件只是 include rs_launch.py, 未安装时仅在运行时报错, 不影响 cos_rsvisual 编译。
#
# 注意: foxy 的 realsense2_camera 参数名随版本不同:
#   - 3.2.3 (foxy 默认源): depth_width / depth_height / depth_fps / enable_color /
#     enable_pointcloud
#   - 4.51+ (ros2 分支): depth_module.profile / enable_color / pointcloud.enable
# 下面默认按 4.51+ 写法, 若用 3.2.3 请改用注释中的旧参数名。
#
# 真机话题对齐:
#   深度图(16UC1): /camera/depth/image_rect_raw
#   点云:          /camera/depth/color/points
# 接真机做检测时, 把 config/follower.yaml 的 depth_topic / camera_info_topic 对准,
# 把 config/sensors_3d.yaml 的 point_cloud_topic 对准即可。

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Launch rviz2 with the cos_rsvisual config')

    # 实相机不走仿真时钟
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # ---- realsense2_camera ----
    # 4.51+ (ros2 分支) 参数名:
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            # 4.51+: 深度 640x480@15, 关彩色, 开点云
            'depth_module.profile': '640x480x15',
            'enable_color': 'false',
            'pointcloud.enable': 'true',
            # 3.2.3 (foxy 默认源) 旧参数名(如用旧版驱动, 换成下面这些):
            # 'depth_width': '640',
            # 'depth_height': '480',
            # 'depth_fps': '15',
            # 'enable_color': 'false',
            # 'enable_pointcloud': 'true',
        }.items(),
    )

    # ---- rviz ----
    rviz_config = os.path.join(
        get_package_share_directory('cos_rsvisual'), 'rviz', 'elfin5_rs.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_rviz_cmd,
        rs_launch,
        rviz_node,
    ])
