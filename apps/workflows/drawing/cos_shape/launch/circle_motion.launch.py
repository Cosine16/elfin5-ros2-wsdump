#!/usr/bin/env python3
# 启动 cos_shape 圆周运动节点（需先启动 elfin5 仿真：scripts/start_sim.sh moveit2）
#
# MoveGroupInterface 需要在本节点的参数服务器上找到 robot_description /
# robot_description_semantic，这里直接加载 elfin5 的 URDF(xacro) 和 SRDF。
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # robot_description：与 elfin5.launch.py 仿真使用同一份 xacro
    xacro_file = os.path.join(
        get_package_share_directory('elfin5_ros2_gazebo'),
        'urdf', 'elfin5.urdf.xacro')
    robot_description = {
        'robot_description': xacro.process_file(
            xacro_file,
            mappings={'use_fake_hardware': 'false', 'use_real_hardware': 'false'}
        ).toxml()
    }

    # robot_description_semantic：SRDF
    srdf_file = os.path.join(
        get_package_share_directory('elfin5_ros2_moveit2'),
        'config', 'elfin5.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # robot_description_kinematics：IK 求解插件。
    # 不加载则 MoveGroupInterface 报 "No kinematics plugins defined"，
    # computeCartesianPath 无法工作，enable 后机械臂不动。
    kinematics_file = os.path.join(
        get_package_share_directory('elfin5_ros2_moveit2'),
        'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        robot_description_kinematics = {'robot_description_kinematics': yaml.safe_load(f)}

    return LaunchDescription([
        Node(
            package='cos_shape',
            executable='circle_motion',
            name='circle_motion',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {
                    'group_name': 'elfin_arm',
                    'ee_link': 'elfin_end_link',
                    'center': [0.3, 0.0, 0.3],
                    'radius': 0.05,
                    'angular_velocity': 0.5,   # rad/s（speed_mode=angular 时生效）
                    'linear_velocity': 0.05,   # m/s （speed_mode=linear 时生效）
                    'speed_mode': 'angular',
                    'inclination': 0.0,        # rad，圆平面法线与 Z 轴夹角
                    'azimuth': 0.0,            # rad
                    'waypoints_per_rev': 64,
                    'revolutions': -1,         # -1 = 无限圈
                    'eef_step': 0.005,
                    'autostart': False,
                },
            ],
        ),
    ])
