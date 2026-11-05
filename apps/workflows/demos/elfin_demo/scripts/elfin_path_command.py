#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹运动接口示例（ROS2 移植自官方 ROS1 elfin_path_command.py）
发布 /elfin_arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
→ 直接由 joint_trajectory_controller 执行，不经过 MoveIt 规划/碰撞检查
⚠ 真机注意：从当前姿态直接插值到目标，路径不可预知，确认空间安全再运行
"""
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rclpy.init()
    node = Node('elfin_path_command')
    pub = node.create_publisher(
        JointTrajectory, '/elfin_arm_controller/joint_trajectory', 1)

    time.sleep(1.0)  # 等待 DDS 订阅关系建立

    msg = JointTrajectory()
    msg.joint_names = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3',
                       'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
    msg.header.stamp = node.get_clock().now().to_msg()

    point = JointTrajectoryPoint()
    point.positions = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
    point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    point.accelerations = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    point.time_from_start.sec = 3  # 官方示例为 0.5s，真机放宽到 3s 更平缓
    msg.points.append(point)

    pub.publish(msg)
    node.get_logger().info('已发布 /elfin_arm_controller/joint_trajectory: 全关节 0.3 rad')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
