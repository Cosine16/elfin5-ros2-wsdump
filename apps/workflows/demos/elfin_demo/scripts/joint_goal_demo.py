#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节角度运动接口示例（ROS2 移植自官方 ROS1 joint_goal_demo.py）
发布 /joint_goal (sensor_msgs/JointState)
→ elfin_basic_api 规划到达指定臂型的路径并执行
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node('joint_goal_demo')
    pub = node.create_publisher(JointState, '/joint_goal', 1)

    time.sleep(1.0)  # 等待 DDS 订阅关系建立

    msg = JointState()
    msg.name = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3',
                'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
    msg.position = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    node.get_logger().info('已发布 /joint_goal: 全关节 1.57 rad')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
