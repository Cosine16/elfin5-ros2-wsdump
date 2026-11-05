#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空间位置运动接口示例（ROS2 移植自官方 ROS1 cart_goal_demo.py）
发布 /cart_goal (geometry_msgs/PoseStamped)
→ elfin_basic_api 规划到达指定末端位姿的路径并执行
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = Node('cart_goal_demo')
    pub = node.create_publisher(PoseStamped, '/cart_goal', 1)

    time.sleep(1.0)  # 等待 DDS 订阅关系建立

    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'elfin_base_link'
    msg.pose.position.x = 0.420
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.445
    # 四元数 (0,1,0,0)：法兰竖直朝下
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 1.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0
    pub.publish(msg)
    node.get_logger().info('已发布 /cart_goal: (0.42, 0.0, 0.445) 法兰朝下')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
