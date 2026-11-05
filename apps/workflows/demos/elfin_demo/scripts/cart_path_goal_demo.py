#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空间位置组运动接口示例（ROS2 移植自官方 ROS1 cart_path_goal_demo.py）
发布 /cart_path_goal (geometry_msgs/PoseArray)
→ elfin_basic_api 以直线方式依次经过各点并执行
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray


def make_pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    # 四元数 (0,1,0,0)：法兰竖直朝下
    p.orientation.x = 0.0
    p.orientation.y = 1.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p


def main():
    rclpy.init()
    node = Node('cart_path_goal_demo')
    pub = node.create_publisher(PoseArray, '/cart_path_goal', 1)

    time.sleep(1.0)  # 等待 DDS 订阅关系建立

    msg = PoseArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'elfin_base_link'
    msg.poses.append(make_pose(0.420, 0.0, 0.445))
    msg.poses.append(make_pose(0.420, 0.1, 0.445))
    pub.publish(msg)
    node.get_logger().info('已发布 /cart_path_goal: 2 个点（向 +Y 平移 10cm）')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
