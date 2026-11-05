#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取机器人使能状态示例（ROS2 移植自官方 ROS1 get_enable_state.py）
订阅 /enable_state (std_msgs/Bool)
⚠ 仅真机有发布者（ros2_control_node 内的驱动节点），仿真中订阅不到
Ctrl+C 退出
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GetEnableStateDemo(Node):
    def __init__(self):
        super().__init__('get_enable_state')
        self.create_subscription(Bool, '/enable_state', self.enable_cb, 10)

    def enable_cb(self, msg):
        self.get_logger().info('robot enable' if msg.data else 'robot disable')


def main():
    rclpy.init()
    node = GetEnableStateDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
