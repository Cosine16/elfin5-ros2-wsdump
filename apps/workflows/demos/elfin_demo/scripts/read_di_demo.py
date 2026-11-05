#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取机器人 DI 接口示例（ROS2 移植自官方 ROS1 read_di_demo.py）
呼叫 /read_di (elfin_robot_msgs/ElfinIODRead)
⚠ 仅真机有效（IO 从站由真实驱动提供）
"""
import rclpy
from rclpy.node import Node
from elfin_robot_msgs.srv import ElfinIODRead


def main():
    rclpy.init()
    node = Node('read_di_demo')
    client = node.create_client(ElfinIODRead, '/read_di')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/read_di 服务不可用（真机驱动未启动？）')
        node.destroy_node()
        rclpy.shutdown()
        return

    req = ElfinIODRead.Request()
    req.data = True
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is not None:
        node.get_logger().info(f'Robot DI: {future.result().digital_input}')
    else:
        node.get_logger().error('服务呼叫超时')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
