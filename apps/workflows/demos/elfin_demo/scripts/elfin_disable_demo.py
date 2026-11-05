#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人去使能接口示例（ROS2 移植自官方 ROS1 elfin_disable_demo.py）
呼叫 /elfin_basic_api/disable_robot (std_srvs/SetBool)
⚠ 仅真机有效；真机断电前请先执行本脚本（等价于面板的 Servo Off）
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = Node('elfin_disable_demo')
    client = node.create_client(SetBool, '/elfin_basic_api/disable_robot')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/elfin_basic_api/disable_robot 服务不可用（basic_api 未启动？）')
        node.destroy_node()
        rclpy.shutdown()
        return

    req = SetBool.Request()
    req.data = True
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is not None:
        node.get_logger().info(
            f'success={future.result().success}, message="{future.result().message}"')
    else:
        node.get_logger().error('服务呼叫超时')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
