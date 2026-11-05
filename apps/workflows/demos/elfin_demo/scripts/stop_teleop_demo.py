#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
停止运动服务示例（ROS2 移植自官方 ROS1 stop_teleop_demo.py）
呼叫 /stop_teleop (std_srvs/SetBool)
→ basic_api 向控制器发空轨迹抢占当前轨迹，机械臂停在当前位置
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = Node('stop_teleop_demo')
    client = node.create_client(SetBool, '/stop_teleop')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/stop_teleop 服务不可用（basic_api 未启动？）')
        node.destroy_node()
        rclpy.shutdown()
        return

    req = SetBool.Request()
    req.data = True
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is not None:
        node.get_logger().info(
            f'success={future.result().success}, message="{future.result().message}"')
    else:
        node.get_logger().error('服务呼叫超时')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
