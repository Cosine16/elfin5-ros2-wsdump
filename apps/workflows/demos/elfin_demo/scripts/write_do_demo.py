#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设置机器人 DO 接口示例（ROS2 移植自官方 ROS1 write_do_demo.py）
呼叫 /write_do (elfin_robot_msgs/ElfinIODWrite)
⚠ 仅真机有效；8192(0x2000) 为官方示例值，请按现场接线确认要置位的 DO 通道
"""
import rclpy
from rclpy.node import Node
from elfin_robot_msgs.srv import ElfinIODWrite


def main():
    rclpy.init()
    node = Node('write_do_demo')
    client = node.create_client(ElfinIODWrite, '/write_do')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/write_do 服务不可用（真机驱动未启动？）')
        node.destroy_node()
        rclpy.shutdown()
        return

    req = ElfinIODWrite.Request()
    req.digital_output = 8192  # 0x2000，官方示例值
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is not None and future.result().success:
        node.get_logger().info('Write DO Success')
    else:
        node.get_logger().error('Write DO 失败或超时')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
