#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取机器人关节状态示例（ROS2 移植自官方 ROS1 get_robot_state.py）
订阅 /elfin_arm_controller/state (control_msgs/JointTrajectoryControllerState)
Ctrl+C 退出
"""
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState


class GetRobotStateDemo(Node):
    def __init__(self):
        super().__init__('get_robot_state')
        self.create_subscription(
            JointTrajectoryControllerState,
            '/elfin_arm_controller/state', self.state_cb, 10)

    def state_cb(self, msg):
        pos = msg.actual.positions
        vel = msg.actual.velocities
        self.get_logger().info(
            'joint position: ' + ', '.join(f'j{i+1}: {p:.4f}' for i, p in enumerate(pos)))
        self.get_logger().info(
            'joint velocity: ' + ', '.join(f'j{i+1}: {v:.4f}' for i, v in enumerate(vel)))


def main():
    rclpy.init()
    node = GetRobotStateDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
