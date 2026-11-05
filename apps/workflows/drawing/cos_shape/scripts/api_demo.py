#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Elfin 机械臂 basic_api 教程级 demo（对应 document/API_description.md）。

演示内容：
  订阅  /enable_state、/fault_state     —— 读取机械臂使能/报错状态
  发布  /joint_goal                     —— 指定臂型（关节角）规划并执行
  发布  /cart_goal                      —— 指定末端位姿规划并执行
  发布  /cart_path_goal                 —— 直线依次经过多个空间点
  发布  /elfin_arm_controller/joint_trajectory —— 直接下发关节轨迹
  呼叫  /stop_teleop                    —— 停止机械臂运动

用法（需先启动仿真：~/cos_ws/scripts/start_sim.sh moveit2）：
  ros2 run cos_shape api_demo.py --mode joint
  ros2 run cos_shape api_demo.py --mode cart
  ros2 run cos_shape api_demo.py --mode cart_path
  ros2 run cos_shape api_demo.py --mode trajectory
  ros2 run cos_shape api_demo.py --mode stop
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Elfin 6 个关节的名字（顺序固定）
JOINT_NAMES = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3',
               'elfin_joint4', 'elfin_joint5', 'elfin_joint6']

# 参考坐标系（elfin_basic_api 的规划基坐标系）
REF_FRAME = 'elfin_base_link'


class ApiDemo(Node):
    def __init__(self):
        super().__init__('elfin_api_demo')

        # ---------- 订阅：机械臂状态 ----------
        self.enable_state = None
        self.fault_state = None
        self.create_subscription(Bool, '/enable_state', self._enable_cb, 10)
        self.create_subscription(Bool, '/fault_state', self._fault_cb, 10)

        # ---------- 发布：各类运动指令 ----------
        self.joint_goal_pub = self.create_publisher(JointState, '/joint_goal', 10)
        self.cart_goal_pub = self.create_publisher(PoseStamped, '/cart_goal', 10)
        self.cart_path_pub = self.create_publisher(PoseArray, '/cart_path_goal', 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/elfin_arm_controller/joint_trajectory', 10)

        # ---------- 服务：停止运动 ----------
        self.stop_cli = self.create_client(SetBool, '/stop_teleop')

    # ================= 状态回调 =================
    def _enable_cb(self, msg: Bool):
        self.enable_state = msg.data
        self.get_logger().info(f'使能状态: {msg.data}')

    def _fault_cb(self, msg: Bool):
        self.fault_state = msg.data
        self.get_logger().warn(f'报错状态: {msg.data}')

    def wait_ready(self, timeout=5.0):
        """spin 一段时间：等 topic 订阅关系建立 + 收到一次状态。"""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.enable_state is not None:
                break

    # ================= demo 1: /joint_goal =================
    def pub_joint_goal(self, positions):
        """令机械臂规划到达指定臂型的路径并执行。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.joint_goal_pub.publish(msg)
        self.get_logger().info(f'已发布 /joint_goal: {positions}')

    # ================= demo 2: /cart_goal =================
    def pub_cart_goal(self, position, orientation=(0.0, 1.0, 0.0, 0.0)):
        """令机械臂规划到达指定空间位姿的路径并执行。
        orientation 为四元数 (x, y, z, w)，默认法兰竖直朝下。"""
        msg = PoseStamped()
        msg.header.frame_id = REF_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = orientation
        self.cart_goal_pub.publish(msg)
        self.get_logger().info(f'已发布 /cart_goal: position={position}')

    # ================= demo 3: /cart_path_goal =================
    def pub_cart_path(self, points, orientation=(0.0, 1.0, 0.0, 0.0)):
        """以直线方式依次经过 points 中的各个空间点并执行。"""
        msg = PoseArray()
        msg.header.frame_id = REF_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = orientation
            msg.poses.append(pose)
        self.cart_path_pub.publish(msg)
        self.get_logger().info(f'已发布 /cart_path_goal: {len(points)} 个点')

    # ================= demo 4: 直接下发关节轨迹 =================
    def pub_trajectory(self, positions, duration_sec=3):
        """直接向控制器发一条单点轨迹（不经过 MoveIt 规划）。"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = int(duration_sec)
        msg.points.append(point)
        self.trajectory_pub.publish(msg)
        self.get_logger().info(
            f'已发布 /elfin_arm_controller/joint_trajectory: {positions}')

    # ================= demo 5: 呼叫 /stop_teleop =================
    def call_stop(self):
        """呼叫服务令机械臂停止运动。"""
        if not self.stop_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/stop_teleop 服务不可用')
            return
        req = SetBool.Request()
        req.data = True
        future = self.stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info(f'stop_teleop 返回: {future.result().message}')
        else:
            self.get_logger().error('stop_teleop 呼叫超时')


def main():
    parser = argparse.ArgumentParser(description='Elfin basic_api 教程 demo')
    parser.add_argument('--mode', required=True,
                        choices=['joint', 'cart', 'cart_path', 'trajectory', 'stop'],
                        help='要演示的 API')
    args = parser.parse_args()

    rclpy.init()
    node = ApiDemo()
    node.wait_ready()

    if args.mode == 'joint':
        # 一个法兰大致朝下的臂型（单位 rad）
        node.pub_joint_goal([0.0, -0.2, 1.6, 0.0, 1.4, 0.0])
        time.sleep(1.0)  # 给话题一点送达时间

    elif args.mode == 'cart':
        node.pub_cart_goal(position=(0.42, 0.0, 0.445))
        time.sleep(1.0)

    elif args.mode == 'cart_path':
        # 一条小三角形折线
        node.pub_cart_path([(0.40, 0.00, 0.40),
                            (0.45, 0.05, 0.45),
                            (0.45, -0.05, 0.45),
                            (0.40, 0.00, 0.40)])
        time.sleep(1.0)

    elif args.mode == 'trajectory':
        node.pub_trajectory([0.1, 0.1, 0.1, 0.1, 0.1, 0.1], duration_sec=3)
        time.sleep(1.0)

    elif args.mode == 'stop':
        node.call_stop()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
