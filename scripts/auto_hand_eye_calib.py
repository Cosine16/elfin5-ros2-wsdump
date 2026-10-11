#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# auto_hand_eye_calib.py —— 自动完成眼在手上标定采样:
#   jitter 模式: 绕当前位姿小幅度扰动关节(/joint_goal), 适合首轮粗标;
#   lookat 模式: 用已有标定结果当先验估算棋盘格在 base 系的位置,
#     生成绕板子半球凝视位姿(方位角/俯角/滚转多样, 姿态差异 30~50 度),
#     走 /cart_goal 笛卡尔目标, 大幅提高 AX=XB 方程组条件数。
#   每个位姿通过 /hand_eye_calib/sample 服务让标定节点检测棋盘格并采样,
#   采够后调用 /hand_eye_calib/solve 输出 mount_xyz/mount_rpy。
#
# 前置: start_real.py 已启动且面板 Servo On; realsense2_camera 与
#       hand_eye_calib_node 已启动(均须在 ROS_LOCALHOST_ONLY=1 域内)。
#
# 用法:
#   python3 auto_hand_eye_calib.py --mode jitter           # 首轮粗标
#   python3 auto_hand_eye_calib.py --mode lookat           # 精标(需已有 yaml 先验)
#   python3 auto_hand_eye_calib.py --dry-run               # 只检测当前位姿可见性

import argparse
import glob
import math
import os
import sys
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import tf2_ros

from cos_realsense.hand_eye_calib_node import (
    detect_board, make_T, quat_to_rot, rot_to_quat)

JOINT_NAMES = [f'elfin_joint{i}' for i in range(1, 7)]

# jitter 模式候选: 0.5m 视距下手腕转 0.2rad 视野中心偏移约 10cm, 小于板宽一半
JITTER_DELTAS = [
    {3: +0.20}, {3: -0.20}, {4: +0.20}, {4: -0.20}, {5: +0.20}, {5: -0.20},
    {0: +0.20}, {0: -0.20},
    {0: +0.20, 4: +0.15}, {0: -0.20, 4: -0.15},
    {3: +0.20, 5: -0.15}, {3: -0.20, 5: +0.15},
    {1: +0.12, 3: +0.15}, {2: -0.12, 3: -0.15},
    {4: +0.15, 5: +0.15}, {4: -0.15, 5: -0.15},
]

# 理想 optical->link 旋转(link: x前y左z上; optical: z前x右y下)
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def load_latest_prior():
    files = sorted(glob.glob(os.path.expanduser('~/.ros/hand_eye_calib/hand_eye_*.yaml')))
    if not files:
        return None
    with open(files[-1]) as f:
        data = yaml.safe_load(f)
    return np.array(data['T_end_camera_link_4x4'], dtype=float), files[-1]


class AutoCalibDriver(Node):
    def __init__(self, args):
        super().__init__('auto_hand_eye_driver')
        self.args = args
        self.joint_pos = {}
        self.enable = None
        self.fault = None
        self.latest_img = None
        self.K = None
        self.D = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(Bool, '/enable_state',
                                 lambda m: setattr(self, 'enable', m.data), 10)
        self.create_subscription(Bool, '/fault_state',
                                 lambda m: setattr(self, 'fault', m.data), 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                                 self._on_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self._on_info, 10)
        self.goal_pub = self.create_publisher(JointState, '/joint_goal', 1)
        self.cart_pub = self.create_publisher(PoseStamped, '/cart_goal', 1)
        self.vel_pub = self.create_publisher(
            Float32, '/elfin_basic_api/vel', qos_profile_sensor_data)
        self.sample_cli = self.create_client(Trigger, '/hand_eye_calib/sample')
        self.solve_cli = self.create_client(Trigger, '/hand_eye_calib/solve')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    def _on_img(self, msg):
        from cv_bridge import CvBridge
        if not hasattr(self, '_bridge'):
            self._bridge = CvBridge()
        try:
            self.latest_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)
            self.D = np.array(msg.d, dtype=float)

    def _wait(self, cond, timeout, desc):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if cond():
                return True
        self.get_logger().error(f'等待超时: {desc}')
        return False

    def call_trigger(self, cli, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, '服务不可用(标定节点在跑吗?)'
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done():
            return False, '服务调用超时'
        res = fut.result()
        return res.success, res.message

    def set_velocity(self, vel):
        msg = Float32()
        msg.data = vel
        for _ in range(5):
            self.vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)

    def current_q(self):
        return [self.joint_pos[n] for n in JOINT_NAMES]

    def lookup_T(self, target, source):
        from rclpy.duration import Duration
        tf = self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(), timeout=Duration(seconds=1.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        return make_T(quat_to_rot(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

    def wait_settled(self, timeout=60.0):
        """等关节停稳; 若 4s 内完全没动过判为规划失败。返回 (到位?, 说明)"""
        q_start = self.current_q()
        t0 = time.time()
        moved = False
        last_q, last_t = None, 0.0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.fault:
                return False, '机械臂报 fault, 中止'
            q = self.current_q()
            now = time.time()
            if max(abs(a - b) for a, b in zip(q, q_start)) > 0.01:
                moved = True
            if not moved and now - t0 > 4.0:
                return False, '关节未动, 规划失败(目标不可达/限位)'
            if moved and last_q is not None and now - last_t >= 0.8:
                if max(abs(a - b) for a, b in zip(q, last_q)) < 0.004:
                    return True, ''
                last_q, last_t = q, now
            elif moved and last_q is None:
                last_q, last_t = q, now
        return False, '运动超时'

    def move_joints(self, goal):
        msg = JointState()
        msg.name = JOINT_NAMES
        msg.position = list(goal)
        self.goal_pub.publish(msg)
        ok, msg_txt = self.wait_settled()
        if ok:
            err = max(abs(c - g) for c, g in zip(self.current_q(), goal))
            if err > 0.05:
                return False, f'到位残差过大 {err:.3f} rad'
        return ok, msg_txt

    def move_cart(self, T_base_end):
        msg = PoseStamped()
        msg.header.frame_id = 'elfin_base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = T_base_end[:3, 3]
        q = rot_to_quat(T_base_end[:3, :3])
        msg.pose.orientation.x, msg.pose.orientation.y, \
            msg.pose.orientation.z, msg.pose.orientation.w = q
        self.cart_pub.publish(msg)
        return self.wait_settled()

    def board_point_in_base(self, T_end_optical):
        """当前位姿检测棋盘格, 反推板心在 base 系位置。失败返回 None。"""
        if self.latest_img is None or self.K is None:
            return None
        pattern = (8, 8)
        objp = np.zeros((64, 3), np.float32)
        objp[:, :2] = np.mgrid[0:8, 0:8].T.reshape(-1, 2) * 0.018
        det = detect_board(self.latest_img, pattern, objp, self.K, self.D)
        if det is None:
            return None
        R_cb, t_cb, _ = det
        T_base_end = self.lookup_T('elfin_base_link', 'elfin_end_link')
        return (T_base_end @ T_end_optical @ make_T(R_cb, t_cb))[:3, 3]

    def lookat_poses(self, p_board, T_end_optical, radius):
        """绕板心的半球凝视位姿(elfin_end_link 目标, base 系)。"""
        inv_T_end_opt = np.linalg.inv(T_end_optical)
        dirs = [(0.0, 0.0, 1.0)]
        for phi in (0, 90, 180, 270):
            t = math.radians(30)
            p = math.radians(phi)
            dirs.append((math.sin(t) * math.cos(p), math.sin(t) * math.sin(p), math.cos(t)))
        for phi in (45, 135, 225, 315):
            t = math.radians(50)
            p = math.radians(phi)
            dirs.append((math.sin(t) * math.cos(p), math.sin(t) * math.sin(p), math.cos(t)))
        poses = []
        for i, d in enumerate(dirs):
            d = np.array(d)
            p_o = p_board + radius * d
            z_o = -d                                   # 光轴指向板心
            x_tmp = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
            y_o = np.cross(z_o, x_tmp)
            y_o /= np.linalg.norm(y_o)
            x_o = np.cross(y_o, z_o)
            R = np.column_stack([x_o, y_o, z_o])
            roll = math.radians([0, 25, -25][i % 3])   # 附加滚转多样性
            R = R @ make_T(cv2_rodrigues(z_o * roll), [0, 0, 0])[:3, :3] if roll else R
            poses.append(make_T(R, p_o) @ inv_T_end_opt)
        return poses

    def run(self):
        log = self.get_logger().info
        if not self._wait(lambda: len(self.joint_pos) == 6, 15, '等待 /joint_states'):
            return 1
        if not self._wait(lambda: self.enable is not None, 15, '等待 /enable_state'):
            return 1
        if not self.enable:
            log('机械臂未使能: 请在面板 Clear Fault -> Servo On 后重跑')
            return 1
        if self.fault:
            log('机械臂处于 fault 状态, 请先清错')
            return 1
        if not self._wait(lambda: self.latest_img is not None and self.K is not None,
                          20, '等待相机图像/内参'):
            return 1

        ok, msg = self.call_trigger(self.sample_cli)
        log(f'当前位姿检测: {msg}')
        if '未检测到' in msg or 'TF' in msg:
            log('当前位姿看不到棋盘格。请用面板把臂摆到"相机正对棋盘格"的位置后重跑。')
            return 1
        n_samples = 1 if ok else 0
        if self.args.dry_run:
            log('--dry-run: 检测正常, 不执行任何运动。')
            return 0

        self.set_velocity(self.args.vel)
        q0 = self.current_q()
        log(f'速度缩放 {self.args.vel:.0%}, 起始位姿: {[round(v, 3) for v in q0]}')

        if self.args.mode == 'lookat':
            prior = load_latest_prior()
            if prior is None:
                log('lookat 模式需要已有标定 yaml 当先验, 未找到; 请先用 jitter 模式粗标')
                return 1
            X_prior, prior_file = prior
            log(f'先验: {os.path.basename(prior_file)}')
            T_end_optical = X_prior @ make_T(R_LINK_OPT, [0, 0, 0])
            p_board = self.board_point_in_base(T_end_optical)
            if p_board is None:
                log('无法在 base 系定位棋盘格(当前帧检测失败), 请调整位姿后重跑')
                return 1
            log(f'板心(base 系): {np.round(p_board, 3).tolist()}')
            radius = float(np.clip(
                np.linalg.norm(self.lookup_T('elfin_base_link', 'elfin_end_link')[:3, 3]
                               - p_board), 0.35, 0.60))
            pose_list = self.lookat_poses(p_board, T_end_optical, radius)
            log(f'lookat 模式: {len(pose_list)} 个凝视位姿, 半径 {radius:.2f} m')
            mover = self.move_cart
            goals = pose_list
        else:
            goals = [[q0[j] + d.get(j, 0.0) for j in range(6)] for d in JITTER_DELTAS]
            mover = self.move_joints

        for i, goal in enumerate(goals):
            if n_samples >= self.args.samples:
                break
            ok, msg = mover(goal)
            if not ok:
                log(f'候选 #{i+1} 跳过: {msg}')
                if 'fault' in msg:
                    return 1
                self.move_joints(q0)
                continue
            time.sleep(1.2)   # 等机械静止, 避免运动模糊/TF 滞后
            ok, msg = self.call_trigger(self.sample_cli)
            log(f'候选 #{i+1} 采样: {msg}')
            if ok:
                n_samples += 1
            self.move_joints(q0)   # 回起始位姿, 保证下一候选从同一基准出发

        log('回到起始位姿...')
        self.move_joints(q0)

        log(f'采样结束, 本轮新增 {n_samples} 组, 求解中...')
        ok, report = self.call_trigger(self.solve_cli, timeout=20.0)
        print('\n' + report)
        return 0 if ok else 1


def cv2_rodrigues(vec):
    import cv2
    return cv2.Rodrigues(np.asarray(vec, dtype=float))[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['jitter', 'lookat'], default='lookat')
    ap.add_argument('--dry-run', action='store_true', help='只检测当前位姿可见性, 不动臂')
    ap.add_argument('--samples', type=int, default=12)
    ap.add_argument('--vel', type=float, default=0.15, help='速度缩放 0~1')
    args = ap.parse_args()
    rclpy.init()
    driver = AutoCalibDriver(args)
    try:
        sys.exit(driver.run())
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
