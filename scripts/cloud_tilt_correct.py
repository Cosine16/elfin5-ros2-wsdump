#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# cloud_tilt_correct.py —— 用实时点云里的桌面平面法向修正手眼标定的残余倾斜。
#
# 动机: 棋盘格 PnP 在浅视角下有旋转偏差(平面目标歧义性), AX=XB 的旋转解不准;
# 而"桌面在 base 系必须水平"是强约束。直接在光学系拟合桌面法向,
# 对 R(end->camera) 施加精确修正(绕垂直于 z 的轴, 不动 yaw, 不动平移)。
#
# 用法: python3 cloud_tilt_correct.py [yaml路径, 默认最新非修正版]
# 前置: 机械臂静止; 相机与机械臂栈运行中; 旧标定 TF 已发布(测量依赖当前链)。

import glob
import os
import sys
import time

import numpy as np
import yaml
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

sys.path.insert(0, '/home/fit/elfin5_ctl/cos_ws/app_ws/install/cos_realsense/lib/python3.10/site-packages')
from cos_realsense.hand_eye_calib_node import make_T, quat_to_rot, rot_to_rpy

R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
Z = np.array([0.0, 0.0, 1.0])


def fit_plane_normal(pts):
    rng = np.random.default_rng(0)
    if len(pts) > 20000:
        pts = pts[rng.choice(len(pts), 20000, replace=False)]
    best = (None, -1)
    for _ in range(400):
        i = rng.choice(len(pts), 3, replace=False)
        p1, p2, p3 = pts[i]
        nv = np.cross(p2 - p1, p3 - p1)
        ln = np.linalg.norm(nv)
        if ln < 1e-9:
            continue
        nv /= ln
        nin = int((np.abs((pts - p1) @ nv) < 0.006).sum())
        if nin > best[1]:
            best = ((nv, p1), nin)
    (nv, p1), _ = best
    inl = pts[np.abs((pts - p1) @ nv) < 0.006]
    _, _, vt = np.linalg.svd(inl - inl.mean(0), full_matrices=False)
    return vt[-1], inl.mean(0), len(inl)


def main():
    files = [f for f in sorted(glob.glob(
        os.path.expanduser('~/.ros/hand_eye_calib/hand_eye_*.yaml')))
        if 'corr' not in f]
    path = sys.argv[1] if len(sys.argv) > 1 else files[-1]
    with open(path) as f:
        data = yaml.safe_load(f)
    X = np.array(data['T_end_camera_link_4x4'], dtype=float)
    print(f'输入: {os.path.basename(path)}')

    rclpy.init()
    n = Node('cloud_tilt_correct')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    cloud = []
    n.create_subscription(PointCloud2, '/camera/camera/depth/color/points',
                          lambda m: cloud.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while not cloud and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not cloud:
        print('无点云')
        sys.exit(1)
    msg = cloud[0]
    pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
    # 光学系内剔除近处杂点(机械臂本体), 保留 0.3~1.2m 桌面带
    r = np.linalg.norm(pts, axis=1)
    pts = pts[(r > 0.3) & (r < 1.2)]
    n_opt, center, nin = fit_plane_normal(pts)
    rms = float(np.sqrt((((np.linalg.norm(pts, axis=1) * 0)) ).mean()))  # placeholder
    print(f'光学系平面: 法向 {np.round(n_opt, 4).tolist()}, 内点 {nin}')

    T_be = np.eye(4)
    tf = buf.lookup_transform('elfin_base_link', 'elfin_end_link',
                              rclpy.time.Time(), timeout=Duration(seconds=2.0))
    t, q = tf.transform.translation, tf.transform.rotation
    T_be = make_T(quat_to_rot(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
    R_be = T_be[:3, :3]

    # 交叉验证: 用当前标定链预测 base 系法向, 应与直接测量一致
    n_meas = R_be @ X[:3, :3] @ R_LINK_OPT @ n_opt
    if n_meas[2] < 0:
        n_meas, n_opt = -n_meas, -n_opt
    tilt = np.degrees(np.arccos(np.clip(n_meas[2], -1, 1)))
    print(f'当前标定链下桌面倾斜 {tilt:.2f} deg (法向 {np.round(n_meas, 4).tolist()})')

    # 精确修正: w = axis(n_meas->z)*angle 在 base 系; delta = R_be^T w (end 系)
    a = np.cross(n_meas, Z)
    la = np.linalg.norm(a)
    if la < 1e-6:
        print('桌面已经水平, 无需修正')
        sys.exit(0)
    a /= la
    w = a * np.arccos(np.clip(n_meas @ Z, -1, 1))
    delta = R_be.T @ w
    R_ec_new = cv2.Rodrigues(delta)[0] @ X[:3, :3]

    n_check = R_be @ R_ec_new @ R_LINK_OPT @ n_opt
    if n_check[2] < 0:
        n_check = -n_check
    print(f'修正后预测倾斜 {np.degrees(np.arccos(np.clip(n_check[2], -1, 1))):.3f} deg')

    X2 = X.copy()
    X2[:3, :3] = R_ec_new
    xyz = X2[:3, 3]
    rr, pp, yy = rot_to_rpy(R_ec_new)
    out = dict(data)
    out['T_end_camera_link_4x4'] = X2.tolist()
    out['mount_xyz'] = [float(v) for v in xyz]
    out['mount_rpy'] = [float(rr), float(pp), float(yy)]
    out['cloud_tilt_correction'] = {
        'tilt_before_deg': float(tilt),
        'measured_normal_base': [float(v) for v in n_meas],
    }
    out_path = path.replace('.yaml', '_cloudcorr.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f, allow_unicode=True)
    print(f'\n修正结果:')
    print(f'  mount_xyz: "{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}"')
    print(f'  mount_rpy: "{rr:.6f} {pp:.6f} {yy:.6f}"')
    print(f'已保存: {out_path}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
