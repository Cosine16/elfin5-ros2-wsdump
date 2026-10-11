#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# refine_hand_eye_tilt.py —— 用"桌面水平"约束修正手眼标定的残余旋转误差。
#
# 原理: AX=XB 小姿态样本下旋转可观性差, 会有几度系统性倾斜; 而棋盘格平放在
# 水平桌面上, 其法向量在 base 系必为 (0,0,1)。用已有标定 yaml 里的原始样本
# 反推每组板法向, 对 R(end->camera) 做最小二乘微修正使法向全部对齐 +z。
# 仅修正 2 个旋转自由度(法向约束), 平移不变。
#
# 用法: python3 refine_hand_eye_tilt.py [yaml路径, 默认取最新]

import glob
import os
import sys

import numpy as np
import yaml
import cv2

Z = np.array([0.0, 0.0, 1.0])
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rot_to_rpy(R):
    p = np.arcsin(np.clip(-R[2, 0], -1, 1))
    r = np.arctan2(R[2, 1], R[2, 2])
    y = np.arctan2(R[1, 0], R[0, 0])
    return r, p, y


def board_normals(samples, R_ec):
    out = []
    for s in samples:
        T_be = np.array(s['T_base_end'])
        T_cb = np.array(s['T_cam_board'])
        n_cam = T_cb[:3, :3] @ Z
        out.append(T_be[:3, :3] @ R_ec @ R_LINK_OPT @ n_cam)
    ns = np.array(out)
    if ns.mean(0)[2] < 0:      # objp 约定下板法向可能朝下, 统一翻到 +z
        ns = -ns
    return ns


def main():
    files = sorted(glob.glob(os.path.expanduser('~/.ros/hand_eye_calib/hand_eye_*.yaml')))
    path = sys.argv[1] if len(sys.argv) > 1 else files[-1]
    with open(path) as f:
        data = yaml.safe_load(f)
    samples = data['samples_raw']
    X = np.array(data['T_end_camera_link_4x4'], dtype=float)
    print(f'输入: {os.path.basename(path)}  样本 {len(samples)} 组')

    ns = board_normals(samples, X[:3, :3])
    print(f'修正前: 板法向(base) 均值 {np.round(ns.mean(0), 4).tolist()}, '
          f'倾斜 {np.degrees(np.arccos(np.clip(ns.mean(0)[2], -1, 1))):.2f} deg, '
          f'离散 {np.degrees(np.arccos(np.clip((ns @ ns.mean(0) / np.linalg.norm(ns.mean(0))), -1, 1))).max():.2f} deg')

    R_ec = X[:3, :3].copy()
    for it in range(4):
        A, b = [], []
        for s in samples:
            T_be = np.array(s['T_base_end'])
            T_cb = np.array(s['T_cam_board'])
            v = R_ec @ R_LINK_OPT @ (T_cb[:3, :3] @ Z)   # 板法向(end系)
            n_i = T_be[:3, :3] @ v
            if n_i[2] < 0:                               # 与翻转约定一致
                n_i, v = -n_i, -v
            A.append(-T_be[:3, :3] @ _skew(v))           # n' ≈ n + A·delta
            b.append(Z - n_i)
        delta, *_ = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)
        R_ec = cv2.Rodrigues(delta)[0] @ R_ec
        ns = board_normals(samples, R_ec)
        tilt = np.degrees(np.arccos(np.clip(ns.mean(0)[2], -1, 1)))
        print(f'迭代 {it+1}: |delta|={np.degrees(np.linalg.norm(delta)):.3f} deg, '
              f'残余倾斜 {tilt:.3f} deg')
        if np.linalg.norm(delta) < 1e-5:
            break

    X2 = X.copy()
    X2[:3, :3] = R_ec
    xyz = X2[:3, 3]
    r, p, y = rot_to_rpy(R_ec)

    # 修正后的 AX=XB 一致性(不应显著变差: 剔除的是系统偏差而非噪声)
    boards = [np.array(s['T_base_end']) @ X2 @ np.array(s['T_cam_board']) for s in samples]
    pos = np.array([B[:3, 3] for B in boards])
    spread = float(np.std(pos, axis=0).max() * 1000)
    print(f'修正后 AX=XB 一致性: 平移离散 {spread:.1f} mm (修正前 '
          f'{data["consistency"]["trans_spread_mm"]:.1f} mm)')

    out = dict(data)
    out['T_end_camera_link_4x4'] = X2.tolist()
    out['mount_xyz'] = [float(v) for v in xyz]
    out['mount_rpy'] = [float(r), float(p), float(y)]
    out['tilt_correction'] = {
        'method': 'table-plane-normal constraint (least squares, rotation only)',
        'tilt_before_deg': float(np.degrees(np.arccos(np.clip(
            board_normals(samples, X[:3, :3]).mean(0)[2], -1, 1)))),
        'tilt_after_deg': float(np.degrees(np.arccos(np.clip(
            board_normals(samples, R_ec).mean(0)[2], -1, 1)))),
    }
    out_path = path.replace('.yaml', '_tiltcorr.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f, allow_unicode=True)
    print(f'\n修正结果:')
    print(f'  mount_xyz: "{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}"')
    print(f'  mount_rpy: "{r:.6f} {p:.6f} {y:.6f}"')
    print(f'已保存: {out_path}')


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


if __name__ == '__main__':
    main()
