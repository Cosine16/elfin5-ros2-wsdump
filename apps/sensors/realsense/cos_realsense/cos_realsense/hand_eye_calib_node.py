#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# hand_eye_calib_node: 眼在手上手眼标定(cos_rsvisual hand_eye_calibrator.hpp 设计稿的落地实现)。
#
# 流程: 棋盘格平放桌面不动 -> 机械臂带相机到 N 个不同位姿 -> 每组样本 =
#   TF(elfin_base_link->elfin_end_link) + 棋盘格在相机系位姿(findChessboardCorners+solvePnP)
#   -> cv2.calibrateHandEye(Tsai) 解 AX=XB 得 elfin_end_link -> camera_color_optical_frame,
#   再借 realsense 驱动发布的 camera_link->*_optical_frame 静态 TF 换算成
#   elfin_end_link -> camera_link, 输出 cos_rsvisual launch 的 mount_xyz/mount_rpy。
#
# 键盘: 空格/s 采样  d 删上一组  r 求解  p 状态  q 退出
# 自检: python3 hand_eye_calib_node.py --self-test (合成数据验证解算与坐标换算, 无需硬件)

import math
import os
import sys
import threading
from datetime import datetime

import cv2
import numpy as np
import yaml

PATTERN_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)


def rot_to_rpy(R):
    # xacro/URDF 约定: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def rot_angle(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def detect_board(bgr, pattern, objp, K, D):
    """返回 (R_cam_board, t_cam_board, corners) 或 None。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    ok, corners = False, None
    if hasattr(cv2, 'findChessboardCornersSB'):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        ok, corners = cv2.findChessboardCorners(gray, pattern, PATTERN_FLAGS | cv2.CALIB_CB_FAST_CHECK)
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    if not ok:
        return None
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), corners


def solve_handeye(samples):
    """samples: [{'T_base_end':4x4, 'R_cam_board':3x3, 't_cam_board':3}]
    返回 (T_end_cam 4x4, trans_spread_mm, max_rot_dev_deg)"""
    R_g2b = [s['T_base_end'][:3, :3] for s in samples]
    t_g2b = [s['T_base_end'][:3, 3] for s in samples]
    R_t2c = [s['R_cam_board'] for s in samples]
    t_t2c = [s['t_cam_board'] for s in samples]
    R_c2g, t_c2g = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI)
    T_end_cam = make_T(R_c2g, t_c2g.reshape(3))

    # 一致性: 同一静止棋盘格, 由各样本反推的"板在 base 系位姿"应完全重合
    boards = [s['T_base_end'] @ T_end_cam @
              make_T(s['R_cam_board'], s['t_cam_board']) for s in samples]
    pos = np.array([B[:3, 3] for B in boards])
    trans_spread_mm = float(np.std(pos, axis=0).max() * 1000.0)
    M = sum(B[:3, :3] for B in boards) / len(boards)
    U, _, Vt = np.linalg.svd(M)
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1
        Rm = U @ Vt
    max_rot_dev = max(rot_angle(Rm.T @ B[:3, :3]) for B in boards)
    return T_end_cam, trans_spread_mm, float(max_rot_dev)


def self_test():
    rng = np.random.default_rng(7)
    T_end_cam_gt = make_T(cv2.Rodrigues(np.array([1.57, 0.0, -1.57]))[0],
                          np.array([0.01, -0.04, 0.08]))
    T_base_board = make_T(cv2.Rodrigues(np.array([math.pi, 0.0, 0.0]))[0],
                          np.array([0.30, 0.05, 0.0]))
    samples = []
    for _ in range(15):
        p_end = np.array([0.30, 0.05, 0.0]) + rng.uniform([-0.25, -0.25, 0.25],
                                                          [0.25, 0.25, 0.55])
        d = np.array([0.30, 0.05, 0.0]) - p_end
        z = d / np.linalg.norm(d)
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.column_stack([x, y, z])
        roll = cv2.Rodrigues(z * rng.uniform(-0.8, 0.8))[0]
        T_base_end = make_T(roll @ R, p_end)
        T_cam_board = np.linalg.inv(T_end_cam_gt) @ np.linalg.inv(T_base_end) @ T_base_board
        samples.append({'T_base_end': T_base_end,
                        'R_cam_board': T_cam_board[:3, :3],
                        't_cam_board': T_cam_board[:3, 3]})
    T_est, spread_mm, rot_dev = solve_handeye(samples)
    terr = np.linalg.norm(T_est[:3, 3] - T_end_cam_gt[:3, 3]) * 1000.0
    rerr = rot_angle(T_est[:3, :3].T @ T_end_cam_gt[:3, :3])
    print(f'[self-test] 平移误差 {terr:.4f} mm, 旋转误差 {rerr:.6f} deg, '
          f'一致性 {spread_mm:.4f} mm / {rot_dev:.6f} deg')
    r, p, y = rot_to_rpy(T_est[:3, :3])
    q = rot_to_quat(T_est[:3, :3])
    print(f'[self-test] xyz={T_est[:3, 3].round(5).tolist()} '
          f'rpy=({r:.4f},{p:.4f},{y:.4f}) quat={q.round(5).tolist()}')
    ok = terr < 0.01 and rerr < 0.01
    print('[self-test]', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


class HandEyeCalibNode:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        self._rclpy = rclpy
        self.node = Node('hand_eye_calib')
        n = self.node
        n.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        n.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        n.declare_parameter('squares_x', 9)          # 棋盘格横向格子数(内角点=格子数-1)
        n.declare_parameter('squares_y', 9)
        n.declare_parameter('square_size', 0.018)    # 单格边长 m, 打印后务必实测(本机棋盘格实测 162mm/9格)
        n.declare_parameter('base_frame', 'elfin_base_link')
        n.declare_parameter('end_frame', 'elfin_end_link')
        n.declare_parameter('min_trans', 0.03)       # 与上一样本最小平移差 m
        n.declare_parameter('min_rot_deg', 8.0)      # 与上一样本最小旋转差 deg
        n.declare_parameter('output_dir', os.path.expanduser('~/.ros/hand_eye_calib'))
        n.declare_parameter('show_view', True)     # 实时监视窗(画出识别到的角点)

        gp = n.get_parameter
        self.image_topic = gp('image_topic').value
        self.base_frame = gp('base_frame').value
        self.end_frame = gp('end_frame').value
        self.pattern = (gp('squares_x').value - 1, gp('squares_y').value - 1)
        self.square_size = gp('square_size').value
        self.min_trans = gp('min_trans').value
        self.min_rot = gp('min_rot_deg').value
        self.output_dir = gp('output_dir').value
        self.show_view = gp('show_view').value
        self._view_warned = False
        self._last_view_det = 0.0

        gx, gy = self.pattern
        self.objp = np.zeros((gx * gy, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:gx, 0:gy].T.reshape(-1, 2) * self.square_size

        from cv_bridge import CvBridge
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        import tf2_ros
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.latest = None          # (bgr, frame_id, stamp)
        self.samples = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, n)
        self.info_sub = n.create_subscription(
            CameraInfo, gp('camera_info_topic').value, self._on_info, 10)
        self.img_sub = n.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        n.get_logger().info(
            f'\n===== 眼在手上手眼标定 =====\n'
            f'  图像: {self.image_topic}   棋盘格内角点: {gx}x{gy} (格子 {gx+1}x{gy+1})\n'
            f'  格边长: {self.square_size} m   TF: {self.base_frame} -> {self.end_frame}\n'
            f'  棋盘格平放桌面固定不动, 机械臂带相机变换位姿, 板子全程在视野内。\n'
            f'  按键: [空格]采样  [d]删上一组  [r]求解  [p]状态  [q]退出\n'
            f'  建议 12~15 组, 姿态变化大(多转手腕), 板子出现在画面不同区域。')

    def _on_info(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.D = np.array(msg.d, dtype=float)
        self.node.get_logger().info(
            f'已收到 camera_info: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
            f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def _on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().warn(f'图像转换失败: {e}')
            return
        self.latest = (bgr, msg.header.frame_id, msg.header.stamp)
        if self.show_view:
            self._update_view(bgr)

    def _update_view(self, bgr):
        try:
            view = bgr.copy()
            found = None
            now = self.node.get_clock().now().nanoseconds
            if self.K is not None and now - self._last_view_det > 300e6:  # 检测限频 ~3Hz
                self._last_view_det = now
                found = detect_board(bgr, self.pattern, self.objp, self.K, self.D)
                self._view_det = found
            else:
                found = getattr(self, '_view_det', None)
            if found is not None:
                cv2.drawChessboardCorners(view, self.pattern, found[2], True)
                cv2.putText(view, f'BOARD OK  dist={np.linalg.norm(found[1]):.2f}m',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                cv2.putText(view, 'no board', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(view, f'samples: {len(self.samples)}   [space]=sample [r]=solve [q]=quit',
                        (10, view.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1)
            cv2.imshow('hand_eye_calib', view)
            cv2.waitKey(1)
        except cv2.error as e:
            if not self._view_warned:
                self._view_warned = True
                self.show_view = False
                self.node.get_logger().warn(f'监视窗不可用({e}), 已关闭; 可改用 rqt_image_view 看原始图像')

    def _lookup_T(self, target, source):
        from rclpy.duration import Duration
        tf = self.tf_buffer.lookup_transform(
            target, source, self._rclpy.time.Time(), timeout=Duration(seconds=1.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        return make_T(quat_to_rot(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

    def take_sample(self):
        log = self.node.get_logger().info
        if self.K is None:
            log('还没有 camera_info, 检查相机节点是否已启动')
            return
        if self.latest is None:
            log(f'还没有图像, 检查 {self.image_topic} 是否有数据 (ros2 topic hz)')
            return
        bgr, frame_id, _ = self.latest
        det = detect_board(bgr, self.pattern, self.objp, self.K, self.D)
        if det is None:
            log(f'未检测到 {self.pattern[0]}x{self.pattern[1]} 内角点棋盘格, '
                f'调整位姿/光照后重试(格子数填错会一直检测不到)')
            return
        R_cb, t_cb, _ = det
        try:
            T_base_end = self._lookup_T(self.base_frame, self.end_frame)
        except Exception as e:
            log(f'TF 查询失败 {self.base_frame}->{self.end_frame}: {e} (机械臂栈在跑吗?)')
            return
        if self.samples:
            last = self.samples[-1]['T_base_end']
            dtrans = np.linalg.norm(T_base_end[:3, 3] - last[:3, 3])
            drot = rot_angle(last[:3, :3].T @ T_base_end[:3, :3])
            if dtrans < self.min_trans and drot < self.min_rot:
                log(f'位姿变化太小(平移 {dtrans*1000:.0f}mm / 旋转 {drot:.1f}deg), '
                    f'换个位姿再采')
                return
        self.samples.append({'T_base_end': T_base_end, 'R_cam_board': R_cb,
                             't_cam_board': t_cb, 'image_frame': frame_id})
        dist = np.linalg.norm(t_cb)
        log(f'样本 #{len(self.samples)} 已采集 (板距相机 {dist:.2f} m)')

    def drop_last(self):
        if self.samples:
            self.samples.pop()
            self.node.get_logger().info(f'已删除上一组, 剩 {len(self.samples)} 组')

    def status(self):
        self.node.get_logger().info(
            f'样本 {len(self.samples)} 组, camera_info {"OK" if self.K is not None else "缺失"}, '
            f'图像 {"OK" if self.latest else "缺失"}')

    def solve(self):
        log = self.node.get_logger().info
        if len(self.samples) < 4:
            log(f'样本不足({len(self.samples)}/4+), 继续采样')
            return
        if len(self.samples) < 8:
            log(f'警告: 只有 {len(self.samples)} 组样本, 建议 12 组以上')
        T_end_cam, spread_mm, rot_dev = solve_handeye(self.samples)

        # optical frame -> camera_link: 借 realsense 驱动发布的静态 TF
        frame_id = self.samples[-1]['image_frame']
        try:
            T_link_opt = self._lookup_T('camera_link', frame_id)
            T_end_link = T_end_cam @ np.linalg.inv(T_link_opt)
            src = f'TF(camera_link->{frame_id})'
        except Exception as e:
            # 驱动未发 TF 时退回理想旋转: link(x前y左z上) <- optical(z前x右y下), 零平移
            R_link_opt = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
            T_end_link = T_end_cam @ make_T(R_link_opt.T, [0, 0, 0])
            src = f'理想旋转(TF 查询失败: {e}, 结果平移可能有数 mm 误差)'

        xyz = T_end_link[:3, 3]
        r, p, y = rot_to_rpy(T_end_link[:3, :3])
        q = rot_to_quat(T_end_link[:3, :3])
        report = (
            f'\n================ 标定结果 ({len(self.samples)} 组样本, Tsai, 光学系来自{src}) ================\n'
            f'一致性: 板位姿平移离散 {spread_mm:.1f} mm, 旋转离散 {rot_dev:.2f} deg '
            f'{"(良好)" if spread_mm < 5 and rot_dev < 1 else "(偏大, 建议补样本重解)"}\n'
            f'elfin_end_link -> camera_link:\n'
            f'  mount_xyz: "{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}"\n'
            f'  mount_rpy: "{r:.6f} {p:.6f} {y:.6f}"\n'
            f'  quaternion(xyzw): {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n'
            f'填入 launch: ros2 launch cos_rsvisual elfin5_rs_face_real.launch.py \\\n'
            f'    mount_xyz:="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" \\\n'
            f'    mount_rpy:="{r:.6f} {p:.6f} {y:.6f}"\n'
            f'手动验发 TF: ros2 run tf2_ros static_transform_publisher \\\n'
            f'    {xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} {y:.6f} {p:.6f} {r:.6f} '
            f'elfin_end_link camera_link\n'
            f'=======================================================================')
        log('\n' + report)

        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(
            self.output_dir, 'hand_eye_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump({
                'frames': {'parent': 'elfin_end_link', 'child': 'camera_link'},
                'mount_xyz': [float(v) for v in xyz],
                'mount_rpy': [float(r), float(p), float(y)],
                'quaternion_xyzw': [float(v) for v in q],
                'samples': len(self.samples),
                'pattern_inner_corners': list(self.pattern),
                'square_size_m': self.square_size,
                'consistency': {'trans_spread_mm': spread_mm, 'rot_dev_deg': rot_dev},
                'T_end_camera_link_4x4': T_end_link.tolist(),
            }, f, allow_unicode=True)
        log(f'结果已保存: {path}')

    def run(self):
        n = self.node
        use_kb = sys.stdin.isatty()
        if use_kb:
            import termios
            import tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        else:
            n.get_logger().warn('stdin 不是终端, 键盘控制不可用(仅自检/挂起)')

        def key_loop():
            import select
            while self._rclpy.ok():
                if not select.select([sys.stdin], [], [], 0.2)[0]:
                    continue
                c = sys.stdin.read(1)
                if c in (' ', 's'):
                    self.take_sample()
                elif c == 'd':
                    self.drop_last()
                elif c == 'r':
                    self.solve()
                elif c == 'p':
                    self.status()
                elif c == 'q':
                    self._rclpy.shutdown()
                    return

        t = threading.Thread(target=key_loop, daemon=True) if use_kb else None
        try:
            if t:
                t.start()
            self._rclpy.spin(n)
        finally:
            if use_kb:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            cv2.destroyAllWindows()
            n.destroy_node()


def main():
    if '--self-test' in sys.argv:
        sys.exit(self_test())
    import rclpy
    rclpy.init()
    try:
        HandEyeCalibNode().run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
