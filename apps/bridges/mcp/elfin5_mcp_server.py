#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Elfin5 MCP Server
=================

以 stdio 方式运行的 MCP (Model Context Protocol) 服务器，把 Hans Robot Elfin5
六轴机械臂的 ROS2 控制接口封装成 LLM agent 可调用的工具。

架构参考 RoboNeuron (github.com/guanweifan/RoboNeuron) 的 FastMCP stdio 模式，
但针对 Elfin5 的实际情况做了两点调整：
  1. Elfin5 的使能/复位/急停都是 ROS2 service，这里直接封装 service 调用；
  2. 运动指令走 elfin_basic_api 的 /joint_goal、/cart_goal、/cart_path_goal，
     由 basic_api 内部用 MoveIt 规划并执行（带碰撞检查），而不是差分 IK 高频关节流。

依赖：系统 ROS2 Humble 环境（rclpy 等）+ venv 中的 mcp<2。
启动：请使用 run_server.sh（会 source ROS 环境）。
"""

import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

from mcp.server.fastmcp import FastMCP

try:
    from elfin_robot_msgs.srv import ElfinIODRead, ElfinIODWrite
    _HAS_IO_SRV = True
except ImportError:
    _HAS_IO_SRV = False

JOINT_NAMES = [f'elfin_joint{i}' for i in range(1, 7)]
DEFAULT_BASE_FRAME = 'elfin_base_link'
DEFAULT_END_FRAME = 'elfin_end_link'
# 法兰竖直朝下的默认姿态（与官方 cart_goal_demo 一致）
FLANGE_DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)

SERVICE_TIMEOUT_SEC = 15.0
PUB_MATCH_TIMEOUT_SEC = 3.0


class Elfin5Bridge(Node):
    """持有与 Elfin5 ROS2 系统通信的全部 publisher/subscriber/client。"""

    def __init__(self):
        super().__init__('elfin5_mcp_server')

        self.joint_goal_pub = self.create_publisher(JointState, '/joint_goal', 1)
        self.cart_goal_pub = self.create_publisher(PoseStamped, '/cart_goal', 1)
        self.cart_path_pub = self.create_publisher(PoseArray, '/cart_path_goal', 1)
        self.vel_pub = self.create_publisher(Float32, '/vel', 1)

        self._latest_joint_states: Optional[JointState] = None
        self._enable_state: Optional[bool] = None
        self._fault_state: Optional[bool] = None

        self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, '_latest_joint_states', msg), 10)
        self.create_subscription(
            Bool, '/enable_state',
            lambda msg: setattr(self, '_enable_state', msg.data), 10)
        self.create_subscription(
            Bool, '/fault_state',
            lambda msg: setattr(self, '_fault_state', msg.data), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ---- 通用辅助 ----

    def call_setbool(self, srv_name: str, data: bool,
                     timeout: float = SERVICE_TIMEOUT_SEC) -> dict:
        client = self.create_client(SetBool, srv_name)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                return {'ok': False,
                        'message': f'服务 {srv_name} 不可用（elfin_basic_api / 驱动未启动？）'}
            req = SetBool.Request()
            req.data = data
            future = client.call_async(req)
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() > deadline:
                    return {'ok': False, 'message': f'呼叫 {srv_name} 超时（>{timeout}s）'}
                time.sleep(0.02)
            res = future.result()
            return {'ok': bool(res.success), 'message': res.message}
        finally:
            self.destroy_client(client)

    def publish_with_match(self, pub, msg, topic: str) -> dict:
        deadline = time.monotonic() + PUB_MATCH_TIMEOUT_SEC
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        matched = pub.get_subscription_count()
        pub.publish(msg)
        if matched == 0:
            return {'ok': False,
                    'message': f'{topic} 没有订阅者（elfin_basic_api 未启动？），指令未送达'}
        return {'ok': True, 'message': f'已发布到 {topic}（{matched} 个订阅者）'}


_node: Optional[Elfin5Bridge] = None
_executor: Optional[MultiThreadedExecutor] = None


def _ensure_ros() -> Elfin5Bridge:
    global _node, _executor
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = Elfin5Bridge()
        _executor = MultiThreadedExecutor()
        _executor.add_node(_node)
        threading.Thread(target=_executor.spin, daemon=True).start()
    return _node


mcp = FastMCP(
    'elfin5',
    instructions=(
        'Hans Robot Elfin5 六轴机械臂控制。典型流程：check_connection 确认后端在线 → '
        'clear_fault 清故障 → enable_robot 使能（仅真机）→ move_j / move_l 运动 → '
        'stop_motion 可随时停止 → 结束后 disable_robot 下电。'
        '位置单位米，关节单位弧度，姿态用四元数 (qx,qy,qz,qw)，默认 (0,1,0,0) 法兰竖直朝下。'
        '所有运动指令由 elfin_basic_api 经 MoveIt 规划后立即执行，执行前请确认工作空间安全。'
    ),
)


# ---- 状态查询 ----

@mcp.tool()
def check_connection() -> dict:
    """检查 MCP server 与 Elfin5 ROS2 后端（elfin_basic_api、驱动）的连接状态。

    在做任何运动前先调用它。返回各关键服务/话题是否在线。
    """
    node = _ensure_ros()
    basic_api_up = node.create_client(SetBool, '/elfin_basic_api/enable_robot') \
        .wait_for_service(timeout_sec=1.0)
    driver_up = node.create_client(SetBool, '/clear_fault') \
        .wait_for_service(timeout_sec=1.0)
    return {
        'ok': True,
        'basic_api_online': basic_api_up,
        'driver_online': driver_up,
        'joint_states_received': node._latest_joint_states is not None,
        'message': 'basic_api_online 为 True 时才能运动；joint_states_received 为 True 说明驱动数据正常',
    }


@mcp.tool()
def get_robot_status() -> dict:
    """获取机器人当前状态：是否已使能、是否有故障、是否在运动中。"""
    node = _ensure_ros()
    motion = node.call_setbool('/get_motion_state', True, timeout=5.0)
    return {
        'ok': True,
        'enabled': node._enable_state,
        'fault': node._fault_state,
        'moving': motion.get('message') if motion.get('ok') else None,
        'message': 'enabled/fault 来自驱动状态话题，None 表示暂未收到（驱动未启动）',
    }


@mcp.tool()
def get_joint_states() -> dict:
    """获取六个关节的当前角度（弧度）。"""
    node = _ensure_ros()
    deadline = time.monotonic() + 3.0
    while node._latest_joint_states is None and time.monotonic() < deadline:
        time.sleep(0.1)
    js = node._latest_joint_states
    if js is None:
        return {'ok': False, 'message': '尚未收到 /joint_states，驱动或仿真未启动'}
    joints = dict(zip(js.name, [round(p, 4) for p in js.position]))
    return {'ok': True, 'joints': joints,
            'ordered': [joints.get(n) for n in JOINT_NAMES]}


@mcp.tool()
def get_end_pose(base_frame: str = DEFAULT_BASE_FRAME,
                 end_frame: str = DEFAULT_END_FRAME) -> dict:
    """通过 TF 获取末端（默认 elfin_end_link）在基座坐标系下的位姿。

    Args:
        base_frame: 参考坐标系，默认 elfin_base_link
        end_frame: 末端坐标系，默认 elfin_end_link
    """
    node = _ensure_ros()
    try:
        tf = node.tf_buffer.lookup_transform(
            base_frame, end_frame, Time(), timeout=Duration(seconds=2.0))
    except Exception as exc:
        return {'ok': False, 'message': f'TF 查询失败: {exc}'}
    t, r = tf.transform.translation, tf.transform.rotation
    return {
        'ok': True,
        'position': {'x': round(t.x, 4), 'y': round(t.y, 4), 'z': round(t.z, 4)},
        'orientation': {'qx': round(r.x, 4), 'qy': round(r.y, 4),
                        'qz': round(r.z, 4), 'qw': round(r.w, 4)},
        'frame': f'{end_frame} in {base_frame}',
    }


# ---- 使能与安全 ----

@mcp.tool()
def clear_fault() -> dict:
    """清除驱动故障（对应官方面板的 "Clear Fault"）。使能前如有故障需先调用。仅真机有效。"""
    return _ensure_ros().call_setbool('/clear_fault', True)


@mcp.tool()
def enable_robot() -> dict:
    """伺服上电使能（"Servo On"）。

    内部流程：停控制器 → 检查运动/对齐状态 → 调底层使能 → 重启控制器。
    仅真机有效；仿真环境会返回 "there is no real driver running"。
    """
    return _ensure_ros().call_setbool('/elfin_basic_api/enable_robot', True)


@mcp.tool()
def disable_robot() -> dict:
    """伺服下电去使能（"Servo Off"）。结束操作或断电前必须调用。仅真机有效。"""
    return _ensure_ros().call_setbool('/elfin_basic_api/disable_robot', True)


@mcp.tool()
def stop_motion() -> dict:
    """软急停：立即停止当前运动。发现异常时第一时间调用。"""
    return _ensure_ros().call_setbool('/stop_teleop', True, timeout=5.0)


@mcp.tool()
def home_robot() -> dict:
    """回到 home 位姿。"""
    return _ensure_ros().call_setbool('/home_teleop', True)


@mcp.tool()
def set_velocity_scale(scale: float) -> dict:
    """设置运动速度缩放（0~1）。调试时建议先用 0.1~0.3 低速验证。

    Args:
        scale: 速度缩放系数，范围 (0, 1]
    """
    if not 0.0 < scale <= 1.0:
        return {'ok': False, 'message': 'scale 必须在 (0, 1] 范围内'}
    node = _ensure_ros()
    msg = Float32()
    msg.data = float(scale)
    return node.publish_with_match(node.vel_pub, msg, '/vel')


# ---- 运动 ----

@mcp.tool()
def move_j(joints: list[float]) -> dict:
    """关节空间运动（move_j）：MoveIt 规划到指定关节角并执行。

    Args:
        joints: 六个关节的目标角度（弧度），顺序 elfin_joint1~6，例如 [0, 0.5, 1.0, 0, 0.5, 0]
    """
    if len(joints) != 6:
        return {'ok': False, 'message': f'需要 6 个关节角，收到 {len(joints)} 个'}
    if not all(math.isfinite(j) for j in joints):
        return {'ok': False, 'message': '关节角必须是有限数值'}
    node = _ensure_ros()
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = JOINT_NAMES
    msg.position = [float(j) for j in joints]
    return node.publish_with_match(node.joint_goal_pub, msg, '/joint_goal')


@mcp.tool()
def move_l(x: float, y: float, z: float,
           qx: float = FLANGE_DOWN_QUAT[0], qy: float = FLANGE_DOWN_QUAT[1],
           qz: float = FLANGE_DOWN_QUAT[2], qw: float = FLANGE_DOWN_QUAT[3],
           frame_id: str = DEFAULT_BASE_FRAME) -> dict:
    """笛卡尔空间点位运动（move_l）：MoveIt 规划到指定末端位姿并执行。

    Args:
        x, y, z: 目标位置（米），默认在 elfin_base_link 坐标系下
        qx, qy, qz, qw: 目标姿态四元数，默认 (0,1,0,0) 法兰竖直朝下
        frame_id: 参考坐标系
    """
    node = _ensure_ros()
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return node.publish_with_match(node.cart_goal_pub, msg, '/cart_goal')


@mcp.tool()
def move_cartesian_path(points: list[list[float]],
                        qx: float = FLANGE_DOWN_QUAT[0], qy: float = FLANGE_DOWN_QUAT[1],
                        qz: float = FLANGE_DOWN_QUAT[2], qw: float = FLANGE_DOWN_QUAT[3],
                        frame_id: str = DEFAULT_BASE_FRAME) -> dict:
    """笛卡尔直线轨迹：末端以相同姿态沿直线依次经过各点。

    Args:
        points: 途径点列表，每个点为 [x, y, z]（米），至少 1 个点
        qx, qy, qz, qw: 全程保持的姿态四元数，默认法兰竖直朝下
        frame_id: 参考坐标系
    """
    if not points or any(len(p) != 3 for p in points):
        return {'ok': False, 'message': 'points 必须是非空的 [x, y, z] 列表'}
    node = _ensure_ros()
    msg = PoseArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    for p in points:
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = map(float, p)
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        msg.poses.append(pose)
    return node.publish_with_match(node.cart_path_pub, msg, '/cart_path_goal')


# ---- 末端 IO / 夹爪（仅真机）----

@mcp.tool()
def write_do(value: int) -> dict:
    """写末端数字输出 DO（夹爪等末端执行器控制）。仅真机有效。

    Args:
        value: DO 位掩码整数，官方示例中 8192 (0x2000) 为常见取值
    """
    if not _HAS_IO_SRV:
        return {'ok': False, 'message': 'elfin_robot_msgs 未编译/未 source，IO 功能不可用'}
    node = _ensure_ros()
    client = node.create_client(ElfinIODWrite, '/write_do')
    try:
        if not client.wait_for_service(timeout_sec=3.0):
            return {'ok': False, 'message': '/write_do 服务不可用（仅真机驱动提供）'}
        req = ElfinIODWrite.Request()
        req.digital_output = int(value)
        future = client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                return {'ok': False, 'message': '呼叫 /write_do 超时'}
            time.sleep(0.02)
        return {'ok': bool(future.result().success), 'message': f'已写 DO={value}'}
    finally:
        node.destroy_client(client)


@mcp.tool()
def read_di() -> dict:
    """读末端数字输入 DI（夹爪/传感器状态）。仅真机有效。"""
    if not _HAS_IO_SRV:
        return {'ok': False, 'message': 'elfin_robot_msgs 未编译/未 source，IO 功能不可用'}
    node = _ensure_ros()
    client = node.create_client(ElfinIODRead, '/read_di')
    try:
        if not client.wait_for_service(timeout_sec=3.0):
            return {'ok': False, 'message': '/read_di 服务不可用（仅真机驱动提供）'}
        req = ElfinIODRead.Request()
        req.data = True
        future = client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                return {'ok': False, 'message': '呼叫 /read_di 超时'}
            time.sleep(0.02)
        return {'ok': True, 'digital_input': future.result().digital_input}
    finally:
        node.destroy_client(client)


def main():
    _ensure_ros()
    code = 0
    try:
        mcp.run(transport='stdio')
    except Exception as exc:
        print(f'elfin5_mcp_server 异常退出: {exc}', file=sys.stderr)
        code = 1
    finally:
        # 后台 spin 线程持有 C++ 执行器，走正常解释器退出路径会在静态析构阶段
        # 触发 "terminate called without an active exception"；stdio server 退出
        # 即进程使命结束，flush 后直接 _exit 跳过静态析构。
        # 注意此时 stdout 可能已被 stdio transport 关闭，flush 需忽略异常。
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except Exception:
            pass
        os._exit(code)

if __name__ == '__main__':
    main()
