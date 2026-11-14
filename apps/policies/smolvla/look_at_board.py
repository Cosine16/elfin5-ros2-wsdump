#!/usr/bin/env python3
"""Move the Elfin5 wrist so the D435i looks at the ArUco board in the Gz
handeye world (reuses the aiming math from elfin5_handeye.calibrate).
Run with use_sim_time:=true against the running handeye simulation.
"""
import threading

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK

from elfin5_handeye.geometry import from_msg, transform

TARGET = np.array([0.75, 0.0, 0.55])   # ArUco board center in the handeye world
CAMERA_POS = np.array([0.35, 0.0, 0.55])  # a reachable wrist-camera viewpoint


def pose_msg(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


class LookAtBoard(Node):
    def __init__(self):
        super().__init__('look_at_board')
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.tf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.spin_executor = SingleThreadedExecutor()
        self.spin_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.spin_executor.spin, daemon=True)
        self.spin_thread.start()

    def result(self, future, timeout=60):
        import time
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError('ROS response timeout')
            time.sleep(0.05)
        return future.result()

    def run(self):
        if not self.ik.wait_for_service(timeout_sec=30):
            raise RuntimeError('MoveIt IK service unavailable')
        if not self.move.wait_for_server(timeout_sec=30):
            raise RuntimeError('MoveIt action unavailable')
        nominal = from_msg(self.tf.lookup_transform(
            'elfin_end_link', 'camera_color_optical_frame', Time(),
            timeout=Duration(seconds=5)).transform)

        forward = TARGET - CAMERA_POS
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0, 0, 1])
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        rotation = np.column_stack([right, down, forward])
        goal_matrix = transform(rotation, CAMERA_POS) @ np.linalg.inv(nominal)

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'elfin_arm'
        request.ik_request.ik_link_name = 'elfin_end_link'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose_msg(goal_matrix)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        response = self.result(self.ik.call_async(request), 15)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'IK failed: {response.error_code.val}')

        constraints = Constraints()
        for name, value in zip(response.solution.joint_state.name,
                               response.solution.joint_state.position):
            if name.startswith('elfin_joint'):
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=value,
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        goal = MoveGroup.Goal()
        goal.request.group_name = 'elfin_arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        handle = self.result(self.move.send_goal_async(goal), 15)
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected goal')
        result = self.result(handle.get_result_async(), 120).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt execution failed: {result.error_code.val}')
        self.get_logger().info('Camera now aimed at the ArUco board.')


def main():
    rclpy.init()
    node = LookAtBoard()
    try:
        node.run()
    finally:
        node.spin_executor.shutdown()
        node.spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
