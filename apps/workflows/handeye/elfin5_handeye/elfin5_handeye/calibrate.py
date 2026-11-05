"""MoveIt motion -> timestamped rendered ArUco observations -> AX=XB -> validation.

Simulation-only automation. Camera mounting TF is used for aiming and *after*
solving for validation. The solver receives only base_T_hand and optical_T_target.
"""
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import yaml

from .geometry import angle_deg, detect_marker, from_msg, solve, transform


def pose_msg(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


class Calibration(Node):
    def __init__(self):
        super().__init__('elfin5_handeye_calibration')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('This automatic motion routine requires use_sim_time:=true')
        self.declare_parameter('output_dir', 'outputs/handeye')
        self.declare_parameter('samples_file', '')
        self.output = Path(self.get_parameter('output_dir').value)
        self.output.mkdir(parents=True, exist_ok=True)
        # A failed rerun must not leave an earlier PASS looking like its result.
        (self.output / 'calibration.yaml').write_text(yaml.safe_dump(
            {'passed': False, 'status': 'collecting', 'sample_count': 0}))
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.tf, self)
        self.bridge = CvBridge()
        self.info = self.image = self.joints = None
        self.depth = self.imu = self.cloud = None
        self.samples = []
        self.subscriptions_ = [
            self.create_subscription(kind, topic, lambda msg, key=key: setattr(self, key, msg),
                                     qos_profile_sensor_data)
            for kind, topic, key in [
                (CameraInfo, '/camera/color/camera_info', 'info'),
                (Image, '/camera/color/image_raw', 'image'),
                (Image, '/camera/depth/image_rect_raw', 'depth'),
                (PointCloud2, '/camera/depth/color/points', 'cloud'),
                (Imu, '/camera/imu', 'imu'),
                (JointState, '/joint_states', 'joints')]]
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.move = ActionClient(self, MoveGroup, '/move_action')

    def wait(self, predicate, timeout=60, description='data'):
        until = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < until:
            if predicate():
                return
            time.sleep(0.05)
        raise TimeoutError('Timed out waiting for ' + description)

    def result(self, future, timeout=60):
        self.wait(future.done, timeout, 'ROS response')
        if future.exception():
            raise future.exception()
        return future.result()

    def lookup(self, parent, child, stamp=None):
        return from_msg(self.tf.lookup_transform(parent, child, stamp or Time(),
                        timeout=Duration(seconds=2)).transform)

    def add_scene(self):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        for name, size, matrix in [
            ('floor', [4.0, 4.0, 0.1], transform(translation=[0, 0, -0.051])),
            ('calibration_board', [0.02, 0.22, 0.22], transform(translation=[0.756, 0, 0.55]))]:
            obj = CollisionObject()
            obj.header.frame_id = 'world'
            obj.id = name
            obj.operation = CollisionObject.ADD
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
            obj.primitive_poses = [pose_msg(matrix)]
            request.scene.world.collision_objects.append(obj)
        if not self.result(self.scene.call_async(request)).success:
            raise RuntimeError('Could not add floor and target to MoveIt planning scene')

    def move_to(self, matrix):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'elfin_arm'
        request.ik_request.ik_link_name = 'elfin_end_link'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose_msg(matrix)
        request.ik_request.robot_state.joint_state = self.joints
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        response = self.result(self.ik.call_async(request), 10)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ValueError(f'IK failed: {response.error_code.val}')
        constraints = Constraints()
        for name, value in zip(response.solution.joint_state.name, response.solution.joint_state.position):
            if name.startswith('elfin_joint'):
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=value, tolerance_above=0.001,
                    tolerance_below=0.001, weight=1.0))
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
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self.result(self.move.send_goal_async(goal), 10)
        if not handle.accepted:
            raise ValueError('MoveIt rejected goal')
        try:
            result = self.result(handle.get_result_async(), 90).result
        except TimeoutError:
            self.result(handle.cancel_goal_async(), 10)
            raise
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ValueError(f'MoveIt execution failed: {result.error_code.val}')

    def capture(self):
        # Wait for a new image at least 0.8 simulation seconds after execution.
        settled = self.get_clock().now().nanoseconds + 800_000_000
        self.wait(lambda: self.image is not None and
                  Time.from_msg(self.image.header.stamp).nanoseconds > settled, 20, 'settled image')
        msg, info, joints = self.image, self.info, self.joints
        if msg.header.frame_id != 'camera_color_optical_frame' or info.header.frame_id != msg.header.frame_id:
            raise RuntimeError('Image/CameraInfo optical frame mismatch')
        if (msg.width, msg.height) != (info.width, info.height):
            raise RuntimeError('Image/CameraInfo dimensions mismatch')
        if not joints.velocity or max(abs(v) for v in joints.velocity) > 0.015:
            raise ValueError('Robot has not settled')
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(str(self.output / 'latest_camera.png'), image)
        optical_target, error, corners = detect_marker(
            image, np.asarray(info.k).reshape(3, 3), np.asarray(info.d))
        stamp = Time.from_msg(msg.header.stamp)
        hand = self.lookup('world', 'elfin_end_link', stamp)
        for sample in self.samples:
            previous = np.asarray(sample['base_T_hand'])
            if np.linalg.norm(hand[:3, 3] - previous[:3, 3]) < .015 and angle_deg(
                    previous[:3, :3].T @ hand[:3, :3]) < 5:
                raise ValueError('Duplicate sample')
        self.samples.append({'stamp_ns': stamp.nanoseconds, 'base_T_hand': hand.tolist(),
                             'optical_T_target': optical_target.tolist(), 'reprojection_rms_px': error})
        (self.output / 'samples.json').write_text(json.dumps(self.samples, indent=2))
        cv2.polylines(image, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.imwrite(str(self.output / f'sample_{len(self.samples):02}.png'), image)
        self.get_logger().info(f'Sample {len(self.samples)}: reprojection RMS {error:.3f} px')

    def run(self):
        self.wait(lambda: all(x is not None for x in (
            self.info, self.image, self.joints, self.depth, self.imu, self.cloud)),
            90, 'RGB, CameraInfo, depth, point cloud, IMU and joint states')
        saved_samples = self.get_parameter('samples_file').value
        if saved_samples:
            # Replay observations from the same robot/camera configuration. No
            # synthetic observations or new movement are generated in this mode.
            self.samples = json.loads(Path(saved_samples).read_text())
            self.finish()
            return
        if not self.ik.wait_for_service(timeout_sec=30) or not self.scene.wait_for_service(timeout_sec=30):
            raise RuntimeError('MoveIt services unavailable')
        if not self.move.wait_for_server(timeout_sec=30):
            raise RuntimeError('MoveIt action unavailable')
        self.add_scene()
        # Aiming uses the nominal attachment; no camera/world TF enters solve().
        nominal = self.lookup('elfin_end_link', 'camera_color_optical_frame')
        target = np.array([0.75, 0.0, 0.55])
        candidates = [
            (0.35, 0.00, 0.55, 0), (0.32, -0.14, 0.48, -25),
            (0.34, 0.14, 0.62, 25), (0.30, -0.10, 0.66, 20),
            (0.36, 0.10, 0.43, -20), (0.29, 0.00, 0.70, -30),
            (0.38, -0.13, 0.58, 30), (0.30, 0.15, 0.52, -30),
            (0.36, 0.00, 0.40, 25), (0.28, -0.16, 0.52, 0),
            (0.32, 0.13, 0.68, 0), (0.39, -0.08, 0.44, -10),
            (0.31, 0.08, 0.60, 35), (0.28, -0.06, 0.63, -35),
            (0.37, 0.16, 0.48, 15), (0.33, -0.12, 0.70, -15),
        ]
        for index, (x, y, z, roll) in enumerate(candidates):
            position = np.array([x, y, z])
            forward = target - position
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0, 0, 1])
            right /= np.linalg.norm(right)
            down = np.cross(forward, right)
            rotation = np.column_stack([right, down, forward]) @ Rotation.from_euler('z', roll, degrees=True).as_matrix()
            self.get_logger().info(f'MoveIt view {index + 1}/{len(candidates)}')
            try:
                self.move_to(transform(rotation, position) @ np.linalg.inv(nominal))
                self.capture()
            except ValueError as error:
                self.get_logger().warning(str(error))
        self.finish()

    def finish(self):
        optical, metrics = solve(self.samples)
        # Convert the solved optical extrinsic using only the camera's internal TF.
        internal = self.lookup('camera_link', 'camera_color_optical_frame')
        mount = optical @ np.linalg.inv(internal)
        truth = self.lookup('elfin_end_link', 'camera_link')
        metrics['translation_error_mm'] = float(np.linalg.norm(mount[:3, 3] - truth[:3, 3]) * 1000)
        metrics['rotation_error_deg'] = angle_deg(mount[:3, :3].T @ truth[:3, :3])
        passed = (metrics['translation_error_mm'] < 5 and metrics['rotation_error_deg'] < 1
                  and metrics['board_translation_rms_mm'] < 5 and metrics['board_rotation_max_deg'] < 1)
        report = {'passed': passed, 'status': 'complete', 'sample_count': len(self.samples), 'method': 'OpenCV PARK',
                  'sample_source': self.get_parameter('samples_file').value or 'live Gazebo RGB + timestamped TF',
                  'parent_frame': 'elfin_end_link', 'child_frame': 'camera_link',
                  'mount_xyz': mount[:3, 3].tolist(),
                  'mount_rpy': Rotation.from_matrix(mount[:3, :3]).as_euler('xyz').tolist(),
                  'quaternion_xyzw': Rotation.from_matrix(mount[:3, :3]).as_quat().tolist(),
                  'hand_T_optical': optical.tolist(), 'hand_T_camera_link': mount.tolist(),
                  'ground_truth_hand_T_camera_link': truth.tolist(), 'metrics': metrics,
                  'streams': {'color_frame': self.image.header.frame_id,
                              'depth_frame': self.depth.header.frame_id, 'depth_encoding': self.depth.encoding,
                              'imu_frame': self.imu.header.frame_id, 'pointcloud_frame': self.cloud.header.frame_id}}
        (self.output / 'calibration.yaml').write_text(yaml.safe_dump(report, sort_keys=False))
        self.get_logger().info(yaml.safe_dump(report, sort_keys=False))
        if not passed:
            raise RuntimeError('Calibration did not meet 5 mm / 1 degree acceptance limits')


def main():
    rclpy.init()
    node = Calibration()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        path = node.output / 'calibration.yaml'
        report = yaml.safe_load(path.read_text()) if path.exists() else {}
        report.update(passed=False, status='failed', error=str(error), sample_count=len(node.samples))
        path.write_text(yaml.safe_dump(report, sort_keys=False))
        raise
    finally:
        executor.shutdown()
        worker.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
