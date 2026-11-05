#!/usr/bin/env python3
"""SmolVLA <-> Elfin5 ROS2 bridge.

Subscribes to joint states + camera image, runs a SmolVLA policy, and streams
the resulting joint targets to the arm's JointTrajectoryController.

The pretrained model (e.g. lerobot/smolvla_base) is trained on SO-100/SO-101,
NOT on Elfin5, so its absolute action values are meaningless for this arm.
This bridge therefore integrates the *relative* motion of the policy output:
    target += gain * (action[t] - action[t-1])
with per-tick clamping and URDF joint limits, so the arm moves smoothly and
stays safe. Fine-tuning on Elfin5 data is required for real task behavior.
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [f"elfin_joint{i}" for i in range(1, 7)]
JOINT_LOWER = np.array([-3.14, -2.35, -2.61, -3.14, -2.56, -3.14])
JOINT_UPPER = np.array([3.14, 2.35, 2.61, 3.14, 2.56, 3.14])


class SmolVlaBridge(Node):
    def __init__(self):
        super().__init__("smolvla_bridge")

        self.declare_parameter("model_id", "lerobot/smolvla_base")
        self.declare_parameter("task", "pick up the object")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("traj_topic", "/elfin_arm_controller/joint_trajectory")
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("gain", 0.02)          # rad per unit of policy action delta
        self.declare_parameter("max_step_rad", 0.08)  # per-tick absolute clamp
        self.declare_parameter("device", "auto")      # auto | cuda | cpu
        self.declare_parameter("fallback_image", "")  # png/jpg used until first camera frame

        p = lambda name: self.get_parameter(name).value
        self.task = p("task")
        self.gain = float(p("gain"))
        self.max_step = float(p("max_step_rad"))
        self.control_hz = float(p("control_hz"))

        import torch

        device = p("device")
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device

        model_id = p("model_id")
        self.get_logger().info(f"Loading SmolVLA policy '{model_id}' on {device} ...")
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        self.policy = SmolVLAPolicy.from_pretrained(model_id)
        self.policy.config.device = device
        self.policy.to(device)
        self.policy.eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config, pretrained_path=model_id
        )
        self.get_logger().info("Policy loaded.")

        self._lock = threading.Lock()
        self._joint_pos = None          # np.array(6), latest arm positions
        self._image = None              # np.array HxWx3 uint8 rgb
        self._prev_action = None
        self._target = None             # current commanded target (integrated)

        fb = p("fallback_image")
        if fb:
            self._image = self._load_image_file(fb)
            self.get_logger().info(f"Using fallback image until camera frames arrive: {fb}")

        bebop = QoSProfile(depth=1)
        bebop.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(JointState, p("state_topic"), self._on_joint_state, 10)
        self.create_subscription(Image, p("image_topic"), self._on_image, bebop)
        self._traj_pub = self.create_publisher(JointTrajectory, p("traj_topic"), 10)
        self._timer = self.create_timer(1.0 / self.control_hz, self._tick)
        self._infer_count = 0

    @staticmethod
    def _load_image_file(path: str) -> np.ndarray:
        import cv2

        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(f"cannot read fallback image: {path}")
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def _on_joint_state(self, msg: JointState):
        pos = np.full(6, np.nan)
        for name, p_ in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                pos[JOINT_NAMES.index(name)] = p_
        if not np.isnan(pos).any():
            with self._lock:
                self._joint_pos = pos

    def _on_image(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgr8":
            arr = arr[..., ::-1]
        elif msg.encoding == "bgra8":
            arr = arr[..., [2, 1, 0]]
        elif msg.encoding == "rgba8":
            arr = arr[..., :3]
        with self._lock:
            self._image = arr[..., :3].copy()

    def _build_observation(self, joint_pos: np.ndarray, image: np.ndarray) -> dict:
        """Unbatched raw observation; the preprocessor adds batch dim, tokenizes
        the task string, normalizes features and moves everything to device."""
        import torch

        obs = {}
        for key, feat in self.policy.config.input_features.items():
            if key == "observation.state":
                state = np.zeros(feat.shape[0], dtype=np.float32)
                n = min(6, feat.shape[0])
                state[:n] = joint_pos[:n]
                obs[key] = torch.from_numpy(state)
            elif key.startswith("observation.images."):
                c, h, w = feat.shape
                img = image
                if img.shape[0] != h or img.shape[1] != w:
                    import cv2

                    img = cv2.resize(img, (w, h))
                obs[key] = torch.from_numpy(img).permute(2, 0, 1).float().div_(255.0)
        obs["task"] = self.task
        return obs

    def _tick(self):
        import torch

        with self._lock:
            joint_pos = None if self._joint_pos is None else self._joint_pos.copy()
            image = None if self._image is None else self._image.copy()
        if joint_pos is None or image is None:
            return
        if self._target is None:
            self._target = joint_pos.copy()
            self.get_logger().info(f"First observation received, starting control at {joint_pos.round(3)}")

        raw = self._build_observation(joint_pos, image)
        obs = self.preprocessor(raw)
        with torch.inference_mode():
            action = self.policy.select_action(obs)
        action = self.postprocessor(action).squeeze(0).float().numpy()
        self._infer_count += 1

        n = min(6, action.shape[0])
        act6 = np.zeros(6)
        act6[:n] = action[:n]
        if self._prev_action is None:
            self._prev_action = act6
            return

        delta = (act6 - self._prev_action) * self.gain
        delta = np.clip(delta, -self.max_step, self.max_step)
        self._prev_action = act6
        self._target = np.clip(self._target + delta, JOINT_LOWER + 0.05, JOINT_UPPER - 0.05)

        point = JointTrajectoryPoint()
        point.positions = self._target.tolist()
        point.time_from_start.nanosec = int(2.0 / self.control_hz * 1e9)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = [point]
        self._traj_pub.publish(traj)

        if self._infer_count % 50 == 1:
            self.get_logger().info(f"tick {self._infer_count}: target={self._target.round(3)}")


def main():
    rclpy.init()
    node = SmolVlaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
