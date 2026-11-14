#!/usr/bin/env python3
"""Validate live joint states and camera frames from the handeye simulation."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState

EXPECTED_JOINTS = tuple(f"elfin_joint{index}" for index in range(1, 7))


class HandeyeSmoke(Node):
    def __init__(self) -> None:
        super().__init__("handeye_sim_smoke")
        self.joint_summary: dict[str, object] | None = None
        self.image_summary: dict[str, object] | None = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        camera_qos = QoSProfile(depth=1)
        camera_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, "/camera/color/image_raw", self._on_image, camera_qos)

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position, strict=False))
        if not all(name in positions for name in EXPECTED_JOINTS):
            return
        values = [float(positions[name]) for name in EXPECTED_JOINTS]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Non-finite Elfin5 joint position received")
        self.joint_summary = {"joint_names": list(EXPECTED_JOINTS), "positions": values}

    def _on_image(self, message: Image) -> None:
        if message.width <= 0 or message.height <= 0:
            return
        if message.encoding != "rgb8":
            raise ValueError(f"Expected rgb8 camera input, got {message.encoding}")
        expected_size = message.width * message.height * 3
        if len(message.data) < expected_size:
            raise ValueError("Camera frame is smaller than its declared dimensions")
        self.image_summary = {
            "width": message.width,
            "height": message.height,
            "encoding": message.encoding,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = HandeyeSmoke()
    started_at = time.monotonic()
    try:
        while node.joint_summary is None or node.image_summary is None:
            if time.monotonic() - started_at > args.timeout:
                raise TimeoutError("Timed out waiting for valid joint states and camera frames")
            rclpy.spin_once(node, timeout_sec=0.2)
        result = {
            "status": "passed",
            "elapsed_sec": time.monotonic() - started_at,
            **node.joint_summary,
            "image": node.image_summary,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
