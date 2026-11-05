#!/usr/bin/env python3
"""Bootstrap the SmolVLA-on-Elfin5 demo (Gz Harmonic + D435i simulation).

Covers everything that was done manually to get the demo running:
  1. apt dependencies for Gz Harmonic + ros_gz (no source compilation);
  2. a Python venv with torch (CUDA build compatible with older GPUs),
     lerobot, transformers and the other runtime libraries;
  3. the lerobot/smolvla_base weights from Hugging Face;
  4. a check for the separately built Harmonic simulation profile;
  5. an optional GPU inference smoke test.

The script is idempotent: finished steps are skipped on re-runs.
No sudo password is stored anywhere; apt steps prompt interactively.

Usage:
    python3 apps/policies/smolvla/setup_smolvla.py [--skip-test]

After setup, run the demo (4 terminals):
    bash apps/policies/smolvla/run_gz_sim.sh                 # Gz + D435i simulation
    apps/policies/smolvla/.venv/bin/python apps/policies/smolvla/look_at_board.py --ros-args -p use_sim_time:=true
    ROS_DOMAIN_ID=85 bash apps/policies/smolvla/run_bridge.sh -p gain:=0.05
    bash apps/policies/smolvla/run_rviz.sh                   # optional visualization
(source scripts/handeye_env.sh before running look_at_board.py.)
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

STATION_ROOT = Path(__file__).resolve().parents[3]
VENV = Path(__file__).resolve().parent / ".venv"
HANDEYE_WS = Path(os.environ.get("HANDEYE_WS", STATION_ROOT / ".build" / "handeye"))
MODEL_ID = "lerobot/smolvla_base"

APT_PACKAGES = [
    "gz-sim8-cli",
    "ros-humble-ros-gz-sim",
    "ros-humble-ros-gz-bridge",
    "ros-humble-gz-ros2-control",
]

# lerobot 0.4.4 requires torch>=2.2.1, torchvision>=0.21 and (for SmolVLA)
# transformers>=4.57.1,<5. torch 2.6.0+cu124 ships sm_50/60 cubins, which also
# run on Pascal GPUs (e.g. the GTX 1050 Ti, sm_61); newer default wheels
# (cu128) do not. cu121 has no torch 2.6.x, hence the cu124 index.
TORCH_INDEX = "https://download.pytorch.org/whl/cu124"
TORCH_PACKAGES = ["torch==2.6.0", "torchvision==0.21.0"]
PYTHON_PACKAGES = [
    "lerobot==0.4.4",
    "transformers>=4.57.1,<5.0.0",
    "num2words",
    "scipy",
    "opencv-python-headless",
]


def log(message):
    print(f"\n=== {message} ===", flush=True)


def run(cmd, **kwargs):
    print("+ " + " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def is_installed(package):
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${Status}", package],
        capture_output=True, text=True,
    )
    return "ok installed" in result.stdout


def step_apt():
    missing = [p for p in APT_PACKAGES if not is_installed(p)]
    if not missing:
        log("apt dependencies already installed, skipping")
        return
    log(f"Installing apt packages (sudo may prompt for password): {missing}")
    run(["sudo", "apt-get", "install", "-y", "--no-install-recommends", *missing])


def step_venv():
    if not (VENV / "bin" / "python").exists():
        log(f"Creating venv at {VENV} (system-site-packages: rclpy access)")
        if shutil.which("uv"):
            run(["uv", "venv", "--system-site-packages", VENV])
        else:
            # Requires python3-venv (ensurepip); uv is the tested path.
            run([sys.executable, "-m", "venv", "--system-site-packages", VENV])
    else:
        log("venv already exists, skipping creation")

    log("Installing Python libraries (lerobot first, torch pinned afterwards)")
    run(["uv", "pip", "install", "--python", VENV / "bin" / "python", *PYTHON_PACKAGES])
    # Installing lerobot alone would pull the newest torch (cu128, no Pascal
    # support). Re-pinning torch/torchvision afterwards fixes that and is a
    # no-op when the correct versions are already present.
    run(["uv", "pip", "install", "--python", VENV / "bin" / "python",
         *TORCH_PACKAGES, "--index-url", TORCH_INDEX])


def step_model():
    log(f"Downloading {MODEL_ID} (cached after first download)")
    code = (
        "from huggingface_hub import snapshot_download;"
        f"print(snapshot_download('{MODEL_ID}'))"
    )
    run([VENV / "bin" / "python", "-c", code])


def step_handeye():
    if not (HANDEYE_WS / "install" / "setup.bash").is_file():
        raise RuntimeError(
            f"Simulation profile missing at {HANDEYE_WS}. "
            f"Run bash {STATION_ROOT / 'scripts' / 'build_handeye.sh'} first. "
            "Do not copy an install tree from another workspace."
        )
    log(f"Using simulation profile: {HANDEYE_WS}")


def step_test():
    log("GPU inference smoke test (loads SmolVLA, runs one action chunk)")
    test_code = r"""
import time
import numpy as np
import torch
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.factory import make_pre_post_processors

device = "cuda" if torch.cuda.is_available() else "cpu"
print("torch", torch.__version__, "device", device)
if device == "cuda":
    print("GPU:", torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0))
policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base")
policy.config.device = device
policy.to(device).eval()
pre, post = make_pre_post_processors(policy_cfg=policy.config, pretrained_path="lerobot/smolvla_base")
obs = {
    "observation.state": torch.zeros(6),
    "observation.images.camera1": torch.rand(3, 256, 256),
    "observation.images.camera2": torch.rand(3, 256, 256),
    "observation.images.camera3": torch.rand(3, 256, 256),
    "task": "pick up the cube",
}
start = time.time()
action = post(policy.select_action(pre(obs)))
print(f"OK: action {action.squeeze(0).numpy().round(4)} in {time.time() - start:.2f}s")
"""
    run([VENV / "bin" / "python", "-c", test_code])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skip-test", action="store_true", help="skip the GPU inference smoke test")
    args = parser.parse_args()

    if not Path("/opt/ros/humble/setup.bash").exists():
        raise RuntimeError("ROS 2 Humble is required at /opt/ros/humble")
    if not shutil.which("uv"):
        raise RuntimeError("uv is required (https://docs.astral.sh/uv/)")

    step_handeye()
    step_apt()
    step_venv()
    step_model()
    if not args.skip_test:
        step_test()

    log("Setup complete. Run the demo with:")
    print("""\
  bash apps/policies/smolvla/run_gz_sim.sh
  ROS_DOMAIN_ID=85 bash apps/policies/smolvla/run_bridge.sh -p gain:=0.05
  bash apps/policies/smolvla/run_rviz.sh
Optional (aim the wrist camera at the ArUco board):
  source scripts/handeye_env.sh
  apps/policies/smolvla/.venv/bin/python apps/policies/smolvla/look_at_board.py --ros-args -p use_sim_time:=true
""")


if __name__ == "__main__":
    main()
