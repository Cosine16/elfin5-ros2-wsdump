#!/usr/bin/env python3
"""检查、下载或安装 Elfin5 厂家 ROS 2 驱动依赖，仅使用 Python 标准库。"""

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys


def capture(command):
    return subprocess.run(
        command, capture_output=True, text=True, check=False,
        env={**os.environ, "LC_ALL": "C"},
    )


def system_info():
    """解析 os-release 数据，不将文件内容作为 shell 命令执行。"""
    result = {}
    for line in Path("/etc/os-release").read_text().splitlines():
        if "=" not in line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        tokens = shlex.split(value)
        result[key] = tokens[0] if tokens else ""
    return result


def package_list(distro, simulation):
    packages = [
        "build-essential", "cmake", "git", "libgtk-3-dev",
        "python3-colcon-common-extensions", "python3-rosdep",
        "python3-wxgtk4.0", "python3-transforms3d",
    ]
    suffixes = [
        "ros-base", "ament-cmake", "ament-cmake-python",
        "joint-trajectory-controller", "controller-manager", "trajectory-msgs",
        "joint-state-broadcaster", "position-controllers",
        "moveit", "xacro", "joint-state-publisher", "joint-state-publisher-gui",
        "robot-state-publisher", "rviz2", "tf2-ros", "tf2-geometry-msgs", "control-msgs",
    ]
    if simulation:
        suffixes += ["gazebo-ros-pkgs", "gazebo-ros2-control"]
    return packages + [f"ros-{distro}-{suffix}" for suffix in suffixes]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=("支持 Ubuntu 22.04/Humble、20.04/Foxy。需预先配置对应 ROS 2 APT 源。"
                "下载仅包含当前机器缺失的包及其依赖，不是完整离线包集。"
                "不包含 D435i、手眼标定或 Gazebo Harmonic。"
                "退出码：0 成功；1 缺少依赖或操作失败；2 参数或系统不支持。"),
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_const", const="check", dest="mode",
                       help="检查已安装依赖（默认，无需 sudo）")
    modes.add_argument("--download", action="store_const", const="download", dest="mode",
                       help="下载依赖，不安装")
    modes.add_argument("--install", action="store_const", const="install", dest="mode",
                       help="安装依赖")
    parser.set_defaults(mode="check")
    parser.add_argument("--download-dir", type=Path,
                        default=Path(__file__).resolve().parent.parent / ".build/downloads/elfin5",
                        help="deb 保存目录（默认仓库下 .build/downloads/elfin5，与执行目录无关）")
    parser.add_argument("--no-sim", action="store_true", help="省略 Gazebo Classic 依赖")
    parser.add_argument("--dry-run", action="store_true", help="预览命令，不修改系统")
    args = parser.parse_args()

    info = system_info()
    if info.get("ID") != "ubuntu":
        parser.error(f"仅支持 Ubuntu，当前为 {info.get('PRETTY_NAME', '未知系统')}")
    distro = {"22.04": "humble", "20.04": "foxy"}.get(info.get("VERSION_ID", ""))
    if distro is None:
        parser.error(f"Ubuntu {info.get('VERSION_ID', '未知')} 未经本仓库验证")
    if distro == "foxy":
        print("警告: Foxy 已停止维护，推荐 Ubuntu 22.04 + Humble。", file=sys.stderr)
    active_distro = os.environ.get("ROS_DISTRO")
    if active_distro and active_distro != distro:
        parser.error(f"当前 ROS_DISTRO={active_distro}，应为 {distro}，请在干净终端运行")
    for command in ("dpkg-query", "dpkg", "apt-get", "apt-cache"):
        if shutil.which(command) is None:
            raise RuntimeError(f"缺少命令: {command}")
    architecture = capture(["dpkg", "--print-architecture"])
    architecture.check_returncode()
    print(f"系统: {info.get('PRETTY_NAME')} | 架构: {architecture.stdout.strip()} | ROS 2: {distro}")

    packages = package_list(distro, not args.no_sim)
    missing = []
    for package in packages:
        status = capture(["dpkg-query", "-W", "-f=${Status}", package])
        installed = status.returncode == 0 and status.stdout.strip() == "install ok installed"
        print(f"[{'已安装' if installed else '缺失'}] {package}")
        if not installed:
            missing.append(package)
    print(f"\n共 {len(packages)} 项，缺失 {len(missing)} 项。")
    if args.mode == "check":
        return int(bool(missing))
    if not missing:
        print("依赖已齐全，无需下载或安装。")
        return 0

    privilege = [] if os.geteuid() == 0 else ["sudo"]

    def run(command):
        if args.dry_run:
            print("将执行: " + shlex.join(command))
        else:
            sys.stdout.flush()
            subprocess.run(command, check=True)

    if not args.dry_run and privilege and shutil.which("sudo") is None:
        raise RuntimeError("需要 root 权限，但未找到 sudo")
    apt = ["apt-get", "-o", "Acquire::Retries=3"]
    run(privilege + apt + ["update"])
    if not args.dry_run:
        unavailable = []
        for package in missing:
            policy = capture(["apt-cache", "policy", package])
            candidate = re.search(r"^\s*Candidate:\s*(\S+)", policy.stdout, re.MULTILINE)
            if policy.returncode != 0 or candidate is None or candidate[1] == "(none)":
                unavailable.append(package)
        if unavailable:
            raise RuntimeError(
                "APT 源无可用候选包: " + ", ".join(unavailable)
                + f"。可能是包名与 {distro} 不兼容，或源未提供该包；"
                "不能仅据此判断网络或软件源配置错误。可用 apt-cache policy <包名> 核实。"
            )
    if args.mode == "download":
        directory = args.download_dir.expanduser().resolve()
        run(privilege + ["mkdir", "-p", str(directory / "partial")])
        run(privilege + apt + ["-y", "--download-only", "-o",
                              f"Dir::Cache::archives={directory}", "install"] + missing)
        print(f"{'计划下载目录' if args.dry_run else 'deb 下载目录'}: {directory}")
    else:
        run(privilege + apt + ["-y", "install"] + missing)
        print("依赖安装完成。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"错误: {error}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n已取消。", file=sys.stderr)
        sys.exit(130)
