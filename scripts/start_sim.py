#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_sim.py
============
根据 README_cn.md 中的仿真章节命令，自动打开多个终端窗口，
并在每个终端中依次执行:
    1. source ~/cos_ws/.build/apps/install/setup.bash  (overlay, 自动串接 .build/vendor)
    2. sudo + ros2 launch <仿真命令>

支持机型(前缀): elfin3 / elfin5 / elfin5_l / elfin10 / elfin10_l / elfin15

用法:
    python3 start_sim.py                      # 默认: 启动 Elfin5 仿真, 使用 sudo
    python3 start_sim.py --no-sudo            # 不使用 sudo
    python3 start_sim.py --elfin elfin5       # 启动 Elfin5 仿真
    python3 start_sim.py --list               # 只打印将要执行的命令, 不启动终端
    python3 start_sim.py --workspace ~/cos_ws/.build/apps   # 指定工作空间路径

注意:
    - 使用 sudo 时, 每个终端会提示输入密码, 请在对应终端中手动输入。
    - 终端在命令结束后会保持打开, 方便查看日志, 按任意键后关闭。
"""

import argparse
import os
import shlex
import shutil
import subprocess
import sys

# ---------------------------------------------------------------------------
# 可配置项
# ---------------------------------------------------------------------------
CATKIN_WS = os.environ.get("APP_BUILD_DIR", os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".build", "apps"))
SETUP_SCRIPT = os.path.join(CATKIN_WS, "install", "setup.bash")

SIM_COMMANDS = {
    "elfin3": [
        ("Gazebo + MoveIt (RViz)", "elfin3_ros2_moveit2 elfin3.launch.py"),
        ("后台程序 basic_api", "elfin3_ros2_moveit2 elfin3_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
    "elfin5": [
        ("Gazebo + MoveIt (RViz)", "elfin5_ros2_moveit2 elfin5.launch.py"),
        ("后台程序 basic_api", "elfin5_ros2_moveit2 elfin5_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
    "elfin5_l": [
        ("Gazebo + MoveIt (RViz)", "elfin5_l_ros2_moveit2 elfin5_l.launch.py"),
        ("后台程序 basic_api", "elfin5_l_ros2_moveit2 elfin5_l_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
    "elfin10": [
        ("Gazebo + MoveIt (RViz)", "elfin10_ros2_moveit2 elfin10.launch.py"),
        ("后台程序 basic_api", "elfin10_ros2_moveit2 elfin10_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
    "elfin10_l": [
        ("Gazebo + MoveIt (RViz)", "elfin10_l_ros2_moveit2 elfin10_l.launch.py"),
        ("后台程序 basic_api", "elfin10_l_ros2_moveit2 elfin10_l_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
    "elfin15": [
        ("Gazebo + MoveIt (RViz)", "elfin15_ros2_moveit2 elfin15.launch.py"),
        ("后台程序 basic_api", "elfin15_ros2_moveit2 elfin15_basic_api.launch.py"),
        ("Elfin Control Panel GUI", "elfin_basic_api fake_elfin_gui.launch.py"),
    ],
}

# 常用的终端模拟器, 按优先级选择 (Ubuntu 20.04 默认 gnome-terminal)
TERMINALS = ["gnome-terminal", "konsole", "xterm", "xfce4-terminal", "tilix"]


# ---------------------------------------------------------------------------
# 构建命令
# ---------------------------------------------------------------------------
def build_launch_command(launch_args, use_sudo):
    """构建执行 ros2 launch 的命令, 可选择是否使用 sudo。"""
    if use_sudo:
        # sudo 会清空环境变量, 所以要在 sudo 内部重新 source 并 cd
        inner = "{} && cd {} && ros2 launch {}".format(
            "source {}".format(shlex.quote(SETUP_SCRIPT)),
            shlex.quote(CATKIN_WS),
            launch_args,
        )
        # -E 保留 DISPLAY 等环境变量, 便于 root 启动 RViz/Gazebo 图形界面
        return "sudo -E bash -c {}".format(shlex.quote(inner))
    return "ros2 launch {}".format(launch_args)


def build_terminal_command(launch_args, use_sudo):
    """构建终端内执行的完整命令: source -> sudo/ros2 launch -> 保持打开。"""
    parts = [
        # DDS 只走回环: enp2s0 常处于断网/EtherCAT 状态, 组播发现会超时。
        # (终端以 bash -c 非交互方式运行, 不读 ~/.bashrc, 必须显式导出)
        "export ROS_LOCALHOST_ONLY=1",
        "cd {}".format(shlex.quote(CATKIN_WS)),
        "source {}".format(shlex.quote(SETUP_SCRIPT)),
        build_launch_command(launch_args, use_sudo),
        # 命令结束后保持终端打开, 按任意键再关闭
        "echo",
        "echo '=== 进程已退出, 按任意键关闭本终端 ==='",
        "read -n1",
    ]
    return " && ".join(parts)


# ---------------------------------------------------------------------------
# 打开终端
# ---------------------------------------------------------------------------
def open_terminal(title, full_command):
    """在系统默认终端模拟器中新开一个终端并执行命令。"""
    terminal = shutil.which("gnome-terminal")
    if terminal:
        subprocess.Popen(
            [terminal, "--title", title, "--", "bash", "-c", full_command]
        )
        return
    terminal = shutil.which("konsole")
    if terminal:
        subprocess.Popen(
            [terminal, "--new-tab", "-p", "tabtitle={}".format(title),
             "-e", "bash", "-c", full_command]
        )
        return
    terminal = shutil.which("xterm")
    if terminal:
        subprocess.Popen(
            [terminal, "-T", title, "-e", "bash -c {}".format(shlex.quote(full_command))]
        )
        return
    sys.exit("错误: 未找到可用的终端模拟器 (gnome-terminal/konsole/xterm), 请先安装其中一个。")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    global CATKIN_WS, SETUP_SCRIPT

    parser = argparse.ArgumentParser(
        description="根据 README_cn.md 自动启动 Elfin 机器人仿真终端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 start_sim.py\n"
               "  python3 start_sim.py --no-sudo\n"
               "  python3 start_sim.py --elfin elfin5\n"
               "  python3 start_sim.py --list",
    )
    parser.add_argument("--elfin", default="elfin5",
                        choices=list(SIM_COMMANDS.keys()),
                        help="机器人机型前缀, 默认 elfin5")
    parser.add_argument("--no-sudo", action="store_true",
                        help="不使用 sudo 执行仿真命令")
    parser.add_argument("--list", action="store_true",
                        help="只打印将要执行的命令, 不启动终端")
    parser.add_argument("--workspace", default=CATKIN_WS,
                        help="工作空间路径, 默认 <repo>/.build/apps")
    args = parser.parse_args()

    CATKIN_WS = os.path.expanduser(args.workspace)
    SETUP_SCRIPT = os.path.join(CATKIN_WS, "install", "setup.bash")

    # 检查工作空间是否已经编译
    if not os.path.isfile(SETUP_SCRIPT):
        sys.exit("错误: 未找到 {} 。\n"
                 "请先确认已经用 colcon build 编译过工作空间, "
                 "或使用 --workspace 指定正确路径。".format(SETUP_SCRIPT))

    use_sudo = not args.no_sudo
    commands = SIM_COMMANDS[args.elfin]

    print("工作空间: {}".format(CATKIN_WS))
    print("机型    : {}".format(args.elfin))
    print("使用sudo: {}".format("是" if use_sudo else "否"))
    print("将启动 {} 个终端:\n".format(len(commands)))

    for title, launch_args in commands:
        full = build_terminal_command(launch_args, use_sudo)
        print("=" * 70)
        print("[{}]".format(title))
        print(full)
        if not args.list:
            open_terminal(title, full)

    print("\n" + "=" * 70)
    if args.list:
        print("以上为将要执行的命令 (--list 模式, 未启动终端)。")
    else:
        print("所有终端已启动。若使用 sudo, 请在对应终端中手动输入密码。")


if __name__ == "__main__":
    main()
