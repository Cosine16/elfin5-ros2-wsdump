#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""start_real.py — 按 README「使用真实的Elfin机器人」启动 Elfin 实机的多终端启动器。

实机启动需要 4 个进程, 严格按依赖顺序排队 (前一步就绪后才启动下一步):

    1. 硬件驱动    ros2 launch <model>_ros2_moveit2 <model>_moveit.launch.py
    2. MoveIt+RViz ros2 launch <model>_ros2_moveit2 <model>_moveit_rviz.launch.py
    3. basic_api  ros2 launch <model>_ros2_moveit2 <model>_basic_api.launch.py
    4. 控制面板    ros2 launch elfin_basic_api elfin_gui.launch.py

步骤 2/3/4 通过 Gate 同步: 在各自终端里轮询等待上游的 service/node 出现
(见 wait_for_ros.sh), 因此终端可以任意时刻手动重启, 不必按顺序重开全部。

环境层: 每个终端按 humble -> .build/vendor -> .build/apps 的顺序显式 source 三层
setup.bash。不依赖 .build/apps 构建时的"自动串接" —— 串接关系是构建时刻的快照,
.build/apps 一旦脱离 .build/vendor 单独构建, 其 setup.bash 就断了驱动层, 终端启动即
'Package not found'。显式 source 让运行期环境不依赖构建历史。

用户模型: 默认全程以当前用户运行。硬件终端经 `sudo capsh` 持有
cap_net_raw/cap_sys_nice —— EtherCAT 裸套接字与实时调度确实需要 root
权限, 但进程属主仍是普通用户。全程同用户, 避免了两类经典跨用户坑:
  - /dev/shm/fastrtps_* 被 root 节点建成 0644, 普通用户订阅静默不通;
  - root 写进 ~/.ros 的日志/缓存变 root 所有, 之后用户启动报权限错。
--sudo 为旧行为(全 root), 仅调试时临时使用。

用法:
    python3 start_real.py                 # 默认 elfin5, 用户态 (推荐)
    python3 start_real.py --elfin elfin10
    python3 start_real.py --check         # 只检查环境, 不启动
    python3 start_real.py --list          # 只打印命令, 不启动
    python3 start_real.py --sudo          # 旧行为: 全部终端 root
    python3 start_real.py --no-wait       # 不做依赖等待
"""

from __future__ import annotations

import argparse
import getpass
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# 路径 (全部相对本脚本定位, 不依赖 ~/cos_ws 之类的硬编码布局)
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)                      # cos_ws/
APP_WS = os.environ.get("APP_BUILD_DIR", os.path.join(REPO_ROOT, ".build", "apps"))
DRIVER_WS = os.environ.get("VENDOR_BUILD_DIR", os.path.join(REPO_ROOT, ".build", "vendor"))
VENDOR_SRC = os.path.join(REPO_ROOT, "third_party", "elfin_robot_ros2")
BRINGUP_CONFIG = os.path.join(
    VENDOR_SRC, "elfin_robot_bringup", "config"
)
WAIT_HELPER = os.path.join(SCRIPT_DIR, "wait_for_ros.sh")

# 终端内按 humble -> .build/vendor -> .build/apps 顺序显式 source (后者覆盖前者,
# 见模块 docstring 环境层); env_prelude() 直接使用这三个路径
HUMBLE_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
DRIVER_SETUP = os.path.join(DRIVER_WS, "install", "setup.bash")
APP_SETUP = os.path.join(APP_WS, "install", "setup.bash")

# 环境链解析后必须存在的驱动包, 由 check_workspace_chain() 动态校验
REQUIRED_PACKAGES = ("elfin5_ros2_moveit2", "elfin5_ros2_gazebo",
                     "elfin_basic_api", "elfin_robot_msgs")

# 手头只有 elfin5 一台机械臂, 驱动已裁剪为 elfin5-only (见 submodule 的 humble-station 分支)
MODELS = ("elfin5",)

# 终端模拟器, 按优先级选用
TERMINALS = ("gnome-terminal", "konsole", "xterm", "xfce4-terminal", "tilix")


# ---------------------------------------------------------------------------
# 同步与流水线声明
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Gate:
    """进程间同步: launch 前等待某个 ROS 实体出现。

    kind 为 "service" 或 "node", pattern 在 `ros2 <kind> list` 输出中匹配。
    """

    label: str
    kind: str
    pattern: str


@dataclass(frozen=True)
class Step:
    """流水线中的一步: 一个终端里执行的一条 ros2 launch。"""

    title: str
    launch: str          # "ros2 launch" 的参数部分, 如 "elfin5_ros2_moveit2 elfin5_moveit.launch.py"
    hardware: bool = False   # 是否硬件驱动 (决定特权包装方式)
    gate: Gate | None = None  # 上游就绪门; None 表示立即启动


def build_pipeline(model: str) -> list[Step]:
    """展开 README 真机章节的 4 条命令 (机型只影响前 3 条的包名/文件名前缀)。"""
    pkg = f"{model}_ros2_moveit2"
    return [
        Step(
            title=f"{model} 硬件驱动 (EtherCAT)",
            launch=f"{pkg} {model}_moveit.launch.py",
            hardware=True,
        ),
        Step(
            title="MoveIt! + RViz",
            launch=f"{pkg} {model}_moveit_rviz.launch.py",
            gate=Gate("controller_manager", "service",
                      "/controller_manager/list_controllers"),
        ),
        Step(
            title="后台程序 basic_api",
            launch=f"{pkg} {model}_basic_api.launch.py",
            gate=Gate("move_group", "node", "move_group"),
        ),
        Step(
            title="Elfin Control Panel",
            launch="elfin_basic_api elfin_gui.launch.py",
            gate=Gate("basic_api", "service", "/get_reference_link"),
        ),
    ]


# ---------------------------------------------------------------------------
# 命令组装
# ---------------------------------------------------------------------------
def env_prelude() -> list[str]:
    """每个终端启动时的环境链: DDS 配置 -> 按序 source 三层 underlay。"""
    # DDS 只走回环: 实机网卡被 EtherCAT 占用, 组播发现会超时导致节点互不可见。
    # 终端以 bash -c 非交互运行, 不读 ~/.bashrc, 必须显式导出。
    exports = ["export ROS_LOCALHOST_ONLY=1"]
    stack = (HUMBLE_SETUP, DRIVER_SETUP, APP_SETUP)
    sources = [f"source {shlex.quote(s)}" for s in stack]
    return exports + sources + [f"cd {shlex.quote(APP_WS)}"]


def payload(step: Step, wait_timeout: int, no_wait: bool) -> str:
    """单条终端命令体: 环境准备 -> (等待上游就绪) -> ros2 launch。"""
    parts = env_prelude()
    if not no_wait and step.gate:
        g = step.gate
        parts.append(
            f"bash {shlex.quote(WAIT_HELPER)} {shlex.quote(g.label)} "
            f"{g.kind} {shlex.quote(g.pattern)} {wait_timeout}"
        )
    parts.append(f"ros2 launch {step.launch}")
    return " && ".join(parts)


def wrap_privilege(cmd: str, step: Step, use_sudo: bool) -> str:
    """决定进程以什么身份运行 (见模块 docstring 的用户模型说明)。"""
    if not step.hardware:
        # 非硬件终端永远不需要 root
        return f"sudo -E bash -c {shlex.quote(cmd)}" if use_sudo else cmd

    if use_sudo:
        # 旧行为: chrt 提实时优先级, 全程 root
        return f"sudo chrt 10 bash -c {shlex.quote(cmd)}"

    # 默认: sudo 只为拿到 capability, 进程仍以当前用户运行。
    # 不能用 setcap 给二进制授权 —— 带 file caps 的进程会被 glibc 置为
    # secure-execution 模式并清掉 LD_LIBRARY_PATH, 而 rcpputils
    # 的 find_library_path 依赖该变量查找 rmw 实现, 加载会失败。
    user = getpass.getuser()
    inner = f"export HOME={shlex.quote(os.path.expanduser('~'))}; {cmd}"
    return (
        f"sudo capsh --keep=1 --user={shlex.quote(user)} "
        f"--inh=cap_net_raw,cap_sys_nice "
        f"--addamb=cap_net_raw,cap_sys_nice -- -c {shlex.quote(inner)}"
    )


def terminal_command(step: Step, wait_timeout: int, no_wait: bool,
                     use_sudo: bool) -> str:
    """终端里的完整命令: 命令体 -> 退出后保持窗口, 方便看日志。"""
    body = wrap_privilege(payload(step, wait_timeout, no_wait), step, use_sudo)
    hold = "echo; echo '=== 进程已退出, 按任意键关闭本终端 ==='; read -n1"
    return f"{body} && {hold}"


def open_terminal(title: str, command: str) -> None:
    """在可用的终端模拟器中新开窗口并执行命令。"""
    argv_builders = {
        "gnome-terminal": lambda: ["gnome-terminal", "--title", title, "--",
                                   "bash", "-c", command],
        "konsole": lambda: ["konsole", "--new-tab", "-p", f"tabtitle={title}",
                            "-e", "bash", "-c", command],
        "xterm": lambda: ["xterm", "-T", title,
                          "-e", f"bash -c {shlex.quote(command)}"],
    }
    for name in TERMINALS:
        if not shutil.which(name):
            continue
        argv = argv_builders.get(
            name,
            lambda: [name, "--title", title,
                     "-e", f"bash -c {shlex.quote(command)}"],
        )()
        subprocess.Popen(argv)
        return
    sys.exit("错误: 未找到终端模拟器, 请先安装 gnome-terminal/xterm 之一。")


# ---------------------------------------------------------------------------
# 环境检查
# ---------------------------------------------------------------------------
def get_ethercat_interface() -> str | None:
    """从 elfin_arm_control.yaml 读取 elfin_ethernet_name。"""
    yaml_path = os.path.join(BRINGUP_CONFIG, "elfin_arm_control.yaml")
    try:
        import yaml
    except ImportError:
        return None
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        return (data or {}).get("/**", {}).get("ros__parameters", {}).get(
            "elfin_ethernet_name"
        )
    except OSError:
        return None


def interface_is_up(name: str) -> bool:
    try:
        out = subprocess.check_output(
            ["ip", "-o", "link", "show", name],
            stderr=subprocess.DEVNULL, text=True,
        )
        return "LOWER_UP" in out or "state UP" in out
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def check_workspace_chain() -> tuple[bool, list[str]]:
    """动态校验: 实际按 humble->.build/vendor->.build/apps 链 source 一遍, 驱动包可解析。

    静态文件检查之外的保险 —— .build/apps 若脱离 .build/vendor 构建, 其 setup.bash
    不含驱动层, 只查文件存在性是发现不了的, 终端启动才报 'Package not found'。
    返回 (是否齐全, 缺失包列表)。
    """
    probe = " && ".join(env_prelude() + ["ros2 pkg list 2>/dev/null"])
    try:
        out = subprocess.run(["bash", "-c", probe], capture_output=True,
                             text=True, timeout=90)
    except subprocess.TimeoutExpired:
        return False, list(REQUIRED_PACKAGES)
    found = set(out.stdout.split())
    missing = [p for p in REQUIRED_PACKAGES if p not in found]
    return not missing, missing


def check_environment() -> bool:
    """实机启动前的预检, 返回是否可继续 (实时内核仅警告不阻断)。"""
    ok = True

    def report(good: bool, message: str) -> None:
        nonlocal ok
        print(("[OK] " if good else "[!!] ") + message)
        ok = ok and good

    report(os.path.isfile(DRIVER_SETUP),
           f"厂家驱动 setup: {DRIVER_SETUP}" if os.path.isfile(DRIVER_SETUP)
           else f"未找到 {DRIVER_SETUP}, 请先运行 bash scripts/build_vendor.sh")

    report(os.path.isfile(APP_SETUP),
           f"应用层 setup: {APP_SETUP}" if os.path.isfile(APP_SETUP)
           else f"未找到 {APP_SETUP}, 请先运行 bash scripts/build_apps.sh")

    drivers_yaml = os.path.join(BRINGUP_CONFIG, "elfin_drivers.yaml")
    report(os.path.isfile(drivers_yaml),
           f"驱动参数文件: {drivers_yaml}" if os.path.isfile(drivers_yaml)
           else f"未找到 {drivers_yaml} (把购买时得到的 elfin_drivers.yaml 放到该目录)")

    iface = get_ethercat_interface()
    if iface is None:
        report(False, "无法从 elfin_arm_control.yaml 读取 elfin_ethernet_name"
                      " (缺少 python3-yaml? 或文件不存在)")
    elif interface_is_up(iface):
        report(True, f"EtherCAT 网卡 {iface}: 已启用")
    else:
        report(False, f"EtherCAT 网卡 {iface} 不存在或未启用, 请修改 "
                      f"elfin_arm_control.yaml 的 elfin_ethernet_name")

    release = os.uname().release
    if "rt" not in release.lower():
        print(f"[!!] 当前内核 {release} 可能不是 PREEMPT_RT 实时内核, "
              f"实机控制需要实时性支持 (仅提示, 不阻断)")

    report(shutil.which("capsh") is not None,
           "capsh 可用 (硬件终端将以普通用户 + ambient capabilities 运行)"
           if shutil.which("capsh")
           else "未找到 capsh (sudo apt install libcap2-bin), 或改用 --sudo")

    chain_ok, missing = check_workspace_chain()
    report(chain_ok,
           "环境链动态校验: 驱动包可解析 (" + ", ".join(REQUIRED_PACKAGES) + ")"
           if chain_ok
           else "环境链 source 后仍找不到驱动包: " + ", ".join(missing)
                + " —— 多半是某层工作空间脱离下层构建, 重跑对应 build_all.sh")

    return ok


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main() -> None:
    global APP_WS, APP_SETUP

    parser = argparse.ArgumentParser(
        description="按 README 真机章节启动 Elfin 实机的 4 个终端 "
                    "(硬件 / MoveIt+RViz / basic_api / 控制面板)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 start_real.py              # elfin5, 用户态\n"
               "  python3 start_real.py --elfin elfin10 --check\n"
               "  python3 start_real.py --list\n"
               "  python3 start_real.py --sudo       # 旧行为: 全 root\n",
    )
    parser.add_argument("--elfin", default="elfin5", choices=MODELS,
                        help="机器人机型前缀, 默认 elfin5")
    parser.add_argument("--sudo", action="store_true",
                        help="全部终端以 root 运行 (旧行为)。默认用户态: 硬件终端经 "
                             "capsh 拿 capability, 避免跨用户 DDS/文件权限问题")
    parser.add_argument("--check", action="store_true",
                        help="只检查环境, 不启动终端")
    parser.add_argument("--list", action="store_true",
                        help="只打印将要执行的命令, 不启动终端")
    parser.add_argument("--workspace", default=APP_WS,
                        help="overlay 工作空间路径, 默认 <repo>/.build/apps")
    parser.add_argument("--wait-timeout", type=int, default=120,
                        help="每步等待上游就绪的超时秒数, 默认 120")
    parser.add_argument("--no-wait", action="store_true",
                        help="不做依赖等待, 直接按顺序启动")
    args = parser.parse_args()

    APP_WS = os.path.abspath(os.path.expanduser(args.workspace))
    APP_SETUP = os.path.join(APP_WS, "install", "setup.bash")

    print("=" * 70)
    print(f"Elfin 实机启动器  机型: {args.elfin}  工作空间: {APP_WS}  "
          f"特权: {'sudo' if args.sudo else '用户态(capsh)'}")

    print("--- 环境检查 ---")
    env_ok = check_environment()
    print(f"环境检查: {'通过' if env_ok else '存在问题 (见 [!!], 可自行判断继续)'}")
    if args.check:
        sys.exit(0 if env_ok else 1)

    if args.sudo:
        print("\n[警告] --sudo 全 root 模式的两类跨用户坑:")
        print("  1. /dev/shm/fastrtps_* 将属 root(0644), 普通用户节点订阅静默不通;")
        print("  2. root 写入 ~/.ros 的文件变 root 所有, 之后用户启动报权限错。")
        print("  全部节点退出后如异常: sudo rm -f /dev/shm/fastrtps_*; "
              "sudo chown -R $USER:$USER ~/.ros")

    steps = build_pipeline(args.elfin)
    wait_mode = "关" if args.no_wait else f"开 (超时 {args.wait_timeout}s)"
    print(f"依赖等待: {wait_mode}\n将启动 {len(steps)} 个终端:\n")

    for idx, step in enumerate(steps, start=1):
        full = terminal_command(step, args.wait_timeout, args.no_wait, args.sudo)
        gate_info = f"  (先等 {step.gate.label} 就绪)" if step.gate and not args.no_wait else ""
        print("=" * 70)
        print(f"[{idx}] {step.title}{gate_info}")
        print(full)
        if not args.list:
            open_terminal(step.title, full)

    print("\n" + "=" * 70)
    if args.list:
        print("以上为将要执行的命令 (--list 模式)。")
    else:
        print("各终端按依赖顺序自行等待就绪。启动完成后: 控制面板先 'Clear Fault' "
              "再 'Servo On'; 关闭电源前先 'Servo Off'。")


if __name__ == "__main__":
    main()
