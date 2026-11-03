# cos_ws — Elfin5 机械臂开发仓库

在华沿（Huayan）Elfin5（E05-1195_Pro，6 轴 5kg）官方 ROS2 支持包的基础上，
搭建 Gazebo 仿真 + MoveIt2 运动规划环境，并承载后续自研应用。

## 运行环境

- 主机：Ubuntu22.04 + ros2 humble

## 目录结构
采用包集合来源作为目录结构分类依据
职责-包-结点组织源码目录

```sh
.
├── app_ws  //基于厂家开发ros2包进行的应用层开发
│   ├── build_all.sh
│   └── src
│       └── cos_shape //画图形
├── document
├── elfin_ws  //elfin-robotics 华沿官方提供的ros2包
│   └── src
│       ├── elfin_demo  //厂家的一些python结点示例
│       └── elfin_robot //驱动包（机械臂驱动的底座）
├── README.md
├── scripts
│   ├── start_real.py  //启动真机的脚本
│   ├── start_sim.py   //启动gazebo模拟的脚本
└── third_party
    └── windows_sdk  //基于v6 TCP API进行开发的示例代码
        └── src
```

## 构建（顺序不能反）

```sh
~/cos_ws/elfin_ws/build_all.sh   # 1. 先构建厂家驱动层
~/cos_ws/app_ws/build_all.sh     # 2. 再构建自研应用层（自动 source elfin_ws）
```

日常使用时只需 source 最上层，elfin_ws 会被自动串接：

```sh
source ~/cos_ws/app_ws/install/setup.bash
# 或一步到位（含 ROS_LOCALHOST_ONLY=1）：
source ~/cos_ws/scripts/elfin_env.sh
```

## 快速开始（仿真）

```sh
# 1. 构建工作空间（首次或改了代码后）
~/cos_ws/elfin_ws/build_all.sh

# 2. 启动 Gazebo + MoveIt2 + RViz（容器内自动以后台进程运行）
~/cos_ws/scripts/start_sim.sh            # 日志在 ~/cos_ws/elfin_ws/log/sim/
~/cos_ws/scripts/start_sim.sh stop       # 停止后台仿真

# 3. 如需 Elfin Control Panel GUI
~/cos_ws/scripts/start_sim.sh gazebo
```

官方仿真命令（等价手动方式，见 `document/README _cn.md`）：

```sh
source ~/cos_ws/elfin_ws/install/setup.bash
ros2 launch elfin5_ros2_moveit2 elfin5.launch.py          # Gazebo + MoveIt2 + RViz
ros2 launch elfin_basic_api fake_elfin_gui.launch.py      # Control Panel（仿真用 fake 版）
```

## 真机（EtherCAT）

实机包（`soem_ros2` / `elfin_ethercat_driver` / `elfin_ros_control` / `elfin_robot_bringup`）已在工作空间内。
启动流程（4 个终端按序，需 PREEMPT_RT 内核）见 **`document/架构与数据流.md`** 真机章节，
一键脚本与诊断指令在 `scripts/legacy/`（`start_real.py`、`诊断指令.md`）。

## 文档

- **`document/架构与数据流.md`** —— 全部包/节点/topic/service 清单、仿真与真机启动命令、数据流图（Obsidian/mermaid）
- **`document/已知问题.md`** —— 坑与断链清单（launch 死代码、硬编码网卡、标定注意事项等）
- `document/README _cn.md`、`API_description.md`、`moveit_plugin_tutorial.md` —— 华沿官方文档

## 说明

- `windows_sdk/` 在 Windows 主机上选择使用MSVC、Mingw、GCC 编译运行，通过 v6 TCP
  直连控制柜，不经过 ROS，也不要放进 colcon 工作空间。
