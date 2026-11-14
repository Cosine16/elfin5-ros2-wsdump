# ctl-elfin5 — Elfin5 工作站系统集成与实验仓库

本仓库负责 Elfin5 机器人工作站的系统集成、应用开发与实验验证，维护运行配置、
构建启动入口、任务流程及依赖版本。厂家软件通过子模块引入，工作站专用集成代码
由本仓库直接维护。

## 目录与职责

```text
.
├── apps/                    自研应用与集成，按能力和职责分类
│   ├── sensors/realsense/   相机接入与数据处理：cos_realsense
│   ├── robot/               自研公共机器人能力的归属位置
│   ├── bridges/mcp/         Elfin5 MCP 接入
│   ├── policies/smolvla/    SmolVLA 推理与动作适配
│   ├── workflows/           drawing、visual_following、handeye、demos
│   └── bringup/             公共启动与场景配置的归属位置
├── third_party/             厂家 fork、外部 SDK 和参考项目
│   ├── elfin_robot_ros2/    固定提交的厂家 fork 子模块
│   ├── RoboNeuron/          固定提交的 RoboNeuron 适配 fork 子模块
│   └── patches/            主仓库记录的集成构建补丁
├── scripts/                 构建、启动与诊断入口
├── document/                使用说明与架构文档
├── .build/                  vendor/apps/handeye 构建与旧产物归档（忽略）
└── outputs/                 标定结果、相机数据和实验截图（忽略）
```

详细规则和迁移映射见 [目录规范](document/repository_layout.md)；
新增代码应遵守 [AGENTS.md](AGENTS.md)。现有 ROS 包整体迁移，包名、节点和通信接口保持稳定。
当前相机/仿真组合仍随原任务包保留，公共包拆分将在后续单独验证。

## 环境与构建

当前入口使用 ROS 2 Humble。历史 Foxy / Gazebo Classic 文档保留作为参考；
Harmonic 与 Classic 使用独立构建、运行环境，不要混合 source。

Classic / 真机应用层依赖厂家 underlay，在仓库根目录执行：

```bash
git submodule update --init --recursive
bash scripts/build_vendor.sh
bash scripts/build_apps.sh
source scripts/elfin_env.sh
# zsh 使用 source scripts/elfin_env.zsh
```

自研构建从 `apps/` 发现 ROS 包，输出到 `.build/apps/`；
默认跳过 Harmonic 专用的 `elfin5_handeye`。MCP 和策略目录不参与 colcon 构建。
厂家构建仅扫描 `third_party/elfin_robot_ros2/`，输出到 `.build/vendor/`，并检查、
应用版本化的 CMake 集成补丁；详细版本、补丁和验证边界见
[厂家 fork 说明](third_party/elfin_robot_ros2.md)。

VS Code 的 [.vscode/settings.json](.vscode/settings.json) 已将 vendor 生成的
`elfin_robot_msgs` Python 安装目录加入补全和 Pylance 搜索路径，需先完成 vendor
构建。若构建后仍提示无法解析 `elfin_robot_msgs.srv`，执行
`Python: Restart Language Server`。这些设置只影响编辑器；运行脚本仍需在对应终端
加载 ROS 环境。仅使用 vendor 时，Bash 执行
`source .build/vendor/install/setup.bash`，zsh 使用 `setup.zsh`；
Classic / 真机应用使用上面的 `scripts/elfin_env.sh` 或 `scripts/elfin_env.zsh`。

Humble / Gazebo Harmonic 的相机仿真与标定：

```bash
bash scripts/build_handeye.sh
bash scripts/run_handeye.sh launch
# 另一个终端：
bash scripts/run_handeye.sh calibrate
```

该配置只构建所需四个包，输出到 `.build/handeye/`，
标定结果写入 `outputs/handeye/`。依赖、模型和验收步骤见
[Harmonic 标定说明](document/elfin5_handeye_harmonic.md)。

迁移前的构建树保留在 `.build/legacy/`，其中的缓存和安装链接含旧路径，
需要在 ROS 主机重新构建后使用新入口。此前结果只作历史证据保留。

## 仿真与真机入口

完成 Classic 应用层构建后：

```bash
bash scripts/start_sim.sh
bash scripts/start_sim.sh stop
# 仿真圆周运动示例：
bash scripts/circle_sim.sh
```

默认构建路径由脚本自身定位，仿真日志写入 `.build/apps/log/sim/`。
真机按 [架构与数据流](document/架构与数据流.md) 配置 EtherCAT、驱动参数及实时环境，
再使用 `python3 scripts/start_real.py --check` 检查。
历史启动与诊断脚本保存在 [scripts/legacy](scripts/legacy/)。

## 上位应用

- [MCP 服务](apps/bridges/mcp/README.md)：独立 Python 环境；客户端入口改为
  当前仓库的 `apps/bridges/mcp/run_server.sh` 绝对路径。
- [SmolVLA](apps/policies/smolvla/README.md)：独立模型依赖，复用 Harmonic 仿真入口。
  模型尚未在 Elfin5 上微调，当前用于链路验证。
- RoboNeuron 启动脚本支持 `ROS_SETUP` 和 `ROS_DISTRO`；本机可显式使用
  `ROS_SETUP=/opt/ros/humble/setup.bash`，无需修改 Jazzy 示例中的核心代码。
- Windows TCP SDK 示例位于 [third_party/windows_sdk](third_party/windows_sdk/)，不参与 colcon 构建。

## 文档

- [源码目录与构建规则](document/repository_layout.md)
- [包清单](document/packagelist.md)
- [架构与数据流](document/架构与数据流.md)
- [已知问题](document/已知问题.md)
- [厂家 ROS 说明](document/README_cn.md)

## Humble / Harmonic 仿真 smoke

完成 `bash scripts/build_handeye.sh` 后，可运行以下集成检查。脚本自行启动和清理
headless 仿真，检查六个关节状态、D435i RGB 图像和 `FollowJointTrajectory` action：

```bash
bash scripts/test_handeye_sim.sh
```

日志和结构化结果写入 `outputs/handeye/sim-smoke/`。同一入口由
`.github/workflows/humble-harmonic-sim.yml` 在 `main`、`feat/**` 分支推送、发往
`main` 的 PR 及手动触发时执行；CI 使用 Xvfb 提供无显示器的软件渲染环境，不运行
SmolVLA GPU 推理。

## 不依赖 ROS 的迁移检查

```bash
PYTHONPATH=apps/workflows/handeye/elfin5_handeye python3 -m pytest -q \
  tests/test_vendor_integration.py apps/workflows/handeye/elfin5_handeye/test \
  --basetemp=.build/validation/pytest-vendor
```

集成测试检查补丁、版本校验、入口路径和共享库的安装链接；CMake 检查使用占位依赖与
C++ 实现，不代替真实 ROS 构建和真机验证。
