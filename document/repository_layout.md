# 源码归属与构建目录规范

自研源码统一放到 `apps/`，按能力与职责分类。源码归属、运行调用关系、部署主机和
构建环境是不同维度；不因为某个模块需要 ROS 或独立 Python 环境就新增根级源码工作空间。

## 分类边界

- `sensors`：设备接入、传感器模型、配置和数据获取。
- `robot`：自研公共机械臂能力；厂家实现由 `third_party/elfin_robot_ros2` 子模块引入。
- `bridges`：协议及接口转换；MCP 不承载模型策略与标定算法。
- `policies`：推理、观测预处理和模型动作适配。
- `workflows`：画图、视觉跟随、标定等跨能力任务。
- `bringup`：公共组合启动与运行配置，按实际复用逐步提取。
- `scripts`：仓库构建、诊断和维护入口；模块专用辅助函数跟随所属模块。

## 本次迁移映射

| 原位置 | 新位置 |
| --- | --- |
| `app_ws/src/cos_realsense` | `apps/sensors/realsense/cos_realsense` |
| `app_ws/src/cos_shape` | `apps/workflows/drawing/cos_shape` |
| `app_ws/src/cos_rsvisual` | `apps/workflows/visual_following/cos_rsvisual` |
| `app_ws/src/cv-elfin5-d435i-handeye` | `apps/workflows/handeye/elfin5_handeye` |
| `elfin5_mcp` | `apps/bridges/mcp` |
| `smolvla_ws` | `apps/policies/smolvla` |
| `app_ws/build_all.sh` | `scripts/build_apps.sh` |
| `handeye_results` | `outputs/handeye` |
| SmolVLA 测试截图 | `outputs/smolvla` |
| `elfin_ws/src/elfin_robot` | `third_party/elfin_robot_ros2`，保留 Git 子模块 |
| `elfin_ws/src/elfin_demo` | `apps/workflows/demos/elfin_demo` |
| `elfin_ws/build_all.sh` | `scripts/build_vendor.sh` |
| `elfin_ws` 中原构建、缓存和日志 | `.build/legacy/elfin_ws` |

保留 ROS 包名、Python 模块名、节点和 topic/service/action 名称。
现有任务包内部的模型、相机和启动内容仍整体保留；本次不同时拆包。

## 构建与运行

| 配置 | 构建入口 | 产物位置 | 环境入口 |
| --- | --- | --- | --- |
| 厂家 underlay | `scripts/build_vendor.sh` | `.build/vendor/{build,install,log}` | `.build/vendor/install/setup.bash` |
| Classic / 真机应用 overlay | `scripts/build_apps.sh` | `.build/apps/{build,install,log}` | `scripts/elfin_env.sh` 或 `.zsh` |
| Humble / Harmonic 标定仿真 | `scripts/build_handeye.sh` | `.build/handeye/{build,install,log}` | Bash 中 source `scripts/handeye_env.sh` |

默认应用构建跳过 Harmonic 专用的 `elfin5_handeye`，避免与 Classic 工作区混用。
Harmonic 配置单独从系统 ROS 构建所需的四个包，不加载 Classic overlay。
构建脚本保留增量产物，不再每次清空工作空间。

构建与环境脚本支持 `VENDOR_BUILD_DIR`（厂家构建根目录）、`APP_BUILD_DIR`（应用构建根目录）、`HANDEYE_WS`（Harmonic
构建根目录）；构建/手眼/MCP/策略入口支持 `ROS_SETUP`（Bash ROS setup 文件）。
`start_sim.sh`、圆周脚本仍支持 `SIM_WS`，Python 启动器仍支持 `--workspace`。
覆盖目录时构建、source 和启动需使用同一个设置。
覆盖目录应使用绝对路径。厂家源码位置始终为 `third_party/elfin_robot_ros2`，
不会随 `VENDOR_BUILD_DIR` 改变。厂家版本和集成补丁见 [fork 说明](../third_party/elfin_robot_ros2.md)。
`HANDEYE_OUTPUT_DIR` 可覆盖标定输出，`ELFIN5_OUTPUT_DIR` 可覆盖相机生成数据的根目录。

旧 `handeye_ws`、无源码文件的 `exp_ws`、根级 build/install/log 已原样归档到
`.build/legacy/`。其中包含旧绝对路径、缓存和安装链接，不作为新入口运行。
迁移后必须在 ROS 主机按上述配置重新构建；不要搬运安装树来替代构建。
历史标定与实验输出保存在 `outputs/`，其内容不代表新目录下已重新验收。

MCP 客户端配置需更新入口绝对路径。两个 Python 应用的本地虚拟环境已保留，激活脚本
和命令入口中的旧前缀已更新；新检出需按各自 README 安装依赖。

## 运行依赖

MCP 经基础 API / MoveIt 发运动目标，SmolVLA 目前直接发关节轨迹，标定通过 MoveIt
采样。启动时选择一个运动指令来源；本次迁移没有新增控制权仲裁器。
SmolVLA 的仿真入口委托公共脚本，但场景仍由手眼包提供；后续提取公共 bringup 时
应独立验证 TF、相机 topic、控制器、规划配置和标定结果。
