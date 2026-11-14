# SmolVLA + Elfin5 闭环启动指南（Gz Harmonic + D435i 仿真）

在 Gz Harmonic 手眼仿真中运行 SmolVLA 视觉-语言-动作闭环：
D435i 实时图像 + `/joint_states` → SmolVLA 推理 → 关节轨迹指令 → Elfin5 运动。
可用 RViz 实时查看机械臂姿态与相机画面。

> 说明：`lerobot/smolvla_base` 是在 SO-101 上训练的预训练模型，未在 Elfin5 上微调。
> 本闭环验证的是完整链路，动作不具备真实任务语义。桥接采用"相对位移积分 +
> 单步钳制 + 关节限位"的安全映射（见 `smolvla_bridge.py` 顶部注释）。

## 0. 首次配置（只做一次）

```bash
cd /home/nanoctl/project/ctl-elfin5
bash scripts/build_handeye.sh
python3 apps/policies/smolvla/setup_smolvla.py
```

脚本自动完成：apt 安装 Gz Harmonic / ros_gz（需要时交互询问 sudo 密码）、
创建 venv 并安装 lerobot/torch 等依赖、下载 `smolvla_base` 模型、
检查独立构建的仿真配置、GPU 推理自检。仿真须先通过上述构建入口生成，
不再复制安装树或手工补装 ROS 包路径；详细系统依赖见
[Harmonic 标定说明](../../../document/elfin5_handeye_harmonic.md)。
重复执行复用已有 venv 与模型缓存；`--skip-test` 可跳过自检。

## 1. 启动 Gz + D435i 仿真（终端 1）

```bash
cd /home/nanoctl/project/ctl-elfin5
bash apps/policies/smolvla/run_gz_sim.sh
```

该入口调用共享的 [run_handeye.sh](../../../scripts/run_handeye.sh) 的 `launch` 命令。若提示缺少 ROS 2 Humble underlay，先安装 Humble，或通过 `ROS_SETUP=/实际路径/setup.bash` 指定已有环境；若提示缺少手眼工作区的 setup，则先执行 `bash scripts/build_handeye.sh`。这两种情况会在启动 Gazebo 前退出。

- headless 运行，默认 `ROS_DOMAIN_ID=85`（与真机默认域 0 隔离）。
- 就绪标志：`/camera/color/image_raw`（640×480 rgb8）、`/joint_states`、
  `/elfin_arm_controller/joint_trajectory` 等 topic 出现。
- 检查命令（新开终端）：

  ```bash
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1
  ros2 topic hz /camera/color/image_raw   # 约 5 Hz
  ```

## 2. 可选：把腕部相机对准 ArUco 标定板（终端 2）

零位时相机朝向天空，画面是纯色。执行一次瞄准动作，
让 SmolVLA 获得有内容的视觉输入。**需在启动桥接之前执行**
（否则桥接的轨迹流会与 MoveIt 轨迹冲突）。

```bash
cd /home/nanoctl/project/ctl-elfin5
source scripts/handeye_env.sh
apps/policies/smolvla/.venv/bin/python apps/policies/smolvla/look_at_board.py --ros-args -p use_sim_time:=true
```

成功输出 `Camera now aimed at the ArUco board.`。

## 3. 启动 SmolVLA 闭环（终端 3）

```bash
cd /home/nanoctl/project/ctl-elfin5
ROS_DOMAIN_ID=85 bash apps/policies/smolvla/run_bridge.sh -p gain:=0.05
```

启动后模型加载约 30–60 s，随后机械臂开始连续运动。
每 50 个控制周期做一次 chunk 推理（GTX 1050 Ti 约 2.5–4 s/chunk）。

常用参数（追加在命令末尾，格式 `-p 名:=值`）：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `gain` | 0.02（示例用 0.05） | 动作位移增益，越大动得越快 |
| `max_step_rad` | 0.08 | 单周期最大关节变化（rad），安全钳制 |
| `control_hz` | 10.0 | 控制频率 |
| `task` | `pick up the object` | 语言指令文本 |
| `model_id` | `lerobot/smolvla_base` | 模型（本地缓存路径亦可） |
| `device` | `auto` | `cuda` / `cpu` |
| `image_topic` | `/camera/color/image_raw` | 相机图像 topic |

## 4. 可选：RViz 可视化（终端 4）

```bash
cd /home/nanoctl/project/ctl-elfin5
bash apps/policies/smolvla/run_rviz.sh
```

WSLg 会在 Windows 桌面弹出 RViz 窗口（机器人模型 + TF + MoveIt 面板，
Fixed Frame 为 `world`）。想看 D435i 实时画面：
**Add → rviz_default_plugins/Image**，Topic 填 `/camera/color/image_raw`。

## 5. 验证闭环成功

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1
ros2 node list | grep smolvla                     # 应有 /smolvla_bridge
ros2 topic hz /elfin_arm_controller/joint_trajectory  # 有持续指令流
ros2 topic echo --once /joint_states              # 间隔几秒多执行几次，关节角在变化
```

RViz 中机械臂持续运动、相机画面随之变化即为闭环跑通。

## 注意事项

- **WSL 内存**：整套（仿真 + MoveIt + 桥接 + RViz）约占 4 GB，
  不要再叠加重型进程，也不要在本机编译大型源码，避免 OOM。
- **停止**：在各终端按 Ctrl+C；后台启动的进程用任务管理面板结束。
- **真机**：桥接只依赖 `/joint_states` 与 `elfin_arm_controller`，
  理论上可直接接真机（域 0），但务必先把 `gain`、`max_step_rad` 调小并确认急停可用。
- **Classic Gazebo 备选**：`elfin5_ros2_gazebo` 仿真无相机，桥接会用
  `outputs/handeye/latest_camera.png` 兜底图像；控制器需用
  `bash apps/policies/smolvla/ensure_controllers.sh` 激活（厂家 launch 依赖的
  `ros2 control` CLI 本机未安装）。
