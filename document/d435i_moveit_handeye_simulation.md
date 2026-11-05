# Elfin5 + D435i + MoveIt 2 手眼标定仿真

> 2026-10-19 更新：下文保留早期 Classic / 第三方面板方案参考。当前已实现的 Humble + Gazebo Harmonic 自动标定入口、构建命令和实际验收记录请看 [Elfin5 手眼标定运行说明](elfin5_handeye_harmonic.md)。当前环境可在沙箱外访问 ROS，文末的旧环境限制不再代表本次验证状态。

## 目标与边界

在 Ubuntu 22.04 / ROS 2 Humble 下，为 Elfin5 配置末端 D435i 眼在手上模型，使用 Gazebo 相机图像、MoveIt 2 机械臂状态和 MoveIt Calibration 的 ROS 2 移植版完成软件链路验证。仿真可以验证 TF、相机图像、标定板检测、采样与求解流程；仿真结果只应恢复仿真中预先设定的安装外参，不能代替实机相机内参和安装外参标定。

工作分支：`feat/elfin5-d435i-hand-eye`。

## 仓库中已有的实现

- `apps/workflows/visual_following/cos_rsvisual/urdf/elfin5_rs_d435i.urdf.xacro` 已把简化 D435i 几何固定连接到 `elfin_end_link`，并声明 `camera_link`、`camera_depth_frame`、`camera_depth_optical_frame` 和 Gazebo Classic 深度相机插件。安装位姿由 `mount_xyz` / `mount_rpy` 控制。
- `apps/workflows/visual_following/cos_rsvisual/launch/elfin5_rs_sim.launch.py` 已以 `use_real_hardware:=false`、`use_fake_hardware:=false` 启动 Gazebo + MoveIt 仿真，默认使用 `mount_xyz="0 0 0.02"`、`mount_rpy="0 0 0"`。
- `apps/workflows/visual_following/cos_rsvisual/config/elfin5_rs.srdf` 已加入相机与相邻机械臂连杆的碰撞豁免。
- `apps/sensors/realsense/cos_realsense/urdf/d435i.urdf.xacro` 是独立相机模型；Elfin5 联合仿真当前使用的是上面的组合 xacro，不要同时再 spawn 一台独立相机。

## 重要兼容说明

MoveIt 官方教程讲解的是 MoveIt Calibration RViz 插件；教程页面仍包含 ROS 1 的 `catkin` 命令。ROS 2 下应使用该项目的 `ros2` 移植分支并用 `colcon` 构建，不能照搬 `catkin build`。教程明确说明仿真可以走完大部分步骤，但真实精度仍需实际机械臂和相机确认。

本仓库现有 D435i 插件使用 `libgazebo_ros_camera.so`，即 Gazebo Classic + `gazebo_ros`。它不等同于仓库另有的 Gazebo Harmonic (`gz-sim`) 机械臂工作流；若本机只安装 Harmonic，需先完成 Classic 兼容环境或另行移植相机传感器和 ROS 图像桥接，不能直接假设此 launch 可运行。

## 1. 检查 Humble 环境并构建工作区

在 WSL Ubuntu 22.04 的 ROS 环境终端执行：

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro ros-humble-robot-state-publisher \
  liburdfdom-tools python3-vcstool
source /opt/ros/humble/setup.bash
cd /path/to/elfin5_station  # 替换为本机仓库目录
export WS="$PWD"

# 厂家驱动/描述包先构建
bash "$WS/scripts/build_vendor.sh"

# 自研应用层后构建
bash "$WS/scripts/build_apps.sh"

source "$WS/.build/apps/install/setup.bash"
```

确认运行时依赖：

```bash
ros2 pkg prefix gazebo_ros
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix cos_rsvisual
ros2 pkg prefix realsense2_camera
```

Gazebo 仿真需要 Gazebo Classic、`gazebo_ros`、MoveIt 2、相机插件和图形显示可用。若 `gazebo_ros` 或插件不存在，先安装与 Humble 匹配的依赖；RealSense 真机驱动只用于以后接入实体 D435i，Gazebo 相机图像由 URDF 中的 Gazebo 插件产生。

## 2. 展开并检查组合 URDF

```bash
source /opt/ros/humble/setup.bash
WS=/path/to/elfin5_station  # 替换为本机仓库目录
source "$WS/.build/vendor/install/setup.bash"
source "$WS/.build/apps/install/setup.bash"

ros2 run xacro xacro \
  $(ros2 pkg prefix cos_rsvisual)/share/cos_rsvisual/urdf/elfin5_rs_d435i.urdf.xacro \
  use_real_hardware:=false use_fake_hardware:=false \
  mount_xyz:="0 0 0.02" mount_rpy:="0 0 0" \
  > /tmp/elfin5_d435i.urdf

check_urdf /tmp/elfin5_d435i.urdf
```

展开结果应包含相机固定关节以及三个相机 frame。`mount_xyz` 和 `mount_rpy` 是用于仿真的初值；不要把默认值当作已测得的实机标定结果。

## 3. 启动 Gazebo + MoveIt 2 仿真

```bash
source /opt/ros/humble/setup.bash
WS=/path/to/elfin5_station  # 替换为本机仓库目录
source "$WS/.build/apps/install/setup.bash"
ros2 launch cos_rsvisual elfin5_rs_sim.launch.py
```

启动后另开终端检查：

```bash
source /opt/ros/humble/setup.bash
WS=/path/to/elfin5_station  # 替换为本机仓库目录
source "$WS/.build/apps/install/setup.bash"

ros2 topic list | grep -E '^/camera/.*(image_raw|camera_info)$'
ros2 topic echo --once /camera/depth/camera_info
ros2 topic echo --once /camera/depth/image_raw --field header
ros2 run tf2_ros tf2_echo elfin_end_link camera_depth_optical_frame
ros2 action list | grep follow_joint_trajectory
```

应确认相机图像和 `CameraInfo` 持续发布，图像 `header.frame_id` 与 TF 中的 optical frame 相符，MoveIt 控制器 action 可见。`ros2 topic echo` 检查图像时只查看 header，避免向终端打印整帧数据。

## 4. 安装 MoveIt Calibration ROS 2 插件

在独立 overlay 中获取 ROS 2 移植版，避免把第三方源码混入厂家或应用包：

```bash
mkdir -p ~/moveit_calibration_ws/src
cd ~/moveit_calibration_ws
git clone https://github.com/ros-planning/moveit_calibration.git \
  -b ros2 src/moveit_calibration
vcs import src < src/moveit_calibration/moveit_calibration.repos --skip-existing

source /opt/ros/humble/setup.bash
WS=/path/to/elfin5_station  # 替换为本机仓库目录
source "$WS/.build/vendor/install/setup.bash"
source "$WS/.build/apps/install/setup.bash"
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

需要 `vcs` 命令时安装 `python3-vcstool`。若上游 `ros2` 分支在 Humble 上出现依赖或 API 不兼容，记录失败包和日志后再固定到兼容提交；不要切换成 ROS 1 分支。

## 5. 标定板与仿真采样

MoveIt Calibration 面板可生成 ArUco 标定图案，但 Gazebo 相机必须实际看到有纹理的平面标定板。把面板生成并保存的目标图做成 Gazebo 可读取的贴图，放到相机视野可达且相对 `world` 固定的位置；确认 RViz 的相机图像面板能看到清晰完整的目标、`CameraInfo` 有有效内参，且目标检测图像能叠加检测坐标轴。仅在 RViz 中打开目标图而 Gazebo 相机看不到目标，不构成仿真标定输入。

在 MoveIt RViz 中打开 `Panels` → `Add New Panel` → `HandEyeCalibration`，按以下参数设置：

| 字段 | Elfin5 仿真值 |
|---|---|
| Sensor configuration | Eye-in-hand |
| Sensor frame | `camera_depth_optical_frame` |
| End-effector frame | `elfin_end_link` |
| Robot base frame | `world` |
| Planning Group | `elfin_arm` |
| Image topic | 选择正在发布的 RGB 图像 topic |
| CameraInfo topic | 与所选图像配套的 CameraInfo topic |
| Object frame | 面板默认的 `handeye_target`，或与仿真目标配置一致 |

使用 MoveIt MotionPlanning 面板移动仿真机械臂，采集至少 5 组样本，建议 12–15 组；姿态应有足够变化，至少绕两个不同轴旋转，并避免目标离开相机视野。标定结果通过面板导出。对比结果与仿真 xacro 的 `mount_xyz` / `mount_rpy`，确认平移和旋转接近设置值；误差受图像纹理、渲染、采样姿态与相机模型影响。

## 6. 当前验证限制与验收记录

本次 Codex 执行环境报告 `ROS_DISTRO=humble`，但实际找不到 `/opt/ros/humble/setup.bash`，仓库的 `install/setup.bash` 也因缺失 ROS underlay 无法 source；因此不能在当前工具 shell 真实启动 Gazebo、RViz、MoveIt 或运行 xacro 检查。请在 WSL 的实际 Ubuntu 环境执行上述步骤并把首次失败日志保存下来，再补全此节的运行结果。

验收项：

- [ ] 工作区构建成功。
- [ ] 组合 xacro 展开成功，URDF 检查通过。
- [ ] Gazebo 中能看到 Elfin5 末端相机，MoveIt 能规划和执行仿真关节轨迹。
- [ ] `/camera` 图像与 CameraInfo 发布正常，图像 frame 和 TF 一致。
- [ ] Gazebo 相机图像可检测固定 ArUco 板，MoveIt Calibration 可采样并计算结果。
- [ ] 计算外参与 xacro 仿真初值在记录的误差范围内。
- [ ] 实机 D435i 的内参、安装外参和 EtherCAT 真机控制尚待有硬件后单独验证。

## 参考

- [MoveIt 2 Hand-Eye Calibration 教程](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.rst)
- [MoveIt Calibration ROS 2 构建说明](https://github.com/AndrejOrsula/moveit2_calibration)
- [ROS 2 移植仓库](https://github.com/ros-planning/moveit_calibration/tree/ros2)
