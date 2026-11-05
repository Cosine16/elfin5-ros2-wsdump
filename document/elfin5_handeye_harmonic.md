# Elfin5 + D435i 眼在手上标定（Humble / Gazebo Harmonic）

## 实现与环境

新增应用包 `elfin5_handeye`，复用厂家 Elfin5 的运动学、STL、关节限位和 `gz_ros2_control` 接口；没有修改厂家子模块。当前入口适用于 **Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 8**。已有的 `cos_rsvisual` 相机入口使用 Gazebo Classic 插件，不能混用。

本流程通过 MoveIt 2 的 `/compute_ik` 和 `/move_action` 规划、执行仿真轨迹，用 OpenCV ArUco/PnP 和 `calibrateHandEye(PARK)` 求解。**不依赖 MoveIt Calibration 的第三方 RViz 面板**。

主要文件：

- [组合 Xacro](../apps/workflows/handeye/elfin5_handeye/urdf/elfin5_d435i.urdf.xacro)：Elfin5 + 支架 + D435i。
- [相机模型](../apps/workflows/handeye/elfin5_handeye/urdf/d435i.xacro)：外形、惯量、光学坐标系、RGB-D 和 IMU。
- [仿真启动](../apps/workflows/handeye/elfin5_handeye/launch/handeye.launch.py)：Gazebo、ROS 桥、控制器、MoveIt、可选 RViz。
- [标定世界](../apps/workflows/handeye/elfin5_handeye/worlds/handeye.sdf)：固定 ArUco 目标，由黑白几何单元直接渲染，无需下载纹理或 Fuel 模型。
- [自动采样与验收](../apps/workflows/handeye/elfin5_handeye/elfin5_handeye/calibrate.py)。
- [几何算法与检测](../apps/workflows/handeye/elfin5_handeye/elfin5_handeye/geometry.py)。

## 构建和运行

在仓库根目录的 Bash 终端中执行：

```bash
bash scripts/build_handeye.sh
bash scripts/run_handeye.sh urdf
bash scripts/run_handeye.sh launch
```

构建仅选择 `elfin_description`、`elfin5_gz_sim`、`elfin5_ros2_moveit2`、`elfin5_handeye` 四个包，使用独立的 `.build/handeye`，避免旧工作区的绝对路径缓存及 EtherCAT 构建依赖。系统必须已经安装与 Harmonic 匹配的 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`、MoveIt 2、ROS 控制器、`xacro`、`cv_bridge`、`liburdfdom-tools`、Python OpenCV（包含 `aruco`）、NumPy、SciPy、PyYAML。脚本会检查主要 ROS 包。不要把 Humble 默认的 Fortress 桥与 Harmonic 8 二进制混搭。

默认无 GUI，启用 Ogre2 headless rendering；无 GPU 时可以使用 Mesa 软件渲染。需要可视化时：

```bash
bash scripts/run_handeye.sh launch headless:=false rviz:=true
```

脚本默认设置 `LIBGL_ALWAYS_SOFTWARE=1`。GPU 驱动可用时可在命令前设置 `LIBGL_ALWAYS_SOFTWARE=0`。

第二个终端运行自动标定：

```bash
bash scripts/run_handeye.sh calibrate
```

流程依次等待传感器与 MoveIt 就绪、添加地面和板的规划碰撞体、尝试 16 个视角、执行轨迹、等待静止、从实际 Gazebo 图像检测 ArUco、按图像时间戳查 TF，最后求解并验收。不满足 IK、检测或重复样本条件的视角会跳过；不足 10 组有效观测会失败。软件渲染下整轮可能需要约 10–15 分钟。`Ctrl+C` 停止启动终端中的仿真。

输出在 [outputs/handeye](../outputs/handeye/)：

- `elfin5_d435i.urdf`：展开的 URDF，mesh 使用 `package://elfin_description/...`。
- `samples.json`：时间戳、机械臂位姿、视觉估计板位姿、重投影误差。
- `sample_XX.png`：实际仿真相机图像与绿色检测边框。
- `calibration.yaml`：相机安装外参、四阶矩阵、四元数、误差指标、验收结果。

重复运行会更新同名输出；需要保留多次实验时使用不同目录：

```bash
bash scripts/run_handeye.sh calibrate -p output_dir:="$PWD/outputs/handeye/run02"
```

采样进程中断时，`samples.json` 在每个成功样本后立即落盘。重新启动**相同安装参数**的仿真后，可以直接重算已有样本（不再移动机械臂）：

```bash
bash scripts/run_handeye.sh calibrate -p samples_file:="$PWD/outputs/handeye/samples.json"
```

新的运行会先将结果标记为 `passed: false, status: collecting`，异常时记录 `status: failed`，避免把旧的成功文件误当作当前结果。

## 模型和坐标约定

相机安装链：

```text
world -> elfin_base_link -> ... -> elfin_link6 -> elfin_end_link
                                                  ├─ camera_bracket
                                                  └─ camera_link
                                                       ├─ camera_color_frame -> camera_color_optical_frame
                                                       ├─ camera_depth_frame -> camera_depth_optical_frame
                                                       └─ camera_imu_frame
```

默认 `elfin_end_link -> camera_link`：

```yaml
mount_xyz: [0.0, 0.0, 0.07]                 # m
mount_rpy: [0.0, -1.5707963267948966, 0.0] # rad
```

这表示法兰前方 70 mm，镜头朝法兰 +Z。支架是 50 mm 长的简化圆柱；相机外壳约 90 × 25 × 25 mm、质量 72 g。安装参数可更改：

```bash
bash scripts/run_handeye.sh launch mount_xyz:="0 0 0.075" mount_rpy:="0 -1.5707963267948966 0"
```

支架为仿真占位几何，改变安装方式后需相应修改其长度/形状并核实碰撞。真实法兰、支架孔位、线缆空间需按实物测量。默认安装外参是仿真真值，不是实测标定结果。

相机 body 使用 X 前、Y 左、Z 上；optical 使用 X 右、Y 下、Z 前，固定旋转为 `rpy=(-pi/2, 0, -pi/2)`。光心在 body 的 X=13 mm。RGB 与深度为共光心理想模型，深度/点云头部使用 `camera_color_optical_frame`；`camera_depth_optical_frame` 与其重合。没有模拟双目红外成像、厂商畸变、真实 RGB/深度基线及 IMU 误差。这足以验证软件标定闭环，不等同于完整 librealsense 硬件仿真。

| ROS topic | 类型 / 说明 |
|---|---|
| `/camera/color/image_raw` | 640 × 480 RGB，15 Hz 仿真时间 |
| `/camera/color/camera_info` | 与图像匹配的内参和 optical frame |
| `/camera/depth/image_rect_raw` | 深度，`32FC1`，米 |
| `/camera/depth/color/points` | PointCloud2 |
| `/camera/imu` | Imu，100 Hz 仿真时间 |
| `/joint_states` | 六轴实际仿真状态 |
| `/clock` | Gazebo 仿真时间 |

脚本默认使用 ROS_DOMAIN_ID=85、ROS_LOCALHOST_ONLY=1 和独立 GZ_PARTITION，所有仿真节点统一使用仿真时间。若调整域，两终端都设置同一个 `HANDEYE_ROS_DOMAIN_ID`。标定程序只面向本仿真场景，不要将自动采样程序直接用于真机。

## 标定计算与验收

固定目标：`DICT_6X6_250`，ID=23，黑色外边框边长 **0.16 m**，白色底板 0.22 m，目标中心约在 world `(0.75, 0, 0.55)`。尺寸参数是黑色 marker 的边长，不含白边。

记 `A = world_T_hand`、`X = hand_T_optical`、`B = optical_T_target`，固定板应满足 `A X B = 常量`。求解器只接收 A 和来自图像 PnP 的 B。相机安装真值只用于生成观察姿态、求解后的误差对照；没有用 Gazebo 板位姿替代视觉检测。

每次 MoveIt 执行成功后，等待至少 0.8 s 仿真时间的新图像，检查关节速度，再按**图像时间戳**查询机械臂 TF。拒绝重复位姿、重投影 RMS >1 px 的图像、少于 10 组样本及单轴旋转退化数据。

求出的 optical 外参通过相机内部固定 TF 转成 `elfin_end_link -> camera_link`。结果不会另发一条与 URDF 冲突的 TF。仿真验收要求：

- 安装平移误差 <5 mm，旋转误差 <1°。
- 反算静止标定板的位置 RMS <5 mm，姿态最大偏差 <1°。
- 六类输入（RGB、内参、深度、点云、IMU、关节状态）均已收到。

默认安装的 pitch=-π/2 靠近欧拉角奇异点，估计出的 roll/yaw 可能与默认数值差异很大；判断姿态误差应使用四元数/旋转矩阵，而不是逐项相减 RPY。

MoveIt 碰撞检查包含相机、支架、地面和板；没有启用 Octomap，因为标定场景的已知障碍由碰撞体表达。启动时 MoveIt 可能提示未配置 3D sensor plugin，这不影响本流程，但也不能据此声称启用了点云避障。

## 检查和故障定位

```bash
bash scripts/run_handeye.sh check control list_controllers
bash scripts/run_handeye.sh check topic echo --once /camera/color/camera_info
bash scripts/run_handeye.sh check topic echo --once /camera/color/image_raw --field header
bash scripts/run_handeye.sh check action list
```

控制器应为 `joint_state_broadcaster` 和 `elfin_arm_controller`，状态均为 active。相机话题存在但没有图像时检查 Ogre2/EGL 渲染日志和 ROS/GZ 桥版本；软件渲染慢时 wall-clock topic hz 会低于仿真设定频率。`IK failed: -31` 表示当前种子下该视角没有可行解，程序会继续其他视角。求解未通过时检查输出样本和误差，不要把失败结果当成标定成功。

离线回归：

```bash
PYTHONPATH=apps/workflows/handeye/elfin5_handeye /usr/bin/python3 -m pytest -q apps/workflows/handeye/elfin5_handeye/test
```

测试覆盖外参方向、错误逆矩阵约定、单轴退化、最小样本数，以及从实际 SDF 黑白单元恢复图案后的 ArUco 检测/PnP。离线测试不是 Gazebo 运行证据；实测记录见下节。

## 本机实测记录

2026-10-19，在本机沙箱外 ROS 环境完成以下验证（Gazebo 8.15.0、ROS 2 Humble、OpenCV 4.5.4）：

| 项目 | 实测结果 |
|---|---|
| colcon 构建 | 4 个包成功 |
| Xacro 展开 / check_urdf | 通过，根为 world，六轴与相机/支架链完整 |
| 控制器 | joint_state_broadcaster / elfin_arm_controller 均 active |
| MoveIt 规划与执行 | 实际调用 OMPL、FollowJointTrajectory 完成多视角运动 |
| 有效样本 | 13 组来自 Gazebo RGB 图像的 ArUco/PnP + 同时间戳机械臂 TF |
| 安装平移误差 | **3.354183 mm** |
| 安装旋转误差 | **0.372321°** |
| 固定板位置 RMS | **0.710130 mm** |
| 固定板旋转最大偏差 | **0.685790°** |
| RGB / CameraInfo frame | camera_color_optical_frame |
| 深度 / 点云 | 收到，深度编码 32FC1，均在 camera_color_optical_frame |
| IMU | 收到，camera_imu_frame |
| 标定验收 | **passed: true** |
| 离线回归 | 3 passed；Python 编译、编辑器诊断和 diff 空白检查通过 |

本轮尝试 16 个视角，第 9 个视角逆解失败并跳过；采集到第 13 个有效样本后，长时间运行的工具进程以 SIGTERM 结束。随后重启相同场景，用保存的 13 组真实观测恢复求解，程序正常退出并输出上述 PASS。没有声称完成了全部 16 个视角，也没有用合成位姿替代未完成的观测。

原始证据在 [calibration.yaml](../outputs/handeye/calibration.yaml)、[samples.json](../outputs/handeye/samples.json)、[采样日志](../outputs/handeye/calibrate.log)、[恢复求解日志](../outputs/handeye/recovery.log)、[第 13 组检测图](../outputs/handeye/sample_13.png)。这些运行产物被 Git 忽略，但保存在当前工作区；按上述命令可重新生成。验证结束后已停止本次启动的仿真。

这证明了仿真模型、图像检测、时间同步、MoveIt 执行和手眼求解链路；实机 D435i 内参、真实安装外参、IMU 标定及真机轨迹安全性尚未验证。
