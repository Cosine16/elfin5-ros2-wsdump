# cos_rsvisual

RealSense D435i 视觉感知 + Elfin5 视觉跟随包(眼在手上 / eye-in-hand)。

> 详细架构(思路/节点/话题/TF 帧/踩坑记录)见 [docs/architecture.md](docs/architecture.md)
> 进度与遗留问题见 [docs/TODO.md](docs/TODO.md)

包含内容:

- **urdf/elfin5_rs_d435i.urdf.xacro**: elfin5 + 简化 D435i 相机模型(box, 不依赖
  realsense2_description)+ gazebo 深度相机插件。文件同步自
  `elfin5_ros2_gazebo/urdf/elfin5.urdf.xacro`, 上游变更需手动同步。
- **config/elfin5_rs.srdf**: elfin5 SRDF + 相机 link 的 disable_collisions。
- **config/follower.yaml**: 四个组件的参数(按节点名分节)。
- **config/fastrtps_udp_only.xml**: 禁用 FastDDS 共享内存传输(防止强杀后
  /dev/shm 残留导致发现/订阅静默失效), 由 launch 自动注入。
- **config/sensors_3d.yaml**: octomap 点云配置。【Foxy 下不生效】—— foxy 的
  MoveIt deb 没有编译 PointCloudOctomapUpdater 插件, 保留供升级 Humble+ 后启用。
- **src/**: 五个 rclcpp 组件(编译为单一 SHARED 库 `cos_rsvisual_components`,
  同容器零拷贝传递 /person_pose 与 /face_pose):
  - `PersonDepthDetector`: 深度图距离带分割 → 最大连通域 → 反投影 → `/person_pose`
  - `PersonVisualDetector`: HOG 人形检测(备选, 默认不加载, 与深度检测器互斥)
  - `FaceVisualDetector`: Haar 人脸检测(彩色图) + 对齐深度取距 → `/face_pose`,
    面向真机实相机(仿真 actor 纹理不满足 Haar 特征), 另发 `/face_debug_image` 调试图
  - `ArmFollower`: 订阅 `/person_pose`(真机人脸跟随实例名为 `face_follower`,
    改订 `/face_pose`), 保持末端位置不变、转动姿态使
    `elfin_end_link` +Z 轴(相机方向)对准目标; 内置 MoveGroupInterface 直接对
    move_group 规划执行(不经过 elfin_basic_api, 原因见 architecture.md §5.2)
  - `ObstacleUpdater`: 把 `/person_pose` 作为圆柱体 collision object 写入
    MoveIt 规划场景(foxy 没有 octomap 插件的替代避障方案)
- **worlds/elfin5_actor.world**: sun + ground_plane + 行走假人 actor
  (gazebo11 自带 walk.dae, 圆心 (2.0,0,0) 半径 0.5m 循环行走 —— 该位置恰在
  home 位姿相机视野与检测距离带内, 地面在 3.0m 处被距离带上限 2.9m 排除)。
- **include/cos_rsvisual/*.hpp**: 后续扩展骨架(手眼标定/跟踪滤波, 函数体 TODO,
  不参与编译)。

## 依赖

ROS2 Foxy 包: `rclcpp` `rclcpp_components` `sensor_msgs` `geometry_msgs` `std_msgs`
`visualization_msgs` `cv_bridge` `image_geometry` `tf2` `tf2_ros` `tf2_geometry_msgs`
`moveit_msgs` `moveit_ros_planning_interface` `OpenCV`, 以及工作区内的
`elfin5_ros2_gazebo` `elfin5_ros2_moveit2` `elfin_basic_api`。
人脸检测的 Haar 级联文件由系统包 `libopencv-data` 提供
(`/usr/share/opencv4/haarcascades/`, OpenCV 4.2 自带, 无需额外安装)。

**仿真运行额外需要(仅运行时, 不影响编译)**:

```bash
sudo apt install ros-foxy-gazebo-plugins   # 提供 libgazebo_ros_camera.so (深度相机插件)
```

实相机验证需要: `ros-foxy-realsense2-camera` + `ros-foxy-librealsense2`
(foxy 官方源已下架, 可从 http://snapshots.ros.org/foxy/final/ubuntu 下载 deb 手动安装)。

编译:

```bash
cd /path/to/ctl-elfin5
bash scripts/build_apps.sh --packages-select cos_rsvisual
source scripts/elfin_env.sh
```

## 使用路径

### 1. 仿真一键启动( gazebo + MoveIt + 视觉跟随 )

```bash
export ROS_LOCALHOST_ONLY=1   # 推荐: 纯回环发现, 本机 FastDDS 组播不稳
ros2 launch cos_rsvisual elfin5_rs_sim.launch.py
```

- 行走假人在机械臂前方绕圈; 深度检测器输出 `/person_pose`;
  跟随器内置 MoveGroupInterface 规划执行, 末端相机对准人;
  障碍物更新器同步把假人写进规划场景(RViz MotionPlanning 可见)。
- 换用 HOG 视觉检测器(与深度互斥):
  `ros2 launch cos_rsvisual elfin5_rs_sim.launch.py enable_visual_detector:=true`
- 运行时关闭跟随: `ros2 param set /arm_follower enable_follow false`

异常退出后建议清理 DDS 共享内存残留, 再重启:

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

### 2. 实相机验证(只有相机, 先看数据)

```bash
ros2 launch cos_rsvisual rs_camera.launch.py
```

foxy 的 realsense2_camera 参数名随版本不同(3.2.3 用 `depth_width/depth_fps/
enable_pointcloud`, 4.51+ 用 `depth_module.profile/pointcloud.enable`),
launch 文件内有注释, 以 `rs_launch.py` 实际为准。

**USB 注意**: D435i 必须接 USB3 口。若只有 USB2.1 口, 深度流需要降到
640x480@15 或更低(`rs-enumerate-devices` 可查看协商速率)。

### 3. 真机接入(预留, 未实测)

1. 真机底驱: 参考 `cos_ws/scripts/start_real.py`(同用户 + capsh +
   ROS_LOCALHOST_ONLY=1, 避免跨用户 DDS 共享内存段问题);
2. 启动 `rs_camera.launch.py` 开实相机;
3. 启动组件容器, 把 `config/follower.yaml` 的:
   - `depth_topic` 对准实相机深度图 `/camera/depth/image_rect_raw` (16UC1);
   - `target_frame` 对准真机 TF 根(通常为 `world`)。

只换检测器: 直接改 launch 中 `vision_container` 的
`composable_node_descriptions`, 两个检测器输出接口相同(`/person_pose`)。

### 4. 真机人脸跟随(实相机 + 实机械臂)

前置: 先用 `cos_ws/scripts/start_real.py` 拉起机械臂硬件 + MoveIt 栈
(EtherCAT ros2_control_node 需要 capsh 用户态授权), 并在 Elfin Control
Panel 里 Clear Fault → Servo On。然后:

```bash
export ROS_LOCALHOST_ONLY=1   # 与 start_real.py 一致
ros2 launch cos_rsvisual elfin5_rs_face_real.launch.py
```

- 启动 realsense2_camera(彩色 640x480@15 + `align_depth` 深度对齐彩色,
  关点云省 USB 带宽), `elfin_end_link → camera_link` 静态 TF 把相机
  TF 树挂到机械臂 TF 树上(外参 = `mount_xyz/mount_rpy` launch 参数);
- `face_visual_detector`: Haar 正脸检测(参数见 `config/face_follow.yaml`),
  人脸框中心区域深度中值反投影 → `/face_pose`; `/face_debug_image`
  调试图用 `rqt_image_view` 查看;
- `face_follower`(ArmFollower 实例): 订阅 `/face_pose`, 末端位置不变、
  转动姿态让相机对准人脸; 真机速度缩放默认 0.2。
- 运行时开关: `ros2 param set /face_follower enable_follow false`
  (检测器同理: `/face_visual_detector enable false`)。
- **安全**: 跟随动作为小姿态调整, 但仍建议首次运行时把人机距离保持在
  keep_distance(0.4m)以上, 手放急停旁; 先 `enable_follow:=false` 启动,
  确认 `/face_pose` 输出合理(RViz/echo)后再打开跟随。

## 眼在手上标定

相机与 `elfin_end_link` 的固定外参在 launch 参数:

```bash
ros2 launch cos_rsvisual elfin5_rs_sim.launch.py mount_xyz:="0 0 0.02" mount_rpy:="0 0 0"
```

真机做完手眼标定(如 easy_handeye)后, 把结果填入这两个参数(或直接改
`launch/elfin5_rs_sim.launch.py` 的默认值)即可, 不必改 URDF。
自研标定流程的骨架在 `include/cos_rsvisual/hand_eye_calibrator.hpp`。

## 已知限制

- 仿真深度图为 32FC1(米), 真机为 16UC1(毫米), 检测组件两种编码都支持,
  但话题名不同, 切换时注意 follower.yaml。
- gazebo 深度相机插件由 `ros-foxy-gazebo-plugins` 提供, 未装时相机无数据
  (编译不受影响)。
- foxy 的 MoveIt deb 没有 octomap 点云插件, 障碍物感知由 obstacle_updater
  以 collision object 方式实现; sensors_3d.yaml 保留供升级后使用。
- 本机 FastDDS 在高负载下 action/topic 送达偶发中断(move_action 通道),
  排查进展见 docs/TODO.md。
