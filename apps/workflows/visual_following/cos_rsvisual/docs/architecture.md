# cos_rsvisual 架构说明

> Elfin5 + RealSense D435i（眼在手上 / eye-in-hand）视觉跟随与障碍物感知
> 环境：Ubuntu 20.04 + ROS2 Foxy + Gazebo 11 + MoveIt2 (2.2.3)
> 更新日期：2026-09-09

---

## 1. 总体思路

```
        ┌───────────────────────── Gazebo / 实相机 ─────────────────────────┐
        │  D435i 深度相机(挂在 elfin_end_link 上, 随手腕一起动)              │
        └───────┬──────────────────────────────────────────────────────────┘
                │ /camera/depth/image_raw (sensor_msgs/Image, 32FC1仿真/16UC1真机)
                │ /camera/depth/camera_info (内参, 取一次即可)
                ▼
   ┌─────────────────────────────┐        ┌─────────────────────────────┐
   │ person_depth_detector (默认) │   或   │ person_visual_detector (备选) │
   │ 距离带分割→最大连通域→质心    │  互斥   │ HOG 人形框 + 深度取中位距离   │
   └───────┬─────────────────────┘        └──────────────┬──────────────┘
           │ 反投影成相机系 3D 点, TF 变换到 world        │
           ▼                                             ▼
        /person_pose (geometry_msgs/PoseStamped, frame=world)
           │                                    │
           ▼                                    ▼
   ┌───────────────────┐              ┌──────────────────────────┐
   │   arm_follower    │              │     obstacle_updater      │
   │ 保持末端位置不变,  │              │ 把人写成圆柱体 collision   │
   │ 最短弧旋转末端 +Z   │              │ object 发 /planning_scene │
   │ 轴(相机方向)对准人  │              │ diff → MoveIt 规划绕开人   │
   └───────┬───────────┘              └─────────────┬────────────┘
           │ MoveGroupInterface(elfin_arm)          │
           ▼                                         ▼
        ┌───────────────────────────────────────────────────┐
        │  move_group (OMPL/RRTConnect, 带规划场景避障)        │
        └───────────────────────┬───────────────────────────┘
                                │ /elfin_arm_controller/follow_joint_trajectory
                                ▼
                     gazebo_ros2_control → 机械臂转动
```

设计要点：

- **眼在手上**：相机刚性挂在 `elfin_end_link`，外参是 launch 参数
  `mount_xyz/mount_rpy`（真机手眼标定后改这两个参数即可，不动 URDF 其他部分）。
- **先深度后视觉**：深度检测不依赖光照/纹理，USB2.1 低带宽下稳定；
  HOG 视觉检测器作为备选实现，接口完全一致（同发 `/person_pose`）。
- **跟随动作最小化**：只转姿态不动位置，目标 = 末端 +Z 轴最短弧旋转到指向人，
  带 deadband（0.15m）+ 限频（1.5s）+ 安全高度/距离检查，防止抖动和乱动。
- **组件化**：三个节点都是 rclcpp 组件，跑在同一个 `vision_container` 进程里，
  `/person_pose` 在容器内零拷贝传递（不经过 DDS 网络栈）。

## 2. 节点 / 组件清单

| 组件类 | 节点名 | 职责 | 关键参数（config/follower.yaml） |
|---|---|---|---|
| `cos_rsvisual::PersonDepthDetector` | `/person_depth_detector` | 深度图距离带 [0.3, 2.9]m 分割，最大连通域质心反投影 | `depth_topic`, `min_range`, `max_range`, `min_cluster_pixels`, `target_frame` |
| `cos_rsvisual::PersonVisualDetector` | `/person_visual_detector` | HOG 人形检测 + 深度取距（备选，`enable_visual_detector:=true` 切换） | `detect_every_n_frames`, `scale_factor` |
| `cos_rsvisual::FaceVisualDetector` | `/face_visual_detector` | Haar 人脸检测（彩色图）+ 对齐深度取距 → `/face_pose`（真机专用，参数在 config/face_follow.yaml） | `cascade_path`, `min_neighbors`, `min_face_size`, `face_topic` |
| `cos_rsvisual::ArmFollower` | `/arm_follower`（人脸跟随实例名 `/face_follower`，改订 `/face_pose`） | 末端姿态跟随；内置 MoveGroupInterface 直接规划执行 | `person_topic`, `deadband`, `update_period`, `keep_distance`, `min_height`, `initial_joints`, `enable_follow` |
| `cos_rsvisual::ObstacleUpdater` | `/obstacle_updater` | `/person_pose` → 圆柱体 collision object → 规划场景 | `obstacle_radius`, `obstacle_height`, `update_period`, `move_threshold`, `lost_timeout` |

真机人脸跟随链路（`elfin5_rs_face_real.launch.py`）：机械臂硬件 + MoveIt 由
`cos_ws/scripts/start_real.py` 先行拉起；本 launch 只加相机与视觉：
realsense2_camera（彩色 + align_depth 对齐深度，关点云）→ FaceVisualDetector
→ `/face_pose` → face_follower（ArmFollower）→ move_group（硬件栈的）。
相机 TF 挂接：realsense 驱动发布 `camera_link → *_optical_frame`，本 launch 的
static_transform_publisher 补 `elfin_end_link → camera_link`（外参 =
launch 参数 mount_xyz/mount_rpy），不再起第二个 robot_state_publisher
（硬件栈已发关节 TF，避免双 rsp 重发同一 TF 树）。

容器：`/vision_container`（rclcpp_components `component_container`，单进程）。

## 3. 话题与消息

| 话题 | 消息类型 | 方向 | 说明 |
|---|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | gazebo 插件 → 检测器 | 仿真 32FC1(米)；真机 realsense 为 `/camera/depth/image_rect_raw` 16UC1(毫米)，两种编码都兼容 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | gazebo 插件 → 检测器 | 相机内参，首帧后即释放订阅 |
| `/camera/points` | `sensor_msgs/PointCloud2` | gazebo 插件 → RViz | 点云可视化（foxy 无 octomap 插件，见 §5） |
| `/person_pose` | `geometry_msgs/PoseStamped` | 检测器 → 跟随器/障碍物更新器 | 人的位置，frame=world，orientation 无意义(恒等) |
| `/face_pose` | `geometry_msgs/PoseStamped` | face_visual_detector → face_follower | 人脸位置，frame=world，orientation 无意义(恒等) |
| `/face_marker` | `visualization_msgs/Marker` | face_visual_detector → RViz | 橙色小球(0.08m) |
| `/face_debug_image` | `sensor_msgs/Image` (BGR8) | face_visual_detector → rqt_image_view | 彩色图画人脸框 + 距离标注，调参用 |
| `/camera/color/image_raw` | `sensor_msgs/Image` (BGR8) | realsense → face_visual_detector | 真机彩色图 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (16UC1) | realsense → face_visual_detector | 对齐到彩色的深度图(align_depth.enable:=true) |
| `/person_marker` | `visualization_msgs/Marker` | 检测器 → RViz | 红色小球 |
| `/planning_scene` | `moveit_msgs/PlanningScene`(diff) | obstacle_updater → move_group | 假人圆柱体 `person_obstacle` 的增删改 |
| `/joint_states` | `sensor_msgs/JointState` | ros2_control → 全图 | |
| `/move_action` | `moveit_msgs/action/MoveGroup` | 跟随器 → move_group | MoveIt 规划+执行 action |

## 4. TF 帧链

```
world ──(固定)──> elfin_base_link ──> elfin_base ──> elfin_link1..6
                                                      │
                                              (固定) elfin_end_joint
                                                      ▼
                                              elfin_end_link
                                                      │  camera_mount_joint
                                                      │  【手眼标定外参入口:
                                                      │    mount_xyz/mount_rpy】
                                                      ▼
                                              camera_link ──> camera_depth_frame
                                                      └─> camera_depth_optical_frame
                                                          (gazebo 深度相机插件的 frame)
```

- 仿真中 camera 挂载外参精确已知（URDF 定义）；
- 真机需做眼在手上标定，结果填入 launch 参数 `mount_xyz` / `mount_rpy`。
  标定工具：`cos_realsense` 的 `hand_eye_calib_node`（棋盘格 AX=XB 自研实现，
  用法见该文件头注释；也可用 easy_handeye2）。
- **真机人脸跟随链路的 TF 略有不同**：硬件栈的 robot_state_publisher 用
  官方 URDF（无相机），`elfin_end_link → camera_link` 由
  `elfin5_rs_face_real.launch.py` 的 static_transform_publisher 发布，
  `camera_link → camera_color/depth_optical_frame` 由 realsense2_camera
  驱动发布（publish_tf 默认开）。三段拼成完整链，无需改动硬件栈。

## 5. 已踩过的坑（Foxy 环境实测）

1. **Foxy 的 MoveIt deb (2.2.3) 没有 OctoMap 点云更新插件**：
   `occupancy_map_monitor/PointCloudOctomapUpdater` 在 foxy 分支的 CMakeLists
   里就没有编译（[foxy 分支源码](https://github.com/ros-planning/moveit2/tree/foxy)）。
   因此障碍物感知用 `obstacle_updater` 写 collision object 实现；
   `config/sensors_3d.yaml` 保留，升级 Humble+ 后可启用真正的 octomap 方案。
2. **`elfin_basic_api_node` 的 `/cart_goal` 只能收到首条消息**：
   gdb + 手动发布实证后续消息送达该进程但订阅回调不再触发（它内部
   MoveGroupInterface 状态问题）。因此 `arm_follower` 内置 MoveGroupInterface
   直接对 move_group 规划执行，不再经过 basic_api 中转。
3. **TF 时钟域必须全链路统一**：Gazebo 用仿真时间戳、其余节点默认系统时间戳时，
   tf2 的 `canTransform` 带超时轮询在 foxy 会把执行器永久卡死（gdb 实证）。
   对策：launch 里所有节点（含组件）统一 `use_sim_time`，且组件内所有 TF 查询
   一律用 `tf2::TimePointZero` 零等待。
4. **FastDDS 共享内存残留**：进程被强杀后 `/dev/shm/fastrtps_*` 残留段会导致
   后续进程发现/订阅随机静默失效（与跨用户 0644 段问题同源，参考
   `cos_ws/scripts/start_real.py` 的说明）。对策：launch 注入
   `config/fastrtps_udp_only.xml` 禁用 SHM 只走 UDPv4；异常退出后手动
   `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`。
5. **foxy launch 差异**：`ComposableNode` 没有 `condition` 参数（用
   `OpaqueFunction` 运行时组装）；字符串参数会被尝试 YAML 解析，URDF 里的
   冒号注释会炸（用 `ParameterValue(..., value_type=str)`）；`spawn_entity`
   以 volatile QoS 订阅 `/robot_description`，必须比 robot_state_publisher
   先订阅（launch 里 rsp 延后 3s）。
6. **home 位姿（全零直立）是腕部奇异位形**：姿态目标 IK 不可解
   （"Unable to sample any valid states"）。跟随器首次就绪时先执行
   `initial_joints`（默认 joint5=-0.5 手腕下弯）离开奇异位形。
7. **USB2.1**：D435i 协商到 USB2.1 时深度流保守取 640x480@15；
   深度图质量比 USB3 差，但距离带分割足够用。

## 6. 遗留问题（详见 [TODO.md](TODO.md)）

- **move_action 通道不稳**：本机 FastDDS 在高负载下 action goal 送达偶发中断
  （跟随器发出 94 个 goal，move_group 只收到前 2 个；新建 action client 完全
  收不到响应）。正在验证 `ROS_LOCALHOST_ONLY=1`（走纯回环发现）。
  这是机械臂尚未连续跟随的直接原因。
- 检测到的"人"本质是最近的大块障碍物，需要人在视野内无遮挡。

## 7. 参考文档

- [realsense-ros 4.51.1（Foxy 兼容版）](https://github.com/IntelRealSense/realsense-ros/tree/4.51.1) — 我们装的版本，README 含全部参数/话题说明
- [librealsense 官方仓库](https://github.com/IntelRealSense/librealsense) — SDK、固件、后处理滤波器文档（doc/post-processing-filters.md）
- [MoveIt2 foxy 分支](https://github.com/ros-planning/moveit2/tree/foxy) — 注意 occupancy_map_monitor 的 CMakeLists 不含 updater 插件（octomap 缺失的证据）
- [gazebo_ros_pkgs Wiki](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki) — ROS2 中深度相机由 `gazebo_ros_camera` 插件统一提供
- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) — 眼在手上标定工具（真机阶段使用）
- ROS2 Composition 概念文档：`https://docs.ros.org/en/foxy/Concepts/About-Composition.html`
  （docs.ros.org 对爬虫有 Anubis 验证，浏览器打开正常）
