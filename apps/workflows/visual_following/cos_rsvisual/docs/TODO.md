# cos_rsvisual TODO 代办清单

> 更新日期：2026-09-09

## ✅ 已完成（仿真环境验证过）

- [x] RealSense 工具链：realsense2_camera 4.51.1 + ros-foxy-librealsense2 2.51.1
  （apt 源失效后从 snapshots.ros.org 手动装 deb），实相机 D435i 跑通（USB2.1, 640x480@15）
- [x] `cos_rsvisual` 包骨架（ament_cmake, 编译零警告）
- [x] 眼在手上 URDF：`urdf/elfin5_rs_d435i.urdf.xacro`，D435i 挂 `elfin_end_link`，
  外参 = launch 参数 `mount_xyz/mount_rpy`；gazebo 深度相机插件
  （`/camera/depth/image_raw`, `/camera/points`）
- [x] SRDF 增补相机 link 的 disable_collisions
- [x] `person_depth_detector`：深度图距离带分割 → 最大连通域 → 反投影 → `/person_pose`
  （仿真中稳定检测行走假人，~34k 像素连通域）
- [x] `person_visual_detector`：HOG 人形检测备选实现（编译通过，未实跑）
- [x] `obstacle_updater`：`/person_pose` → 圆柱体 collision object → `/planning_scene`
  （RViz/MoveIt 规划场景可见假人障碍物）
- [x] `arm_follower`：内置 MoveGroupInterface；初始姿态关节动作（joint5=-0.5）
  规划+执行成功（"Solution was found and executed"）
- [x] 一键仿真 launch：gazebo(actor world) + MoveIt + RViz + 组件容器
- [x] 假人 world：圆心 (2.0,0,0) r=0.5m，位于 home 位姿相机视野与距离带内
- [x] `face_visual_detector`：Haar 正脸检测（可选侧面级联兜底）+ 对齐深度取距
  → `/face_pose` + `/face_debug_image` 调试图（编译零警告，待实相机实跑）
- [x] 真机人脸跟随 launch：`elfin5_rs_face_real.launch.py`（realsense 彩色 +
  align_depth + `elfin_end_link→camera_link` 静态 TF + FaceVisualDetector +
  face_follower(ArmFollower 实例, 订 `/face_pose`)），参数文件
  `config/face_follow.yaml`；前置硬件栈仍由 `cos_ws/scripts/start_real.py` 拉起

## 🔧 进行中

- [ ] **move_action 送达问题**（阻塞"机械臂连续跟随"的最后一环）：
  跟随器发出 94 个 goal，move_group 只收到前 2 个；新建 action client 完全收不到
  响应。疑似本机 FastDDS 发现/组播在高负载下失效。
  正在验证：全链路 `ROS_LOCALHOST_ONLY=1`（纯回环发现）。
  备选：换 CycloneDDS（`apt install ros-foxy-rmw-cyclonedds-cpp`，
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）；或跟随改走 JTC 直发轨迹
  （绕开 move_group，牺牲规划层避障，跟随动作为小姿态调整可接受）。
- [ ] 跟随规划偶发失败（no_solution）：确认初始姿态后姿态目标的可达性，
  必要时放宽 `goal_orientation_tolerance`

## 📋 待办

- [ ] **真机人脸跟随实跑调优**：
  - Haar 参数（`min_neighbors`/`min_face_size`/`scale_factor`）按实机画面调，
    看 `/face_debug_image`（rqt_image_view）；误检多调大 min_neighbors
  - USB2.1 带宽：彩色 + align_depth 640x480@15 若掉帧，先关 align_depth
    改用 `/camera/depth/image_rect_raw`（人脸框取距会受像素错位影响，慎用），
    或降 `rgb_module.profile`/`depth_module.profile` 到 15fps 以下
  - 确认 `/face_pose` 合理后先 `enable_follow:=false` 观察，再开跟随
- [ ] **真机手眼标定**：easy_handeye（eye-in-hand），结果填 `mount_xyz/mount_rpy`；
  骨架：`include/cos_rsvisual/hand_eye_calibrator.hpp`
- [ ] **实相机接入**：`rs_camera.launch.py` + follower.yaml 话题切换到
  `/camera/depth/image_rect_raw`（16UC1）；真机链路参考 `cos_ws/scripts/start_real.py`
  （同用户 + capsh + ROS_LOCALHOST_ONLY=1）
- [ ] **`person_visual_detector` 实跑调优**：仿真需要彩色图（gazebo 插件加 RGB 相机），
  HOG 参数（scale_factor/min_neighbors）按实机画面调
- [ ] **`person_tracker` 滤波**：/person_pose 卡尔曼平滑，抑制检测抖动导致的跟随抖动；
  骨架：`include/cos_rsvisual/person_tracker.hpp`
- [ ] **真机安全**：跟随前 enable_robot 检查、速度缩放上限、急停策略
- [ ] **升级 Humble+ 后**：启用 `config/sensors_3d.yaml` 的 octomap 点云方案
  （foxy 的 MoveIt deb 没有 PointCloudOctomapUpdater 插件）

## ⚠️ 已知限制

- Foxy + FastDDS 在本机高负载下发现/送达不稳；重启仿真前清理
  `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`，launch 已注入 UDP-only 配置
- 深度检测器检测的是"距离带内最大连通域"，不是真正的"人"：
  视野内其他大物体也会被当作目标；多人/遮挡场景需要视觉检测器接管
- `urdf/elfin5_rs_d435i.urdf.xacro` 是 `elfin5_ros2_gazebo/urdf/elfin5.urdf.xacro`
  的拷贝（xacro 无法嵌套 robot 根元素），上游变更需手动同步（文件头有注释）
