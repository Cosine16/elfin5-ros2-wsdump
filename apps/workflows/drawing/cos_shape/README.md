# cos_shape

基于 MoveIt2 + ROS2 topic 的轨迹形状演示包（当前：圆周运动）。

## circle_motion 节点

让机械臂末端（`elfin_end_link`）围绕一个三维点做圆周运动，
支持指定**角速度**（rad/s）或**线速度**（m/s）、圆平面**倾角**和**方位角**。
运动期间末端姿态保持不变。

实现：`computeCartesianPath` 生成整圈关节轨迹 →
`IterativeParabolicTimeParameterization` 打时间戳 → 整体时间缩放到目标周期
（T = 2π/ω 或 T = 2πr/v）→ `execute` 执行，逐圈循环。

### 前提

```sh
# 仿真已启动（Gazebo + MoveIt2）
~/cos_ws/scripts/start_sim.sh moveit2
```

**机械臂需先摆到"法兰大致朝下"的姿态**（圆周运动锁定启动时刻的末端姿态）。
从零位（手臂竖直朝天、法兰朝上）直接 enable，MoveIt 会因目标姿态不可达
（OMPL `Unable to sample any valid states for goal tree`，5 秒超时）而失败。
一个可用的仿真准备姿态（末端约 [0.475, 0, 0.34]，法兰竖直朝下）：

```sh
ros2 action send_goal /elfin_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [elfin_joint1, elfin_joint2, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6], \
   points: [{positions: [0.0, -0.2, 1.6, 0.0, 1.4, 0.0], time_from_start: {sec: 5}}]}}"
```

### 启动

```sh
source ~/cos_ws/.build/apps/install/setup.bash
ros2 launch cos_shape circle_motion.launch.py          # 默认参数，等待 enable
# 或完全自定义：
ros2 run cos_shape circle_motion --ros-args \
  -p center:="[0.3, 0.0, 0.3]" -p radius:=0.05 -p angular_velocity:=0.5 \
  -p inclination:=0.0 -p autostart:=true
```

### 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `group_name` | `elfin_arm` | MoveIt 规划组 |
| `ee_link` | `elfin_end_link` | 末端连杆 |
| `center` | `[0.3, 0.0, 0.3]` | 圆心（规划坐标系，elfin_base） |
| `radius` | `0.05` | 半径 [m] |
| `angular_velocity` | `0.5` | 角速度 [rad/s] |
| `linear_velocity` | `0.05` | 线速度 [m/s] |
| `speed_mode` | `angular` | `angular` / `linear` |
| `inclination` | `0.0` | 圆平面法线与 Z 轴夹角 [rad]（0=水平圆） |
| `azimuth` | `0.0` | 倾角绕 Z 轴的方位 [rad] |
| `waypoints_per_rev` | `64` | 每圈路径点数 |
| `revolutions` | `-1` | 圈数，-1 = 无限 |
| `eef_step` | `0.005` | 笛卡尔插补步长 [m] |
| `autostart` | `false` | 启动即开始运动 |

### Topic 接口（在线修改，下一圈生效）

```sh
# 开始 / 停止
ros2 topic pub --once /circle_motion/enable std_msgs/msg/Bool "{data: true}"

# 改圆心
ros2 topic pub --once /circle_motion/set_center geometry_msgs/msg/Point "{x: 0.35, y: 0.0, z: 0.35}"

# 改速度（发布即切换模式）
ros2 topic pub --once /circle_motion/set_angular_velocity std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic pub --once /circle_motion/set_linear_velocity  std_msgs/msg/Float64 "{data: 0.1}"

# 改倾角（竖直圆 = 1.5708）
ros2 topic pub --once /circle_motion/set_inclination std_msgs/msg/Float64 "{data: 1.5708}"

# 改方位角 / 半径
ros2 topic pub --once /circle_motion/set_azimuth std_msgs/msg/Float64 "{data: 0.7854}"
ros2 topic pub --once /circle_motion/set_radius std_msgs/msg/Float64 "{data: 0.08}"

# 看状态（每圈一条）
ros2 topic echo /circle_motion/state
```

### 调参面板（wxPython GUI）

容器内一次性 `ros2 topic pub` 偶发丢包，推荐使用长驻的图形面板调参：

```sh
source ~/cos_ws/.build/apps/install/setup.bash
ros2 run cos_shape circle_panel.py
```

面板可在线修改圆心/半径/速度（角速度或线速度）/倾角/方位角并提供启停按钮，
底部显示 `~/state` 状态。所有修改下一圈生效。

### 注意

- 速度、圆心、倾角的修改在**下一圈**生效；`enable=false` 会立即停止当前圈。
- 若目标周期超过关节满速能力，节点会警告并按最快速度执行（时间不会快过物理极限）。
- 圆心/半径不可达时节点报错并自动停止，请调整参数后重新 enable。
