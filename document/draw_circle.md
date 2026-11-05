> 本文保留历史排查或方案记录，原路径与当时环境不改写。当前源码、构建和运行入口见 [目录规范](repository_layout.md) 与 [厂家 fork 说明](../third_party/elfin_robot_ros2.md)。

我先沿着当前节点的主入口、定时器回调和控制指令发布路径读取代码，再对照启动文件确认它实际连接到哪个机械臂控制接口。重点是把“启动后到画圈动作”的调用链串起来。

Read 

Read , lines 1 to 320

Read skill `ros2-engineering-skills`, lines 1 to 220

当前代码已经显示出核心机制：它不是实时逐点发速度，而是每圈先生成笛卡尔路径，再交给 MoveIt 规划、加时间参数并执行。接下来我补齐路径规划函数和启动入口，以确认每个关键参数如何落到实际运动上。

Searched for text `circle_motion` (`cos_ws` · `elfin_ws/src/cos_shape/**`), 22 results

Read , lines 300 to 431

Read 

路径函数确认了几何闭环：默认是水平圆，末端姿态锁定；每圈用 64 个离散姿态点，MoveIt 在点之间补足笛卡尔段。现在只再核对 launch 的默认参数和文档里的启动命令，就可以把“如何触发”和“速度如何换算”说明完整。

Read 

Read 

Read 

**核心结论**

这个节点通过 MoveIt 的笛卡尔路径规划，把末端执行器的目标位置按圆周离散成一组路径点，然后将整圈轨迹发送给机械臂控制器执行。它不是逐时刻发布速度，而是“每圈规划一次、每圈执行一次”。

代码入口在 `circle_motion_node.cpp`。

**1. 节点启动**

`main()` 创建 `CircleMotion` 节点，并启动一个独立的 ROS2 executor 线程：

```cpp
std::thread spinner([&executor]() { executor.spin(); });
node->run();
```

这样 `run()` 可以持续执行运动，同时另一个线程仍能处理：

- `/circle_motion/enable`
- `/circle_motion/set_radius`
- `/circle_motion/set_center`
- `/circle_motion/set_angular_velocity`
- 其他参数话题

构造函数中加载默认参数，例如：

- 圆心：`[0.3, 0.0, 0.3]`
- 半径：`0.05 m`
- 角速度：`0.5 rad/s`
- 每圈路径点：`64`
- 无限循环：`revolutions = -1`
- 默认不自动启动：`autostart = false`

对应代码位于 `circle_motion_node.cpp:94`。

**2. 等待启动命令**

`run()` 创建 MoveIt 的 `MoveGroupInterface`：

```cpp
moveit::planning_interface::MoveGroupInterface mgi(
    shared_from_this(), group_name_);
```

默认规划组是 `elfin_arm`，末端连杆是 `elfin_end_link`。

之后循环等待：

```cpp
if (!enabled_.load()) {
  rclcpp::sleep_for(...);
  continue;
}
```

当执行：

```bash
ros2 topic pub --once /circle_motion/enable \
  std_msgs/msg/Bool "{data: true}"
```

订阅回调把：

```cpp
enabled_.store(true);
```

于是运动流程开始。

**3. 计算圆周几何形状**

圆周上的点由 `circlePoint(cfg, theta)` 计算。

它首先根据 `inclination` 和 `azimuth` 计算圆平面的法向量：

```cpp
n = [
  sin(inclination) * cos(azimuth),
  sin(inclination) * sin(azimuth),
  cos(inclination)
]
```

然后构造圆平面内的两个正交方向 `u`、`v`，最终得到：

$$
p(\theta)
=
center
+
radius
\left(
\cos(\theta)u+\sin(\theta)v
\right)
$$

默认情况下：

- `inclination = 0`
- `azimuth = 0`
- 圆平面水平
- `u` 基本沿 X 轴
- `v` 基本沿 Y 轴

因此默认轨迹近似为：

$$
x = c_x+r\cos(\theta)
$$

$$
y = c_y+r\sin(\theta)
$$

$$
z = c_z
$$

几何计算代码在 `circle_motion_node.cpp:286`。

**4. 先移动到圆周起点**

每次开始运动时，节点先计算：

```cpp
const Vec3 p0 = circlePoint(cfg, 0.0);
```

然后读取机械臂当前姿态，并固定末端姿态：

```cpp
auto current_pose = mgi.getCurrentPose(ee_link_);
const auto keep_orientation = current_pose.pose.orientation;
```

这意味着：

- 末端位置沿圆周变化
- 末端姿态保持启动时不变

随后将末端目标设置为圆周起点：

```cpp
mgi.setPoseTarget(start_pose, ee_link_);
mgi.move();
```

这一步是普通 MoveIt 目标位姿规划，不是圆周运动本身。

对应代码在 `circle_motion_node.cpp:216`。

**5. 生成一整圈的笛卡尔路径**

每圈生成 `64` 个路径点：

```cpp
for (int i = 1; i <= waypoints_per_rev_; ++i) {
  theta = 2*pi*i/waypoints_per_rev_;
  p = circlePoint(cfg, theta);
}
```

也就是：

```text
theta = 2π/64
theta = 2π*2/64
...
theta = 2π
```

每个点都使用相同的 `keep_orientation`，所以姿态不变。

然后调用：

```cpp
mgi.computeCartesianPath(
    waypoints,
    eef_step_,
    0.0,
    traj);
```

MoveIt 会根据这些末端笛卡尔路径点求解逆运动学，生成对应的关节轨迹。

这里：

- `waypoints` 定义了末端应该经过的位置
- `eef_step = 0.005 m` 控制笛卡尔插补步长
- `jump_threshold = 0.0` 表示禁用关节跳变阈值检查
- 返回值 `fraction` 表示路径成功覆盖比例

如果覆盖率低于 `99%`，节点停止，不执行这一圈。

对应代码在 `circle_motion_node.cpp:307`。

**6. 根据速度给轨迹加时间**

路径生成后，节点计算目标周期。

角速度模式：

$$
T=\frac{2\pi}{\omega}
$$

默认角速度为 `0.5 rad/s`：

$$
T=\frac{2\pi}{0.5}\approx12.57s
$$

线速度模式：

$$
T=\frac{2\pi r}{v}
$$

例如半径 `0.05 m`、线速度 `0.05 m/s`：

$$
T=\frac{2\pi \times 0.05}{0.05}\approx6.28s
$$

MoveIt 先使用 `IterativeParabolicTimeParameterization` 根据关节速度、加速度限制生成时间戳，然后把整个轨迹的时间整体缩放到目标周期。

缩放后：

- 时间乘以 `k`
- 速度除以 `k`
- 加速度除以 `k²`

因此路径形状不变，只改变运动速度。

对应代码在 `circle_motion_node.cpp:329`。

**7. 执行一圈并重复**

轨迹通过：

```cpp
mgi.execute(traj);
```

发送给 MoveIt 执行器，再由 MoveIt 转发给机械臂的 FollowJointTrajectory 控制器。

节点使用异步执行：

```cpp
auto fut = std::async(
    std::launch::async,
    [&mgi, &traj]() { return mgi.execute(traj); });
```

执行期间每 `100 ms` 检查一次 `enabled_`：

```cpp
if (!enabled_.load())
  mgi.stop();
```

所以发布：

```bash
ros2 topic pub --once /circle_motion/enable \
  std_msgs/msg/Bool "{data: false}"
```

会停止当前轨迹。

当前圈执行成功后：

```cpp
++rev;
publishState(cfg, rev);
```

然后重新读取参数、重新规划下一圈。

因此参数更新具有以下效果：

- 半径、圆心、倾角、方位角：下一圈生效
- 角速度或线速度：下一圈生效，并切换速度模式
- `enable=false`：尝试立即停止当前圈

**整体执行链**

```text
启动 launch
  -> 创建 circle_motion 节点
  -> 加载机器人模型、SRDF、运动学插件
  -> 等待 /circle_motion/enable=true
  -> 读取当前末端姿态
  -> 计算圆周起点
  -> MoveIt 移动到起点
  -> 计算 64 个圆周路径点
  -> computeCartesianPath 求解笛卡尔轨迹
  -> 时间参数化
  -> 按目标周期缩放
  -> mgi.execute() 执行一圈
  -> 发布状态
  -> 重新规划下一圈
```

启动文件 `circle_motion.launch.py` 负责加载机器人描述和 MoveIt 配置；实际的圆周计算与轨迹执行都在 `circle_motion_node.cpp` 中完成。