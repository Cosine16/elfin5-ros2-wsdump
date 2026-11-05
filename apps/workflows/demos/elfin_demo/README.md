# elfin_demo

Elfin 机械臂 ROS2 控制接口示例包，移植自官方资料 `ros_demo.zip` 中的
`elfin_ros_demo`（ROS1/rospy → ROS2/rclpy），接口名已替换为本工作区
（`document/API_description.md`）的 ROS2 实际名称。

## ROS1 → ROS2 接口对照

| 功能 | ROS1 名称 | ROS2 名称（本包使用） | 类型 |
|---|---|---|---|
| 关节角度运动 | `elfin_basic_api/joint_goal` | `/joint_goal` | sensor_msgs/JointState |
| 空间位置运动 | `elfin_basic_api/cart_goal` | `/cart_goal` | geometry_msgs/PoseStamped |
| 空间位置组运动 | `elfin_basic_api/cart_path_goal` | `/cart_path_goal` | geometry_msgs/PoseArray |
| 轨迹直发 | `elfin_arm_controller/command` | `/elfin_arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory |
| 关节状态 | `elfin_arm_controller/state` | `/elfin_arm_controller/state` | control_msgs/JointTrajectoryControllerState |
| 使能状态 | `elfin_ros_control/elfin/enable_state` | `/enable_state` | std_msgs/Bool（仅真机） |
| 使能/去使能 | `elfin_basic_api/(dis)enable_robot` | `/elfin_basic_api/(dis)enable_robot` | std_srvs/SetBool（仅真机） |
| 停止运动 | `elfin_basic_api/stop_teleop` | `/stop_teleop` | std_srvs/SetBool |
| 读 DI | `.../io_port1/read_di` | `/read_di` | elfin_robot_msgs/ElfinIODRead（仅真机） |
| 写 DO | `.../io_port1/write_do` | `/write_do` | elfin_robot_msgs/ElfinIODWrite（仅真机） |

## 示例清单（scripts/）

| 脚本 | 作用 | 仿真可用 | 真机可用 |
|---|---|---|---|
| `joint_goal_demo.py` | 全关节转到 1.57 rad | ✔ | ✔ |
| `cart_goal_demo.py` | 末端移到 (0.42, 0, 0.445) 法兰朝下 | ✔ | ✔ |
| `cart_path_goal_demo.py` | 末端直线向 +Y 平移 10cm | ✔ | ✔ |
| `elfin_path_command.py` | 绕过规划直发轨迹（全关节 0.3 rad） | ✔ | ⚠ 无碰撞检查 |
| `get_robot_state.py` | 打印各关节位置/速度 | ✔ | ✔ |
| `get_enable_state.py` | 打印使能状态 | ✘（无人发布） | ✔ |
| `elfin_enable_demo.py` | 使能伺服 | ✘（返回 no real driver） | ✔ |
| `elfin_disable_demo.py` | 去使能（断电前必做） | ✘ | ✔ |
| `stop_teleop_demo.py` | 急停在当前位置 | ✔ | ✔ |
| `read_di_demo.py` / `write_do_demo.py` | IO 从站读写 | ✘ | ✔ |

## 编译

```bash
cd /path/to/ctl-elfin5
bash scripts/build_vendor.sh
bash scripts/build_apps.sh --packages-select elfin_demo
source scripts/elfin_env.sh
```

## 跑仿真

```bash
# 终端 1：Gazebo + MoveIt2 + 控制器
~/cos_ws/scripts/start_sim.sh moveit2

# 终端 2：basic_api（/joint_goal /cart_goal /cart_path_goal /stop_teleop 的提供方）
source ~/cos_ws/scripts/elfin_env.sh
ros2 launch elfin5_ros2_moveit2 elfin5_basic_api.launch.py

# 终端 3：跑示例（仿真无需使能）
source ~/cos_ws/scripts/elfin_env.sh
ros2 run elfin_demo joint_goal_demo.py
ros2 run elfin_demo cart_goal_demo.py
ros2 run elfin_demo cart_path_goal_demo.py
ros2 run elfin_demo elfin_path_command.py   # 不经 MoveIt，直发控制器
ros2 run elfin_demo get_robot_state.py      # 持续打印关节状态，Ctrl+C 退出
ros2 run elfin_demo stop_teleop_demo.py     # 运动中急停
```

> 只想起一个终端做快速验证时，可跳过 basic_api，直接用
> `elfin_path_command.py`（只依赖 JTC，仿真主 launch 已含）。

## 跑真机

```bash
# 终端 1~4：一键拉起（硬件驱动 + MoveIt + basic_api + 面板）
python3 ~/cos_ws/scripts/start_real.py

# 终端 5：先清错，再使能，确认状态
export ROS_LOCALHOST_ONLY=1   # 与 start_real 的终端保持一致
source ~/cos_ws/scripts/elfin_env.sh
ros2 service call /clear_fault std_srvs/srv/SetBool "{data: true}"
ros2 run elfin_demo elfin_enable_demo.py     # 应返回 "robot is enable"
ros2 run elfin_demo get_enable_state.py      # 应打印 robot enable

# 然后跑运动示例（建议顺序：先看状态，再小幅运动）
ros2 run elfin_demo get_robot_state.py
ros2 run elfin_demo cart_goal_demo.py        # 推荐首个真机动作（经碰撞检查）
ros2 run elfin_demo joint_goal_demo.py

# 结束：先停止运动、去使能，再断电
ros2 run elfin_demo stop_teleop_demo.py
ros2 run elfin_demo elfin_disable_demo.py
```

真机注意事项：

- 真机首动建议用 `cart_goal_demo.py`（目标点在工作空间中部、经 MoveIt
  规划与碰撞检查）；`elfin_path_command.py` 绕过规划，慎用。
- `write_do_demo.py` 会置位 DO（官方示例值 0x2000），运行前确认接线。
- 机械臂未使能时发 joint/cart 目标不会动；有 fault 时需先 `/clear_fault`。
