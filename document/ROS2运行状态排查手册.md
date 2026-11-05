# Elfin ROS2 运行状态排查手册

> 适用环境: Ubuntu 20.04 + ROS2 Foxy + Fast DDS, 实机驱动以 root 运行, 调试终端为普通用户 fit。
> 2026-08-22 在上次硬件测试的遗留现场上实测整理。

## 0. 先备好环境(每次开终端第一件事)

```bash
source ~/cos_ws/scripts/elfin_env.sh
```

当前入口依次加载 Humble、`.build/vendor/install/setup.bash`、`.build/apps/install/setup.bash`，
并设置 `ROS_LOCALHOST_ONLY=1`。历史排查的 Foxy 环境仅作为背景记录。
第三项必须和 `start_real.py` / `start_sim.py` 里的设置一致, 否则看不到它们启动的节点。

需要 root 身份调试时:

```bash
bash ~/cos_ws/scripts/root_ros.sh                 # 环境已配好的 root shell
bash ~/cos_ws/scripts/root_ros.sh ros2 topic list # 以 root 跑单条命令
```

## 1. 排查方法论: 分层验证

ROS2 的通信分三层, 出问题时**按层定位**, 不要混在一起猜:

| 层 | 验证手段 | 通过标准 |
|---|---|---|
| 发现 (discovery) | `ros2 node/topic/service list` | 名单完整 |
| 话题数据流 | `ros2 topic echo / hz` | 有数据到达 |
| 服务调用 | `ros2 service call` | 有 response |

关键结论(2026-08-22 实测): **root ↔ fit 跨用户的发现、话题、服务全部正常**。
"普通用户 rqt 看不到东西"的真实原因不是权限, 而是下面 §5 的几个坑。

## 2. 常用命令速查

### 2.1 进程层: 谁还活着, 以什么身份跑

```bash
ps -eo pid,user,rtprio,cmd | grep -E 'ros2|rviz|gazebo|move_group|controller_manager|elfin' | grep -v grep
```

上次实机启动残留的进程(全部 root):

| PID | 进程 | 承载的节点 |
|---|---|---|
| 19123 | `ros2_control_node` | `/controller_manager`, `/elfin_hw`(驱动), 各 controller |
| 19119 | `static_transform_publisher` | world→elfin_base_link 静态 TF |
| 19360 | `rviz2` | `/rviz2` |
| 19362 | `move_group` | `/move_group`, `/moveit_simple_controller_manager` |
| 19915 | `elfin_basic_api_node` | `/elfin_basic_node`(teleop/motion 服务) |
| 19969 | `elfin_gui.py` | `/elfin_gui_node` 等(控制面板) |
| 15878 | `_ros2_daemon` (fit) | ros2 CLI 的守护进程, 见 §5 坑 1 |

### 2.2 发现层

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 service list | grep -E 'teleop|enable|fault|read_d|write_d'   # 只看 elfin 相关
```

### 2.3 归属与 QoS: 某个 topic 谁发谁收

```bash
ros2 topic info -v /enable_state
```

输出里 `Node name` 给出发布方/订阅方节点名, `QoS profile` 给出可靠性。
QoS 兼容规则: 订阅方要求 ≤ 发布方提供。RELIABLE 发布 + BEST_EFFORT 订阅 = 兼容; 反过来不兼容(收不到)。

### 2.4 数据流验证(要给足时间, 见 §5 坑 2)

```bash
ros2 topic echo /enable_state        # 驱动发布, ~10Hz, 使能状态
ros2 topic echo /fault_state         # 驱动发布, 故障状态
ros2 topic hz /joint_states          # 期望 ~50Hz(实测有抖动)
ros2 topic echo /tf --no-arr         # robot_state_publisher 发布
```

Foxy 没有 `ros2 topic echo --once`(Humble 才有), 用 `timeout 10 ros2 topic echo ... | head -20` 或 Ctrl+C。

### 2.5 服务验证(实机只调只读类!)

```bash
# 只读, 安全:
ros2 service call /get_motion_state std_srvs/srv/SetBool "{data: true}"
ros2 service call /get_reference_link std_srvs/srv/SetBool "{data: true}"
# 会改变机器人状态, 确认后再调:
ros2 service call /clear_fault std_srvs/srv/SetBool "{data: true}"
ros2 service call /enable_robot std_srvs/srv/SetBool "{data: true}"
```

`/enable_robot` 的 response.message 只有四种(见 `elfin_ethercat_driver.cpp:611`):
`please clear fault first` / `there is no ethercat client` / `robot is not enabled`(20s 超时) / `robot is enabled`。

### 2.6 节点详情: 某节点提供的全部接口

```bash
ros2 node info /elfin_basic_node     # basic_api 的 service/topic 全貌
ros2 node info /elfin_hw             # 驱动的 service/topic 全貌
```

## 3. Elfin 链路全景(排查时对照)

```
elfin_gui.py (控制面板, wxPython)
  │  service: /joint_teleop /cart_teleop /stop_teleop /home_teleop   (SetInt16/SetBool)
  ▼
elfin_basic_api_node (TeleopAPI, 用 MoveGroup 算目标点)
  │  action: /elfin_arm_controller/follow_joint_trajectory           (FollowJointTrajectory)
  ▼
elfin_arm_controller (ros2_control, 在 ros2_control_node 进程内)
  │  command interface (进程内内存, 非消息)
  ▼
ElfinHWInterface (elfin_ros_control 硬件插件)
  │  进程内 C++ 调用 PDO 读写; 每个控制周期 read() 里顺带 rclcpp::spin_some(elfin_hw 节点)
  ▼
ElfinEtherCATDriver → SOEM → EtherCAT → 机械臂

面板 ──service──▶ 驱动(旁路, 不过 basic_api):
  /enable_robot /disable_robot /clear_fault (SetBool)
  /read_di /read_do /write_do (ElfinIODRead/ElfinIODWrite)
驱动 ──topic──▶ 面板: /enable_state /fault_state (Bool, ~10Hz)

状态显示链: 驱动 read() → joint_state_controller ─/joint_states─▶ robot_state_publisher
  ─/tf─▶ rviz2 / 面板(TF 查 world→elfin_end_link 显示末端位姿)
其他: 面板速度滑条 ─/vel(Float32)─▶ basic_api;  /set_reference_link /set_end_link (SetString)
```

要点:
- Servo On 走的是 **service**, rqt_graph 画不出来, 只能用 `ros2 service call` 或面板验证。
- 驱动的 service/timer 靠 `read()` 里的 `spin_some` 泵动(`elfin_hardware_interface.cpp:280`),
  控制循环停了它们全停。
- `/enable_state` 发布有延迟: 订阅方首次收到数据可能要等 30s 以上(发现过程慢), 不是没发。

## 4. rqt 的正确打开方式

```bash
source ~/cos_ws/scripts/elfin_env.sh
rqt    # 或 rqt_graph
```

- `rqt_graph`: 只看 node↔topic; service/action 看不到。
- Plugins → Services → Service Caller: 图形化调 `/enable_robot` `/clear_fault`, 可代替面板隔离问题。
- Plugins → Topics → Topic Monitor: 盯 `/enable_state` `/fault_state`。
- `rqt_console`: 看 /rosout。root 节点的日志也能收到(发现层正常)。

Foxy 官方文档:
- CLI 工具: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html
- Services: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html
- Topics: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
- rqt: https://docs.ros.org/en/foxy/Concepts/About-RQt.html

## 5. 本次排查踩过的坑(重要)

1. **ros2 daemon 缓存**: `ros2 node list` 等来自后台 daemon(pid 15878)的缓存, 可能陈旧。
   现场对不上时先 `ros2 daemon stop && ros2 daemon start`, 再等几秒。
2. **窗口太短误判**: Foxy 下首次发现 + 端到端匹配在这台负载高的机器上要 30s 以上。
   `timeout 4 ros2 topic echo` 什么都收不到就下结论, 会误判成"数据不通"。给 60s。
3. **`bash -c 'source ... && 命令 &'` 的坑**: `&` 会把整条 `source && 命令` 链都丢到后台,
   前台 shell 没 source 到环境, 后续命令在错误的环境里跑。后台化时拆行写:
   `source ...; 命令A & 命令B`。
4. **管道缓冲**: `ros2 topic pub | head` 时 Python 标准输出是块缓冲, 进程被 timeout 杀掉后
   缓冲区丢失, "没输出"不代表"没发布"。写文件或不用管道。
5. **`pkill -f '关键字'` 会匹配到自己**: 执行 pkill 的 bash 命令行里若含同样关键字, 会自杀。
6. **sudo 环境**: sudo 清空环境变量, 所以 root 下必须重新 source(`root_ros.sh` 已封装)。
   但跨用户 DDS 通信本身没有问题, 不要再怀疑权限。
7. **网卡环境**: 本机 enp2s0 同时跑 EtherCAT(混杂模式)和 DHCP(192.168.137.x), 另有 tailscale0。
   多网卡下 Fast DDS 发现可能更慢, 这也是坑 2 的原因之一; `ROS_LOCALHOST_ONLY=1` 可规避。

## 6. 上次现场快照(2026-08-22, 实机连接中)

- `/enable_state` = false(未使能), `/get_motion_state` = 不动, 与"Servo On 失败"现象一致。
- `/joint_states` ~50Hz 但时间戳抖动明显(100ms 级), 疑似控制循环被阻塞或机器负载高, 待查。
- 节点名重复告警: `/elfin_basic_node` x4, `/rviz2` x2, 多个 transform_listener ——
  basic_api 和 rviz 内部创建了多个同名节点, 是无害但烦人的既有行为。
- 驱动 IO 服务 `/elfin_module_enable_slave2-4` 等在线, EtherCAT 从站扫描正常。

## 7. 下一步: Gazebo 仿真 debug

真机链路已验证畅通, 后续在仿真里复现/调试面板与驱动交互:

```bash
python3 ~/cos_ws/scripts/start_sim.py            # 起 Gazebo + MoveIt + basic_api + 面板(3 个终端)
```

仿真模式差异: 面板 `use_fake_robot=True`, Servo On/Off 改调 `/elfin_basic_api/enable_robot|disable_robot`
(只做 controller 切换, 不碰 EtherCAT), IO 轮询关闭; 轨迹 action 发给 Gazebo 的同名控制器。
排查命令与本文档完全相同, 只是进程都以普通用户运行, 没有 sudo 问题。
