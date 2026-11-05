> 本文保留历史排查或方案记录，原路径与当时环境不改写。当前源码、构建和运行入口见 [目录规范](repository_layout.md) 与 [厂家 fork 说明](../third_party/elfin_robot_ros2.md)。

# Elfin5 仿真迁移指南：Gazebo Classic 11 → Gazebo Harmonic

> 分工约定：本文档是"教练手册"。实施者（你）按阶段动手，每个阶段有明确验收标准；
> 卡住时带着现象和日志来问。目标版本 **Gazebo Harmonic (gz-sim 8, LTS → 2029-05)**，
> 配合 ROS 2 Humble / Ubuntu 22.04（非默认配对，官方支持）。
>
> 迁移全程**不改 Classic 版任何文件**，新包并存，随时可回退。
> MoveIt / RViz / basic_api / 面板 / cos_shape 全部不动——它们只看话题层。

---

## 0. 背景知识（先建立心智模型）

### 新旧架构的本质区别

| | Classic 11 | Harmonic (gz-sim) |
|---|---|---|
| 架构 | 单体 gzserver + 杂烩 plugin | **一切皆 System**，插件化挂载到 world / model / sensor |
| 进程 | gzserver + gzclient 两个进程 | `gz sim` 单进程（GUI 是一个 system） |
| 内部通信 | gz-transport（旧版） | gz-transport（新版，话题/服务） |
| ROS 桥 | gazebo_ros 插件直接混进仿真进程 | **ros_gz_bridge**：gz-transport ⇄ ROS 2 双向桥，仿真进程不依赖 ROS |
| 机器人控制 | libgazebo_ros2_control.so 模型插件 | gz_ros2_control（GazeboSimSystem），仍是模型插件 |
| 场景描述 | SDF 1.6 + `model://` 本地模型库 | SDF 1.7+ + Fuel 网络模型库（或本地路径） |
| 渲染 | OGRE1 | OGRE2（材质写法完全不同） |

### 关键学习资源（按阶段用到再查）

- 官方迁移指南: https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/
- gz-sim 教程（system/plugin 机制）: https://gazebosim.org/docs/harmonic/tutorials/
- ros_gz 仓库（README + demos）: https://github.com/gazebosim/ros_gz （humble 分支）
- gz_ros2_control（README + demos，最有价值的对照样本）: https://github.com/ros-controls/gz_ros2_control （humble 分支）
- 控制文档: https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html

### 本项目 Classic 绑定点清单（迁移时要逐一替换的 4 处）

1. `elfin5_ros2_gazebo/urdf/elfin5_gazebo.xacro:9-14` — `libgazebo_ros2_control.so` + `GazeboSystem`
2. `elfin5_ros2_moveit2/launch/elfin5.launch.py:49-53,71` — `gazebo_ros` 的 launch + `spawn_entity.py`
3. `elfin5_ros2_gazebo/worlds/elfin5.world` — SDF 1.6 + `model://sun`/`model://ground_plane`
4. `elfin5_gazebo.xacro` 末尾 — `gazebo.material` OGRE1 材质脚本引用

---

## 1. 环境准备（Stage 0 前置，需要 sudo）

### 1.1 装 Harmonic

```bash
# 加 OSRF 源
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic
```

验证：`gz sim --version` 应显示 8.x。

> Classic 不用卸载，两者可共存。但注意：**不要在同一个终端里混用**，环境变量会串。
> 建议单独开一个终端（或 alias）跑新仿真，保持 `source /opt/ros/humble/setup.bash` 干净起步。

### 1.2 源码构建 ros_gz + gz_ros2_control

Humble 的 apt 二进制 `ros-humble-ros-gz*` 是 Fortress 配对版，**不能装**（会和源码版冲突）。
新建独立 overlay 工作空间（与 elfin_ws 平级）：

```bash
mkdir -p ~/project/elfin5_station/gz_ws/src && cd ~/project/elfin5_station/gz_ws/src
git clone -b humble https://github.com/gazebosim/ros_gz.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
cd ~/project/elfin5_station/gz_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src -y -r   # 缺啥装啥，需要 sudo 时输密码
export GZ_VERSION=harmonic              # 关键！编译期选择 Gazebo 版本
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> WSL2 注意：rosdep 若缺系统包会要求 sudo。构建出的 `gz_ws/install` 之后
> 按 `humble → gz_ws → elfin_ws → .build/apps` 的顺序 source。

### 1.3 Stage 0 验收标准

- [ ] `gz sim /usr/share/gz-harmonic/.../shapes.sdf`（或 `gz sim shapes.sdf`）在 WSLg 下弹出 GUI 并能拖动视角
- [ ] 另开终端 `gz topic -l` 能列出 `/world/shapes/clock` 等话题 → 理解 gz-transport 是独立于 ROS 的第二张网
- [ ] `gz topic -e -t /world/shapes/clock` 有数据流

**WSL2 已知坑**：Harmonic 用 OGRE2 渲染，llvmpipe 软渲染下空场景流畅、复杂场景掉帧；
可以尝试 `GALLIUM_DRIVER=d3d12 gz sim ...` 看能否走 WSLg 的 GPU 直通（未验证项，试出结果请记录）。
gz-transport 发现走组播，若 `gz topic -l` 为空，检查 `GZ_IP`/`GZ_RELAY`，WSL2 一般默认即可。

---

## 2. Stage 1：最小新包 `elfin5_gz_sim`——让机器人在 Harmonic 里"站起来"

学习目标：world 的 system 声明、ros_gz_sim 的 launch 与 create 服务、URDF 生成链路。

### 2.1 建包

在 `elfin_ws/src/elfin_robot/elfin5/` 下新建 `elfin5_gz_sim`（ament_cmake，纯数据包，
仿照 `elfin5_ros2_gazebo` 的 CMakeLists/package.xml，把 launch/worlds 目录装上）。
**package.xml 记得补 exec_depend**（ros_gz_sim、ros_gz_bridge、gz_ros2_control、
robot_state_publisher、xacro）——Classic 版一个依赖没写，别学它。

### 2.2 写 world：`worlds/elfin5.world`

对照 Classic 的 `model://sun` + `model://ground_plane`，gz-sim 要显式声明系统：

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="elfin5_world">
    <plugin filename="gz-sim-physics-system"      name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri></include>
  </world>
</sdf>
```

> 想彻底搞懂，故意删掉 Physics 系统看现象（模型悬空不动）、删掉 UserCommands 看现象
> （无法 spawn/删除模型）。这是理解 "一切皆 System" 最快的办法。
> Fuel 模型首次加载要联网下载（缓存到 `~/.gz/fuel`）；WSL2 网络正常即可。
> 若不想依赖网络，可在 world 里内联 sun/ground_plane 的 SDF。

### 2.3 写 launch：`launch/elfin5_gz.launch.py`（先不接控制器）

镜像 `elfin5.launch.py` 的结构，替换点：

| Classic 写法 | Harmonic 写法 |
|---|---|
| `gazebo_ros/launch/gazebo.launch.py` + world 参数 | `ros_gz_sim/launch/gz_sim.launch.py`，参数 `gz_args:=<world路径>` |
| `gazebo_ros/spawn_entity.py -topic robot_description` | `ros_gz_sim` 的 `create` 节点，参数 `-topic robot_description -name elfin5` |
| `/clock` 由 gazebo_ros_init 直发 | `ros_gz_bridge` 的 `parameter_bridge` 节点：`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` |
| （无对应） | 可选 `ros_gz_sim` 的 `ros_gz_bridge` 桥 `/world/elfin5_world/pose/info` 等 |

robot_state_publisher、static_tf 部分原样照抄（这部分正好观察：它们对仿真器完全无感）。

### 2.4 Stage 1 验收标准

- [ ] `ros2 launch elfin5_gz_sim elfin5_gz.launch.py` 后 GUI 里出现 elfin5（可能因重力瘫倒，正常——没有控制器）
- [ ] `ros2 topic echo /clock --once` 有数据（桥工作正常）
- [ ] `gz topic -l | grep elfin5` 能看到模型相关 gz 话题
- [ ] 能用自己的话解释：机器人模型是怎么从 xacro 流进 gz-sim 的（xacro → robot_description 话题 → create 服务 → UserCommands 系统）

---

## 3. Stage 2：接入 gz_ros2_control——让控制器 active

学习目标：hardware_interface 在新架构下的形态、gz_ros2_control 插件机制。

### 3.1 改 xacro（新建 `elfin5_gz_sim/urdf/elfin5_gz.xacro`，别动原文件）

对照 `elfin5_gazebo.xacro` 逐行改写：

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find elfin5_ros2_gazebo)/config/elfin_arm_controller.yaml</parameters>
  </plugin>
</gazebo>

<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <!-- 6 个 joint 的 command/state interface 原样照抄，Classic 的这部分完全兼容 -->
</ros2_control>
```

材质引用（`gazebo.material` 脚本）改成 SDF 内联写法，或先整段删掉（不影响功能）：

```xml
<gazebo reference="elfin_link2">
  <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
</gazebo>
```

### 3.2 launch 里加控制器 spawner

`gz_ros2_control` 插件在 gz-sim 进程内创建 controller_manager（与 Classic 版机制相同，
`elfin5.launch.py:125-128` 的注释说的就是这个设计）。launch 里用普通
`controller_manager/spawner` 节点按序拉起 `joint_state_broadcaster` → `elfin_arm_controller`，
事件链写法照抄现有 launch 的 `OnProcessExit` 模式——顺手把 Classic 版
"两套加载逻辑互相竞态" 的冗余改掉，只留一套。

### 3.3 Stage 2 验收标准

- [ ] `ros2 control list_controllers` 两个控制器均 active
- [ ] `ros2 topic echo /joint_states --once` 有 6 关节数据
- [ ] 机器人在 GUI 里**直立不瘫**（位置接口 hold 住零位）——这是和 Stage 1 最直观的区别
- [ ] `ros2 topic pub` 一条 JTC 轨迹（或 `ros2 action send_goal /elfin_arm_controller/follow_joint_trajectory`）能让臂动

---

## 4. Stage 3：接通 MoveIt——复刻 elfin5.launch.py 全链路

学习目标：use_sim_time 全链路、MoveIt 与仿真时钟的耦合。

### 4.1 写 `elfin5_gz_moveit.launch.py`（或并入 elfin5_gz_sim 的 launch，二选一）

MoveIt 部分（move_group 参数、ompl yaml、kinematics yaml、RViz 节点）从
`elfin5.launch.py:144-278` **大段照抄**——这正是观察"仿真器更换对规划层透明"的好机会。

### 4.2 顺手修两个已知问题（见《已知问题.md》）

- **#9**：`use_sim_time` launch 参数声明了但没传给节点 → 新 launch 里统一传
  `{'use_sim_time': LaunchConfiguration('use_sim_time')}` 给所有节点
- **#16**：Classic 下 `/clock` 有 publisher 无数据 → Harmonic 下由 parameter_bridge 保证，
  用 `ros2 topic hz /clock` 验收

### 4.3 Stage 3 验收标准

- [ ] RViz 里拖 goal → Plan & Execute，Gazebo 里的臂同步运动
- [ ] `ros2 topic hz /clock` 有频率（~1000Hz 或按 world 配置）
- [ ] move_group 日志无 TF 时间戳告警（说明 use_sim_time 链路通了）

---

## 5. Stage 4：回归 + 收尾

- [ ] `circle_motion` 在 .build/apps 下回归（`scripts/circle_sim.sh` 改指向新 launch，或加 SIM 后端切换开关）
- [ ] 性能记录：WSLg + llvmpipe 下 Gazebo 实时因子（GUI 右下角 RTF），与 Classic 版对比
- [ ] 文档更新：《架构与数据流.md》仿真章节、《已知问题.md》#6/#9/#16 状态
- [ ] 根 README / document/README_cn.md 补 Harmonic 启动命令

---

## 6. 常见坑速查（WSL2 + Harmonic 特有）

| 现象 | 原因 | 处理 |
|---|---|---|
| ~~`gz sim` 黑屏~~ | ~~OGRE2 + 软渲染~~ | **2026-10-11 实测推翻**：llvmpipe 下场景/形状渲染正常，见下行 |
| GUI 工具栏图标显示为黑色小方块（实测） | Qt5 Quick 材质图标在 llvmpipe 软渲染下的外观缺陷 | 纯外观问题，不影响功能；待 `wsl --update` 修好 d3d12 直通后复查 |
| `GALLIUM_DRIVER=d3d12` 后 GL 上下文创建失败崩溃（实测） | 本机 WSLg 的 D3D12 直通处于损坏状态 | Windows 侧 `wsl --update` + 更新显卡驱动后重试 |
| `QT_QUICK_BACKEND=software` 段错误（实测） | gz-gui 的 `RenderWindowItem` 必须持有真实 GL 上下文，软件 Qt 后端与之冲突 | 此路不通，不要试 |
| `gz topic -l` 空 | gz-transport 组播发现问题 | 同机检查 `GZ_IP=127.0.0.1`；跨机才需要 relay |
| create 节点报超时 | world 里没声明 UserCommands 系统 | 回 2.2 检查 |
| 控制器 load 失败找不到 GazeboSystem | `GZ_VERSION=harmonic` 没 export 就编译了 gz_ros2_control | 重编 gz_ws |
| ros_gz 桥找不到话题 | 消息类型映射写错 | `ros2 run ros_gz_bridge parameter_bridge --help` 查语法；`gz topic -l` 对照 |
| Fuel 模型下载慢/失败 | 网络问题 | 内联 SDF 替代，或预先 `gz fuel download` |

## 7. 分支管理建议

```bash
# 当前仓库处于 detached HEAD (61af126)，先落分支再动手
git checkout -b build/adapt_wsl2_env      # 环境适配：WSL2 文档、残留清理记录
git checkout -b feat/gz-harmonic-migration # 迁移本体：elfin5_gz_sim 包 + 文档
```

- 环境结论（WSLg/llvmpipe/sudo 清单）进 `build/adapt_wsl2_env`
- 每个 Stage 一个 commit，commit message 里记现象和坑，这就是你的学习笔记
- gz_ws 是否纳入本仓库管理待定（它是第三方源码 overlay，也可放外层不进 git）
