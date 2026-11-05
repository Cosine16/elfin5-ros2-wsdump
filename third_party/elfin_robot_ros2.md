# Elfin5 厂家 fork 集成记录

## 来源与固定版本

- 源码位置：[elfin_robot_ros2](elfin_robot_ros2/)，Git 子模块。
- 厂家来源：`huayan-robotics/elfin_robot_ros2`，依据 fork 的 [STATION.md](elfin_robot_ros2/STATION.md)。
- 引入 fork：`git@github.com:Cosine16/elfin_robot_ros2.git`。
- 本次保留提交：`23b89ab5ace051e770d4a5463afae2325d083752`。
- 当前本地分支：`feat/gz-harmonic-migration`；迁移未切换、合并或拉取分支。
- 记录的厂家基线：`humble_ethercat@1381e3f6d404eaf3311cae4311641e4d8f369b1a`。

主仓库 gitlink 固定源码提交，[revision 文件](elfin_robot_ros2.revision) 供构建前校验。
更新依赖时一并审查这两个记录及补丁；不在构建中自动跟踪远端分支。
`.gitmodules` 不再用 `branch=humble-station` 暗示当前版本来自该分支。
Git 子模块的逻辑标识仍保留原名 `elfin_ws/src/elfin_robot`，实际路径已更新；
这是为保留现有本地 Git 元数据，新的检出仍按新路径初始化。

本次检查使用本地历史，未联网核验远端引用，也未推送或创建 fork 提交。

## 历史改动边界

相对记录的厂家基线，当前提交链含 12 次提交，包括文档及机型裁剪：

| 类型 | 代表提交 | 内容 |
| --- | --- | --- |
| 构建修复 | `c89e54a`、`083d50f`、`1a5c0e6` | SOEM 编译、MoveIt 参数命名、日志格式 |
| 工位参数与硬件适配 | `81de7e4`、`c2db9e6`、`75d51cb` | 参数覆盖、EtherCAT 拓扑与 IO、原点标定 |
| 点动控制 | `1cce70d`、`04034f6` | 显式取消 goal、异步句柄接管 |
| 产品裁剪 | `a25e49b` | 保留 Elfin5，移除其他机型 |
| Harmonic 仿真 | `23b89ab` | 新增 `elfin5_gz_sim` |

本地缓存的 `origin/humble-station` 为另一条分支，包含 `e954507`、`913c6ea`
两次启动相关提交；本次没有将它们合并进当前版本。
这些记录说明补丁来源，不表示本次对历史改动重新进行了硬件验收。

## 主仓库维护的构建补丁

[0001-export-driver-library.patch](patches/elfin_robot_ros2/0001-export-driver-library.patch)
仅修改两个 CMake 文件：

1. 安装并通过 ament 导出 `elfin_ethercat_driver` 的共享库、头文件和依赖。
2. `elfin_ros_control` 从已安装的包获取依赖，移除 `../../../install`、
   `../../../build` 和裸 `.so` 链接；移除指向旧构建目录的 RPATH。

应用方式：

```bash
bash scripts/prepare_vendor.sh --check  # 检查已应用或可应用
bash scripts/prepare_vendor.sh          # 应用未应用的补丁；可重复执行
bash scripts/build_vendor.sh            # 构建前自动执行上述检查/应用
```

补丁文件由主仓库版本化，fork 的 HEAD 保持不变。准备后子模块中两个 CMake 文件
会显示为本地修改，这是明确记录的集成补丁。版本不匹配或补丁冲突时脚本失败，
不会 reset、覆盖其他改动或切换提交。后续若将修复提交到 fork，应同步更新固定版本
并移除已经吸收的补丁。

本次不改硬件配置、编码器原点、扭矩系数、EtherCAT 状态处理、点动逻辑或仿真后端。
公共 bringup 与工位配置的进一步提取仍是独立工作。

## 源码、构建与示例

- 厂家构建只扫描 `third_party/elfin_robot_ros2`，输出到 `.build/vendor`。
- 项目移植的 [elfin_demo](../apps/workflows/demos/elfin_demo/) 由应用层构建，包名保留。
- Harmonic 配置从同一厂家源码选取所需包，独立输出到 `.build/handeye`。
- 原工作空间构建树和缓存保留在 `.build/legacy/elfin_ws`，不作为运行 underlay。
- 迁移后需要重新构建；当前环境未安装 ROS，完整 ROS 构建、仿真和真机运行尚未验证。
