# 自研应用与集成

本目录按能力和职责组织源码；构建方式、Python 环境和部署位置由各模块自行声明。

| 目录 | 职责 | 当前内容 |
| --- | --- | --- |
| [sensors/realsense](sensors/realsense/) | 相机接入、模型和数据获取 | `cos_realsense` |
| [robot](robot/) | 自研公共机械臂控制与机器人配置 | 暂无独立公共包，厂家能力由 `third_party/elfin_robot_ros2` 引入 |
| [bridges/mcp](bridges/mcp/) | Agent 协议适配 | Elfin5 MCP 服务 |
| [policies/smolvla](policies/smolvla/) | 模型推理与动作映射 | SmolVLA 集成应用 |
| [workflows](workflows/) | 组合多项能力完成任务 | 画图、视觉跟随、手眼标定、接口示例 |
| [bringup](bringup/) | 跨应用的公共启动与场景配置 | 预留职责，现有启动文件随原包保留 |

ROS 包在这些目录下通过各自的 `package.xml` 被发现，不额外套一层 `src/`。
MCP 和 SmolVLA 保留独立 Python 环境，其父目录的 `COLCON_IGNORE` 防止 colcon
扫描应用虚拟环境。`apps/` 本身不保存 build/install/log。

构建与运行入口在 [仓库说明](../README.md)，迁移映射及边界见
[目录规范](../document/repository_layout.md)。包名、节点名、ROS 接口和模型文件内容
在本轮迁移中保持稳定。

现有 `cos_realsense` 内仍含标定工具，`cos_rsvisual` 和 `elfin5_handeye` 内仍含
各自的相机/场景组合。这里先按包的主要职责整体归位；后续根据实际复用需要再提取
公共传感器、算法和 bringup 包，避免同时改变包结构和运行行为。
