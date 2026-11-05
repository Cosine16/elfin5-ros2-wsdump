# 源码包清单

自研源码按职责归入 `apps/`，目录名不改变 ROS 包名。

| ROS 包 | 源码位置 | 构建配置 |
| --- | --- | --- |
| cos_realsense | [相机能力](../apps/sensors/realsense/cos_realsense/) | Classic / 真机应用 |
| cos_shape | [画图流程](../apps/workflows/drawing/cos_shape/) | Classic / 真机应用 |
| cos_rsvisual | [视觉跟随](../apps/workflows/visual_following/cos_rsvisual/) | Classic / 真机应用 |
| elfin5_handeye | [手眼标定](../apps/workflows/handeye/elfin5_handeye/) | Humble / Harmonic |
| elfin_demo | [接口示例](../apps/workflows/demos/elfin_demo/) | Classic / 真机应用 |

独立 Python 应用：[MCP](../apps/bridges/mcp/)、[SmolVLA](../apps/policies/smolvla/)。
它们的父目录有 `COLCON_IGNORE`，由各自的虚拟环境运行。

厂家层 `third_party/elfin_robot_ros2` 中的包包括 `elfin_description`、`elfin5_gz_sim`、
`elfin5_ros2_moveit2`、`elfin5_ros2_gazebo`、`soem_ros2`、`elfin_ros_control`、
`elfin_robot_msgs`、`elfin_robot_bringup`、`elfin_ethercat_driver`、
`elfin_basic_api`，以当前子模块内容为准。`elfin_demo` 是本项目移植维护的示例，
已由应用层构建。

可在仓库根目录检查自研 ROS 包发现：

```bash
colcon --log-base .build/discovery/log list --base-paths apps
```
