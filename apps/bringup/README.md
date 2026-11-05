# 公共运行编排

用于跨应用复用的机器人、相机、仿真和运行配置。
现阶段启动文件仍随原 ROS 包安装，仓库级入口放在 [scripts](../../scripts/)。

Harmonic 仿真与标定通过 `scripts/build_handeye.sh` 和 `scripts/run_handeye.sh`
构建、启动；SmolVLA 的 `run_gz_sim.sh` 委托同一个入口，避免维护第二套路径。
这仍复用了手眼包中的场景，公共相机/仿真包的提取尚未实施。
