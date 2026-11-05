# Elfin5 MCP 接入

把 Elfin5 ROS 状态、基础控制和 IO 能力暴露为 stdio MCP 工具。
机械臂后端需单独启动；本应用通过 `elfin_basic_api` 请求运动，不负责启动驱动或模型推理。

在仓库根目录准备独立环境（ROS 2 Humble 的系统 Python）：

```bash
python3 -m venv --system-site-packages apps/bridges/mcp/.venv
apps/bridges/mcp/.venv/bin/python -m pip install -r apps/bridges/mcp/requirements.txt
bash apps/bridges/mcp/run_server.sh
```

MCP 客户端的 `command` 更新为当前仓库内 `apps/bridges/mcp/run_server.sh` 的绝对路径。
启动脚本从自身位置定位仓库，并加载 ROS 与 `.build/vendor/install` 的厂家消息环境；
`VENDOR_BUILD_DIR` 可覆盖厂家构建目录，stdout 保留给 MCP 协议。
`ROS_SETUP` 可覆盖系统 ROS setup 文件，`ROS_DOMAIN_ID` 由调用方传入。

迁移时已保留本地 `.venv` 并更新入口路径。新检出或更换 Python 解释器时应重新创建环境。
