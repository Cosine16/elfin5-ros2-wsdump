# RoboNeuron 适配 Ubuntu 22.04 / ROS 2 Humble / SmolVLA 分析与实施方案

## 1. 结论

**可以适配，建议继续，但不能把 RoboNeuron 当前默认安装流程直接照搬到本机。**

推荐路线是：保留 Ubuntu 22.04 和 ROS 2 Humble，以 Python 3.10 承载 ROS/RoboNeuron 边界，复用项目现有独立 SmolVLA 推理环境，通过 RoboNeuron 的子进程隔离方式接入模型；在父项目中实现 Elfin5 专用的关节空间动作适配器。不要把 `smolvla_base` 的输出直接当作 `EEFDeltaCommand`，也不要把 SO-101 的绝对动作直接下发给 Elfin5。

本结论分为两层：

- **工程链路可行**：Humble 消息接口已实际构建、导入成功；本机已有可识别 CUDA 的 SmolVLA 环境。
- **真实任务能力尚未具备**：预训练 `lerobot/smolvla_base` 未在 Elfin5 上微调，现有桥接只验证安全运动链路，不具备可靠任务语义。

## 2. 本次已完成事项与验证边界

### 2.1 子模块

已在父仓库将现有干净克隆注册为子模块：

- 路径：`third_party/RoboNeuron`
- 原始上游：`https://github.com/guanweifan/RoboNeuron.git`
- 引入 fork：`https://github.com/Cosine16/RoboNeuron.git`
- 上游分析基线：`5b792d59b8e8f870bf87b6ca0fd53a0c43fbf7eb`
- 当前固定提交：`6e93977a74cbba87be0b01c54533779851dc79ea`，包含 Humble 运行时配置支持。
- `.gitmodules` 与 gitlink 已提交到集成分支。
- SmolVLA 路线不需要 RoboNeuron 内部 OpenVLA/OpenVLA-OFT/OpenPI 嵌套源码；CI 只检出直接子模块，不为本任务额外拉取嵌套依赖。

### 2.2 本机环境实测

| 项目 | 实测状态 |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS |
| ROS | ROS 2 Humble，`ros2` CLI 可用 |
| 系统 Python | 3.10.12；未安装 `python3.12` |
| GPU | GeForce GTX 1050 Ti，4096 MiB（4 GB）显存，Compute Capability 6.1 |
| SmolVLA venv | Python 3.10.12、torch 2.6.0+cu124、lerobot 0.4.4、transformers 4.57.6 |
| CUDA 探针 | `torch.cuda.is_available() == True`，识别到 GTX 1050 Ti |
| uv | `/home/nanoctl/.local/bin/uv`；非登录命令的默认 PATH 中不可直接调用 |

ROS 2 官方文档明确列出 Humble 的 deb 包支持 Ubuntu Jammy 22.04：[ROS 2 Humble Ubuntu 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

### 2.3 已执行的轻量验证

1. 用 Python 3.10 对 `third_party/RoboNeuron/src` 和 `ros` 执行 `compileall`：通过。
2. 仅 source `/opt/ros/humble/setup.bash`，用 `colcon` 构建 `roboneuron_interfaces`：通过，耗时约 16.7 秒。
3. 在现有 SmolVLA venv 中导入 `rclpy`、`EEFDeltaCommand`、`RawActionChunk`、`TaskSpaceState`：通过。
4. 构建仅有 CMake 兼容性/策略警告，没有编译错误。
5. 本轮未运行完整 SmolVLA 推理、未启动 Gazebo、未运行 MoveIt、未操作真实机械臂。

因此，“Humble 完全不能运行 RoboNeuron”不成立；真正的阻塞在 Python 包声明、观测契约和动作语义。

## 3. 兼容性分析

### 3.1 Ubuntu 22.04 与 ROS 2 Humble：可保留

RoboNeuron 的 ROS 接口是纯 `rosidl` 消息包，`CMakeLists.txt` 只生成三个消息，没有 Jazzy 专属 API；本次已用 Humble 实际构建成功。参见：

- `third_party/RoboNeuron/ros/roboneuron_interfaces/CMakeLists.txt:24-30`
- `third_party/RoboNeuron/ros/roboneuron_interfaces/package.xml:10-21`

不建议为了 RoboNeuron 默认示例而升级到 Jazzy/Ubuntu 24.04，因为这会扩大现有 Elfin5 厂家栈、Gazebo Harmonic 配置和标定环境的迁移范围。

### 3.2 Python：需要 Humble 兼容分支，而非混用解释器

当前 `pyproject.toml` 强制 `>=3.12,<3.13`，而 Humble 系统 Python 是 3.10：

- `third_party/RoboNeuron/pyproject.toml:6`
- `third_party/RoboNeuron/README.md:103-110,231-236`

虽然源码通过 Python 3.10 语法编译，但这不等于依赖解析和测试已经通过。应在自有 RoboNeuron fork 中建立 `humble-smolvla` 分支，调整 Python 3.10 元数据并重新生成锁文件、运行单元测试；不要仅删除版本限制后就宣称兼容。

推荐环境拆分：

- **ROS/RoboNeuron 核心环境**：Python 3.10 + `--system-site-packages`，访问 Humble 的 `rclpy`、`cv_bridge` 和构建后的消息；不安装模型权重和重型 CUDA 依赖。
- **SmolVLA 模型环境**：复用 `apps/policies/smolvla/.venv`，保持 torch 2.6.0+cu124、lerobot 0.4.4、transformers 4.57.6。

不要让 Python 3.12 直接加载为 Python 3.10 构建的 Humble `rclpy`。若坚持核心也使用 3.12，就需要另建 ROS Python 边界进程或重建整套 ROS Python 栈，复杂度更高，不是本项目的短路径。

### 3.3 SmolVLA 模型接入：架构匹配，但需新增 wrapper/worker

RoboNeuron 已具备合适的扩展点：

- `ModelWrapper` 定义了 `load` 与 `predict_action`：`src/roboneuron_core/adapters/vla/base.py:21-50`。
- OpenVLA wrapper 已将推理代理到独立子进程：`src/roboneuron_core/adapters/vla/openvla.py:19-53`。
- 子进程通过 JSON/stdio 交换图像和动作：`src/roboneuron_core/runtime/openvla_client.py:99-157`。

因此 SmolVLA 应实现为 `SmolVLAWrapper + SmolVLASubprocessClient + smolvla_worker`，而不是把 lerobot/torch 安装进核心环境。

需要额外处理两个现有假设：

1. `vla_server.py` 对除 dummy 外的模型强制要求核心环境安装 torch；SmolVLA 代理模型应豁免此要求，否则独立环境失去意义。见 `vla_server.py:151-160,200-230,503-507`。
2. 当前 VLA extra 声明 `torch>=2.9.1`，与本机为 Pascal 显卡固定的 torch 2.6.0+cu124 不一致。SmolVLA 核心环境不应安装该 extra；模型依赖只留在模型 venv。见 `pyproject.toml:19-22` 和 `apps/policies/smolvla/setup_smolvla.py:45-57`。

### 3.4 观测契约：必须加入关节状态，不能只传单张图

现有 VLA server 只有 OpenVLA-OFT 分支会构造多图像/状态观测；其他模型只收到主相机图像：

- `third_party/RoboNeuron/src/roboneuron_core/servers/vla_server.py:127-148`
- 其 `state_topic` 当前是 7D `TaskSpaceState`，不是 Elfin5 的 6D `JointState`：`vla_server.py:249-250,274-281`。

父项目已有可用的观测构造参考：`apps/policies/smolvla/smolvla_bridge.py:99-140`。适配时应：

- 订阅 `/camera/color/image_raw`，统一转为 RGB；
- 订阅 `/joint_states`，按 `elfin_joint1` 到 `elfin_joint6` 显式排序，缺关节或含非有限值时拒绝推理；
- 按 checkpoint 的 `input_features` 构造 `observation.state` 和 `observation.images.*`；
- 对图像和状态做时间戳/新鲜度检查；
- 初始链路可沿用现有的相同图像填充多个相机输入，但这只用于兼容性验证。正式微调必须让相机数量、顺序和训练数据一致。

### 3.5 动作契约：这是最关键的不可直接复用点

RoboNeuron 当前原生动作语义是 7D EEF delta 或 7D normalized Cartesian velocity：

- `EEFDeltaCommand.msg:3-14`
- `action_semantics.py:86-155`
- `RawActionChunk.msg` 虽能承载任意 `(T,D)`，但 edge 只支持上述 Cartesian 协议。

父项目现有 SmolVLA 桥接则输出 6 个关节轨迹目标，且对预训练动作采用“相邻动作差分积分 + 单步钳制 + 关节限位”：`apps/policies/smolvla/smolvla_bridge.py:7-12,161-179`。

`smolvla_base` 来自 SO-101 数据，6 维中还可能包含夹爪语义；Elfin5 当前桥接的 6 维是机械臂关节。二者不能仅因维度相同就视为等价。直接配置为 `eef_delta` 会发生语义错误，直接配置为默认 `raw_action_chunk` 又会因不是 7D 而被拒绝或误解释。

推荐新增显式协议：

- **链路演示**：保留现有相对积分安全映射，使用独立的 demo-only 协议名称，防止误当绝对关节命令。
- **Elfin5 微调后**：采用 `elfin_joint_position`（6D，或含夹爪时另定维度），由父项目动作适配器转换为 `/elfin_arm_controller/joint_trajectory`。
- 不建议首版把 SO-101 关节动作硬转换为 7D Cartesian 动作。若坚持使用 RoboNeuron 现有 Cartesian edge，则需要重新采集/训练 7D Cartesian 数据集，这是另一条路线。

### 3.6 Chunk 与实时性：能运行不等于实时

本地 LeRobot 0.4.4 官方源码显示 SmolVLA 默认 `chunk_size=50`、`n_action_steps=50`，并同时提供：

- `predict_action_chunk`：一次返回完整 chunk；
- `select_action`：内部排队，逐次返回单步，队列空时才重新推理。

来源：`apps/policies/smolvla/.venv/lib/python3.10/site-packages/lerobot/policies/smolvla/configuration_smolvla.py:29-45` 和 `modeling_smolvla.py:310-350`；对应上游版本为 [Hugging Face LeRobot v0.4.4](https://github.com/huggingface/lerobot/tree/v0.4.4)。

RoboNeuron 适配应使用 `predict_action_chunk` 产生 `(T,6)`，不要为组装一个 chunk 连续调用 50 次 `select_action`。现有项目文档记录 GTX 1050 Ti 约 2.5–4 秒/chunk，控制频率 10 Hz：`apps/policies/smolvla/README.md:58-75`。该数字是已有项目记录，本轮未重新跑完整模型基准。

这意味着：

- 4 GB 消费级显卡可作为链路推理平台，但不能据此承诺硬实时或所有任务均可流畅运行；
- 推理与运动执行必须在不同进程，利用已有 chunk 覆盖下一次推理耗时；
- 需要记录 chunk 延迟、队列剩余步数、状态年龄和推理超时时长；
- 新 chunk 到达时替换旧队列可能造成不连续，应通过短执行前缀、提前请求或 RTC 策略单独验证；
- 超时、空 chunk、NaN/Inf、状态过期时必须停止发送新运动，而不是继续积分旧动作。

## 4. 推荐总体架构

```text
D435i /camera/color/image_raw ───────┐
                                     ├─> RoboNeuron VLA server（Python 3.10 / Humble）
Elfin5 /joint_states ────────────────┘              │
                                                    │ stdio JSON，独立进程
                                                    v
                                   SmolVLA worker（现有模型 venv）
                                   torch 2.6.0+cu124 / lerobot 0.4.4
                                                    │
                                                    v
                              RawActionChunk，显式 joint-space 协议
                                                    │
                                                    v
                       父项目 Elfin5 动作适配器（apps/policies/smolvla）
                       关节顺序 / 限位 / 限速 / 超时 / 单一发布者
                                                    │
                                                    v
                    /elfin_arm_controller/joint_trajectory（仿真或真机二选一）
```

通用 SmolVLA/Humble 支持放在自有 RoboNeuron fork，并通过子模块固定提交；Elfin5 专用配置、启动和动作适配留在父仓库 `apps/policies/smolvla/`。不要把父项目专用代码直接提交到上游 RoboNeuron。

## 5. 分阶段实施方案

### P0：固定依赖与 Humble 基线

1. 为 RoboNeuron 建自有 fork/分支，父仓库仍用 gitlink 固定确切提交，不使用 `submodule update --remote`。
2. 增加 Python 3.10 支持元数据，重新生成锁文件；运行 dummy、协议和本地单元测试。
3. 核心环境不安装 SmolVLA 模型依赖；让代理型 wrapper 不再被要求加载核心 torch。
4. 保留已验证的 `roboneuron_interfaces` Humble 构建，构建产物放 `.build/roboneuron/`。
5. 启动脚本使用 `/home/nanoctl/.local/bin/uv` 的绝对路径，或显式配置 PATH，避免非登录 shell 找不到 uv。

**验收**：Python 3.10 环境可安装；Humble 消息构建/导入通过；dummy VLA 链路测试通过。

### P1：实现 SmolVLA 独立推理运行时

1. 新建 SmolVLA wrapper、client、protocol、worker，接口对齐现有 OpenVLA 子进程模式。
2. worker 内使用 `SmolVLAPolicy.from_pretrained` 与 `make_pre_post_processors`；模型 ID 与本地路径分开处理，不能把 Hugging Face ID 错当仓库相对路径。
3. 使用 `predict_action_chunk`，返回有限 `float32 (T,D)`；校验维度、有限性和空结果。
4. 配置项至少包括模型路径/ID、`runtime_python`、设备、启动/请求超时、相机映射、状态维度和动作协议。
5. 本机先保持已知可用的 torch 2.6.0+cu124 和默认 FP32；不默认启用 BF16、FlashAttention 或新的 CUDA wheel。FP16/量化只在单独精度和稳定性验证后考虑。

**验收**：dummy 图像/6D 状态得到 `(50,6)`；重复调用无进程泄漏；错误能通过协议返回；记录显存和 p50/p95 延迟。

### P2：补齐观测与 Elfin5 动作适配

1. 将 VLA server 的模型特判改为声明式观测映射，SmolVLA 接收 RGB 图像、6D JointState 和 instruction。
2. 在父项目将现有 `smolvla_bridge.py` 的安全执行部分拆成动作适配器，避免同时存在两份模型推理和两路轨迹发布。
3. 为 demo 相对积分和微调后的绝对关节位置使用不同协议；严禁静默切换。
4. 校验六个关节名称、顺序、限位、单步/单位时间最大变化、轨迹时间、状态超时和停止行为。
5. 任一时刻只允许一个 `/elfin_arm_controller/joint_trajectory` 发布者；现有独立 bridge、MoveIt 演示和 RoboNeuron edge 不得并行抢控制权。

**验收**：错误关节名、缺状态、过期图像、NaN、超长/不足 chunk 均被拒绝；停止后无新轨迹；旧独立 bridge 不再重复加载模型。

### P3：Gazebo Harmonic 闭环与性能门禁

1. 沿用 `apps/policies/smolvla/run_gz_sim.sh`，仿真使用 `ROS_DOMAIN_ID=85`，与真机隔离。
2. 先 dummy/固定 chunk，再 SmolVLA 空场景/静态指令，最后才允许闭环运动。
3. 记录 10 Hz 执行稳定性、chunk 推理 p50/p95、最大控制空窗、显存、内存和异常恢复。
4. 验证提前推理与 chunk 替换策略，不能只以“关节角发生变化”作为完成标准。
5. 本轮只完成静态和消息级验证，以上仿真验收仍待实施。

**验收**：无控制空窗或有明确安全停止；无并行发布者；重复运行结果和日志可追溯；模型延迟超限时安全降级。

### P4：Elfin5 数据与微调

1. 采集 Elfin5 实际相机、6D 关节状态、动作和任务文本，固定关节顺序、单位、控制频率和坐标/相机约定。
2. 训练/验证数据包含对应归一化统计；部署必须使用微调 checkpoint 的预处理器和统计，不再复用 SO-101 动作统计。
3. 4 GB GTX 1050 Ti 只定位为推理验证平台；训练放到内存/显存足够的独立环境，不把训练作为本机前置目标。
4. 先仿真回放和任务成功率评估，再决定是否进入真机。

**验收**：离线动作误差、仿真任务成功率、越限/碰撞/停止指标均达标；模型、数据集、统计和配置版本绑定。

### P5：真机准入

仅在 P3/P4 通过后进行。使用独立真机配置和 ROS domain，先低速、小范围、单步/短 chunk，确认急停、限位、通信中断停止和人工接管。预训练 base 模型不得直接进入真机任务验收。

## 6. 不建议的路线

- 不建议升级 Ubuntu 24.04/Jazzy 作为第一步，迁移面过大且非必要。
- 不建议在 Python 3.12 中直接复用 Humble 的 Python 3.10 二进制包。
- 不建议把 SmolVLA 和全部 CUDA 依赖装进 RoboNeuron 核心环境。
- 不建议把 `smolvla_base` 的 6D 输出直接当 Elfin5 绝对关节位置或 7D EEF delta。
- 不建议同时保留旧 `smolvla_bridge.py` 和新 RoboNeuron 控制链路并行发轨迹。
- 不建议以“画面更新/关节运动”替代动作语义、安全和任务成功率验收。

## 7. 来源与证据

- ROS 2 官方 Humble Ubuntu 22.04 安装文档（本次在线读取成功）：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- RoboNeuron 原始上游分析基线：https://github.com/guanweifan/RoboNeuron/tree/5b792d59b8e8f870bf87b6ca0fd53a0c43fbf7eb
- RoboNeuron 自有 fork 固定提交：https://github.com/Cosine16/RoboNeuron/tree/6e93977a74cbba87be0b01c54533779851dc79ea
- RoboNeuron 本地源码：`third_party/RoboNeuron/`，提交与自有 fork 固定提交一致。
- Hugging Face LeRobot v0.4.4：https://github.com/huggingface/lerobot/tree/v0.4.4
- 本机安装的 LeRobot 0.4.4 SmolVLA 源码与缓存模型配置；在线 Hugging Face 页面本次未成功读取，因此未把网页中的额外声明作为证据。
- 父项目现有 SmolVLA 实现和运行记录：`apps/policies/smolvla/`。
