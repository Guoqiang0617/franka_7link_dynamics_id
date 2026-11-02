# 项目结构与原代码对应关系

## 1. 设计原则与阅读顺序

本工程放在工作区根目录，与 `src/` 并列。主目录没有 `package.xml`，不是 ROS 包；Python 源码、数据快照、生成结果、历史参考和可选硬件扩展分开放置。所有新离线入口从当前工程定位配置和数据，接收者不需要原 `/home/guoqiang/franka_ros2_ws`。

推荐阅读顺序：根 `README.md` → `docs/WINDOWS_SETUP.md` → `docs/PROCESS.md` → `docs/IDENTIFICATION_METHOD.md` → `results/RUN_REPORT.md`。想修改某一步时再看下表。

## 2. 入口、算法与职责

| 可执行入口 | 核心模块 | 作用 |
|---|---|---|
| `run_pipeline.py` | 标准库编排 | quick/full 顺序执行全部离线步骤，保存运行报告 |
| `scripts/generate_ffs_trajectories.py` | `trajectory_generation.py` | 纯 NumPy 生成有界 FFS，分 run 存 JSON 和 summary |
| `scripts/prepare_dataset.py` | `dataset.py` | 逐 CSV 建时间、微分、平滑、抽样，保存 NPZ 与质量记录 |
| `scripts/identify_parameters.py` | `estimation.py` | Pinocchio 回归、秩与先验、Tukey IRLS、SDP、参数表 |
| `scripts/evaluate_parameters.py` | `evaluation.py` | 未参与拟合的测试集，完整动力学/重力替换两种指标 |
| `scripts/export_model.py` | `model_export.py` | 替换七连杆 URDF inertial，读回验证，生成 MJCF |
| `scripts/run_ffs_simulation.py` | `simulation.py` | 同轨迹闭环对比，保存 XML、时序、指标和两张图 |
| `tools/check_environment.py` | 独立检查工具 | 环境、求解器、必要文件、大小/哈希检查 |
| `tools/build_data_manifest.py` | 独立清单工具 | 重建包内原始快照清单，保留配对规则和来源 |

辅助模块位于 `franka_7link_dynamics_id/`：

- `inertial.py`：CAD 向量、参数名、平行轴/伪惯量变换和物理检查。
- `parameter_io.py`：下游入口的参数格式和模型边界检查，防止误用旧排列或外部零件参数。
- `trajectory.py`：读取单条 JSON，统一计算 FFS 的位置、速度和加速度。
- `paths.py`：当前工程与数据路径；不通过 ROS ament 定位文件。

## 3. 配置、数据、模型和结果

| 位置 | 内容与维护方式 |
|---|---|
| `config/fr3_model.yaml` | FR3 MDH、七连杆 CAD 先验、限位、重力、摩擦先验、固定仿真 armature |
| `config/ffs_generation.yaml` | 新轨迹的谐波、频率、时长、初姿态、随机范围及检查设置 |
| `config/identification.yaml` | PC-IRLS 上限、阈值、Tukey、正则、LMI 和求解方式 |
| `data/raw/` | 已采集原始副本；不在这里写新输出 |
| `data/trajectories/` | 已有 JSON、summary、旧版；新生成放 `generated/<run_id>/` |
| `data/MANIFEST.csv` | 原始快照的 1791 项清单及 SHA-256 |
| `data/processed/` | 可重建 NPZ、预处理报告；quick 单独放子目录 |
| `models/fr3_nominal.urdf` | 无外部 mesh 依赖的名义动力学模板 |
| `models/generated/` | 默认 full 导出的新 URDF 和两个 MJCF |
| `results/identification,evaluation,simulation/` | 默认 full 的参数、测试评估、闭环结果 |
| `results/quick/` | 小数据流程检查，不能替代完整辨识结论 |
| `reference/` | 原始图/旧实现/旧参数/模型依据，仅供追溯 |
| `docs/verification/` | 此次交付的环境和可运行性证据 |
| `tests/` | CSV、惯性变换、FFS 和跨引擎逆动力学一致性检查 |
| `.vscode/` | Python 扩展推荐、解释器使用提示和两个运行配置 |

processed、generated 和 results 可从原始数据与代码重建；建议交接时带上现成结果，让接收者安装环境前也能看图和报告。没有把旧工作区的 build/install/log、Linux 环境或其他项目复制进来。

## 4. 从哪些原脚本整理而来

以下原路径相对 `franka_ros2_ws/`，只作迁移记录。

| 原实现 | 当前对应 |
|---|---|
| `src/franka_ffs_controllers/scripts/ffs_trajectory_generator.py` | 普通 Python `trajectory_generation.py`，移除 ROS 参数服务依赖 |
| `src/franka_ffs_controllers/src/ffs_trajectory_controller.cpp` | 可选 `ros2_collector/src/ffs_trajectory_controller.cpp` |
| `src/franka_ffs_controllers/src/ffs_data_collection_controller.cpp` | 可选 `ros2_collector/src/ffs_data_collection_controller.cpp` |
| `src/franka_ffs_controllers/scripts/ffs_orchestrator.py` | 可选 `ros2_collector/scripts/ffs_orchestrator.py` |
| `src/franka_identification_controllers/mujoco_validation/pc_irls_improved.py` | `dataset.py`、`estimation.py`、`inertial.py`，统一排列及严格求解路径 |
| `src/franka_identification_controllers/mujoco_validation/validate_identified_params.py` | `evaluation.py`，同时给出完整动力学主指标和重力替换辅助指标 |
| `src/franka_identification_controllers/mujoco_validation/scripts/5_ffs_simulation.py` | `model_export.py`、`simulation.py`，修正转换与摩擦并保留历史参考 |
| `src/franka_description/robots/fr3/*.yaml` | 当前模型配置与 `reference/fr3_description/` 来源依据 |
| 原 `ffs_data/train/`、`test/` | 当前 `data/raw/` 与 `data/trajectories/`，按文件类型分开 |

迁移时没有直接照搬所有旧脚本：主流程增加参数格式边界检查、原始哈希、失败报告、URDF 往返与跨引擎验证，并移除了旧机器的运行路径依赖。历史脚本中已有的另一种参数约定或近似回退不能与新输出混用。

## 5. 特意排除的另一套任务

原 `src/franka_identification_controllers/` 同时含有 `imu_force_collector_node.cpp`、`force_sensor_publisher_node.cpp`、其他传感器/采集节点和实验配置。目录名字不能用于判断所有代码都是七连杆本体辨识。

本工程只整理上表所列的七连杆 FFS 回归与仿真链，没有引入“机械臂末端加力传感器和 IMU，再装待测零件”的回归、外部传感器驱动或其数据。`tau_ext_hat_filtered` 是机器人内部字段，也没有当作外置力传感器使用。

## 6. 扩展边界

Windows 接收者修改离线算法只需要主工程；需要真实机器人时才读 `ros2_collector/README.md`。该子目录有独立 `package.xml`，包名 `franka_7link_dynamics_id_ros2`，与原控制器插件分名，避免构建时相互覆盖。只对 Linux ROS 2 采集作可选扩展，编译验证不等于真机验证。

若更换机器人型号、加末端负载、换参数排列或新增电机惯量待估项，需要同步更新模型、数据边界、导出与一致性检查。不能把另一个项目的 NPZ 改名放进来继续运行。
