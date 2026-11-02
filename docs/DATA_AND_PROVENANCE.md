# 数据快照、配对与历史来源

本工程从原工作区复制独立副本，未移动或修改 `ffs_data` 原文件。运行代码只访问当前工程内的数据；原工作区的路径不要求在接收者电脑存在。原始数据清单共 **1791 项、10394229778 字节**，含约 10.4 GB CSV。

## GitHub 与独立数据包

GitHub 只发布代码、轨迹 JSON、清单和复现材料，不包含 `data/raw/` 或 `data/processed/`。raw 压缩包的下载位置及解压步骤见 [README](../README.md#原始数据下载与解压)，下载链接将在数据上传完成后补充。将压缩包解压到项目 `data/` 目录，最终得到 `data/raw/{train,test,unassigned}/`。processed 由预处理命令或流水线自动生成。

下面的数量和样本数描述原始归档快照；接收的数据包是否齐全，以 `data/MANIFEST.csv` 的存在性、大小和哈希校验为准。

## 1. 包含的文件

下表的源路径相对原 `/home/guoqiang/franka_ros2_ws/`，仅用于追溯。

| 内容 | 原来源 | 当前工程内位置 | 数量 |
|---|---|---|---:|
| 训练采集 CSV | `ffs_data/train/` | `data/raw/train/` | 186 |
| 测试采集 CSV | `ffs_data/test/` | `data/raw/test/` | 500 |
| 未分类 CSV | `ffs_data/` 根目录 | `data/raw/unassigned/` | 2 |
| 训练 FFS JSON | `ffs_data/train/` | `data/trajectories/train/` | 500 |
| 当前测试 FFS JSON | `ffs_data/test/` | `data/trajectories/test/` | 500 |
| 旧测试 FFS JSON | `ffs_data/test/` | `data/trajectories/test/legacy_20260514_174449/` | 100 |
| 汇总及备份 | `ffs_data/` | `data/trajectories/summaries/` | 3 |

summary 包括 `ffs_summary_20260514_174449.json`、该文件的 `.bak` 和 `ffs_summary_single_test.json`。JSON 和 summary 是轨迹生成程序的输出；CSV 是执行轨迹时的采集结果，二者不是同一种数据。

`data/MANIFEST.csv` 逐文件记录类型、split、轨迹编号、文件时间戳、原相对路径、当前相对路径、配对 JSON、配对规则、字节数和 SHA-256。清单不包括后续生成的 processed、results 或 generated 轨迹。

## 2. 可用文件与不足样本记录

| split | 原始 CSV | 可处理 CSV | full 抽样后状态数 |
|---|---:|---:|---:|
| train | 186 | 185 | 143359 |
| test | 500 | 498 | 473975 |

以下 3 个文件各只有 **1 个样本**，无法估计加速度，原件仍在：

- train：`trajectory_098_20260515_230214.csv`
- test：`trajectory_124_20260515_230214.csv`
- test：`trajectory_258_20260515_230214.csv`

跳过原因记录在 `data/processed/train.json` 与 `test.json`。预处理不会删除或修补它们。若出现缺列、非有限信号或时间不递增等其他错误，会停止并给出文件名，不会静默忽略。

两条 unassigned CSV 没有进入默认训练或测试；旧版 100 条测试 JSON 也不会递归进入当前测试轨迹集。生成了 500 条训练 JSON 不代表全部已有对应的采集 CSV。

## 3. CSV 与 JSON 的配对规则

训练 CSV 有 60 条带 `20260514_174449`、125 条带 `20260515_230214`，另有一条 `trajectory_001.csv`。包内训练 JSON 只有 `20260514_174449` 这一组。

清单先按“轨迹编号 + 相同时间戳”精确配对；训练 CSV 找不到相同时间戳时，按同编号唯一训练 JSON 标为 `trajectory_index_reuse`。这是对原流程中轨迹复用的配对记录，**文件名规则本身不证明每次物理执行都用了相同系数**。需要严格复核某次执行时，可将 CSV 的 `q_d/dq_d` 与候选 JSON 的 FFS 曲线对照。七连杆辨识使用实测 `q/dq/tau_J`，不以这一候选配对作为动力学回归输入。

测试 CSV 均带 `20260515_230214`，与当前测试 JSON 同名精确匹配。旧测试 JSON 单独归档，避免仅按编号误配。

时间戳是原文件的身份标记，不应单凭它断言机器人的日历时间、采集时区或全部安装状态。多个 CSV 即便编号相同，也作为不同记录保留，不按编号覆盖或合并。

## 4. 为什么属于本体流程，以及证据的边界

相关采集器是原 `src/franka_ffs_controllers/src/ffs_data_collection_controller.cpp`，其内部状态记录包括：

| 信号 | 含义 | 当前用途 |
|---|---|---|
| `ros_time_ns, period_ns` | ROS 时刻与更新周期 | 建立时间、检查采样 |
| `q, dq` | 七关节实测状态 | 动力学回归 |
| `q_d, dq_d` | 机器人期望状态 | 轨迹配对及实测误差图 |
| `tau_J` | 测量关节力矩 | 拟合和测试目标 |
| `tau_J_d` | 期望/命令侧力矩 | 原样保留，当前不拟合 |
| `tau_ext_hat_filtered` | Franka 内部外力矩估计 | 原样保留，不是外置传感器读数 |
| `O_T_EE` | 末端位姿矩阵 | 原样保留 |
| `mass, coriolis, gravity` | Franka 模型输出 | gravity 用于辅助指标；其他保留供核验 |

这个采集器没有外置六维力传感器或 IMU 输入；七连杆回归代码也没有外部零件参数、IMU 回归或外力传感器标定步骤。工作区中的 `imu_force_collector_node.cpp`、`force_sensor_publisher_node.cpp` 等另一套代码没有移入主流程。

**没有外置传感器列，不等于证明机械臂末端物理上没有装东西。** 本次整理无法单靠 CSV 还原当时夹具/负载、机器人负载配置、温度及全部实验条件。包内数据按原本体 FFS 流程归档；若研究要求精确测得裸机械臂本体惯性，应补充这些条件的实验记录。当前参数必须按报告中的可辨识性与建模限制理解。

## 5. 两张历史图的真正来源

用户先前指定的文件位于原：

```text
ffs_data/src/franka_identification_controllers/mujoco_validation/results/ffs_simulation/
    ffs_simulation_comparison.png
    error_zoom.png
```

对应生成脚本为原：

```text
src/franka_identification_controllers/mujoco_validation/scripts/5_ffs_simulation.py
```

这些图、原 XML、两个时序 NPZ、脚本、旧辨识/验证脚本和旧参数快照已放入 `reference/legacy_ffs_simulation/`。`PROVENANCE.json` 列出来源和逐文件哈希。

通过比较历史 NPZ 中保存的 `q_d` 与现有 JSON 的期望位置，确认历史图对应测试轨迹 **350**，最大绝对差为 **0 rad**。旧脚本文本中的默认编号 391 与已保存结果不一致，因此新默认选择 350，以保存数据的证据为准。

新程序修正了惯量处理、摩擦力方向与模型运动学等问题，并采用严格的惯性约束，因此不会承诺像素复现旧图。具体差别见 [模型导出与仿真](MODEL_EXPORT_AND_SIMULATION.md)。旧参数格式也不会直接送入新导出器。

## 6. 清单与复制校验

下载并解压 raw 数据后执行：

```text
python tools/check_environment.py --verify-data
python tools/check_environment.py --verify-hashes
```

前者核对所有清单文件的存在性和大小；后者读取全部约 10.4 GB 内容核对 SHA-256。环境检查不依赖原工作区。原始来源路径只保存在清单、历史归档和报告中。

`python tools/build_data_manifest.py --hashes` 可以重新生成清单，但交接后应先校验原清单；不要先重建再用新清单证明复制正确。新采集数据先放独立 run 目录，核对后再决定如何纳入新一轮实验。
