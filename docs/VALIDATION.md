# 本次交付验证记录

本记录对应包内既有数据与默认配置。完整数值以 `results/` 中 JSON/NPZ 为准，不将快速小样本结果当作完整辨识结论。

## 1. 实际验证范围

| 项目 | 结果 | 边界 |
|---|---|---|
| Linux 独立 Python 环境 | full 和 quick 全流程成功 | 不需要 ROS 或机器人连接 |
| Windows x64 依赖 | conda-forge win-64 依赖解析成功 | 未在 Windows 系统上实机运行；Conda dry-run 不执行 pip 阶段 |
| 目录可迁移性 | 含中文与空格的独立副本重新安装后完成 quick | Linux 实际执行；只复制 quick 所需数据，不重复复制全部 10 GB |
| 原始快照校验 | 1791 项大小与 SHA-256 均通过 | 已同时核对包内副本与原工作区来源 |
| 单元与动力学一致性 | 7 项通过 | 检查变换和接口，不证明参数等于真实连杆数值 |
| 新轨迹生成 | all/train/test 模式及已有 summary 复用均通过 | 关节采样界限，不包含碰撞或真机执行 |
| 可选 ROS 2 采集 | 编译成功、launch 参数解析成功 | 未连接 FR3，未验证实际硬件采集 |

环境为 Python 3.10.22、Pinocchio 3.8.0、NumPy 2.2.6、SciPy 1.15.2、pandas 2.3.3、CVXPY 1.6.7、MuJoCo 3.3.7。Windows 使用同一环境 YAML 的平台构建；不能把 Linux 的二进制环境直接复制为 Windows 环境。

## 2. 完整数据辨识

- 训练 185 条有效 CSV，143359 个状态；测试 498 条有效 CSV，473975 个状态。
- 84 项输出：70 惯性 + 14 摩擦；数值秩 57，零空间维数 27。
- 求解器 `cvxpy SDP (CLARABEL: optimal)`，25 次 IRLS 收敛，最后相对变化 `9.0663077353e-05`。
- 训练总体未加权 RMSE `1.768937 Nm`。train/test 激励范围和状态分布不同，不应直接用训练/测试误差大小判断过拟合。

| 测试指标 | 名义 / Nm | 辨识 / Nm |
|---|---:|---:|
| 完整逆动力学总体 RMSE（主指标） | 1.282252 | 1.018111 |
| CSV 重力替换总体 RMSE（辅助） | 1.275998 | 1.048328 |

逐关节完整动力学 RMSE：

| 关节 | 名义 / Nm | 辨识 / Nm |
|---|---:|---:|
| J1 | 1.492051 | 1.159805 |
| J2 | 2.118338 | 1.724169 |
| J3 | 1.603718 | 1.315102 |
| J4 | 1.429364 | 1.050780 |
| J5 | 0.285965 | 0.180817 |
| J6 | 0.273118 | 0.236244 |
| J7 | 0.155754 | 0.125709 |

本次七个关节的测试力矩 RMSE 均下降；这只支持当前测试数据分布下的力矩预测改善。重力替换指标用了机器人内部重力，不能作为重力参数独立准确性的证据。

## 3. 各连杆数值的解释

下表列出输出中的质量，方便接收者立即发现结果的局限。模型满足数值物理约束，但部分质量明显偏离名义先验。

| 连杆 | 名义质量 / kg | 辨识代表解 / kg | 名义值的倍数 |
|---|---:|---:|---:|
| link1 | 2.927465 | 2.927517 | 1.000 |
| link2 | 2.935537 | 10.434944 | 3.555 |
| link3 | 2.244901 | 8.915961 | 3.972 |
| link4 | 2.615596 | 5.245978 | 2.006 |
| link5 | 2.327121 | 2.076837 | 0.892 |
| link6 | 1.817038 | 1.327550 | 0.731 |
| link7 | 0.627143 | 0.065230 | 0.104 |

**不能把这些数值当成各连杆独立称重值或唯一真实参数。** 秩不足、参数耦合、未建模传动项和实验条件都会影响代表解。物理可行性并不等于与 CAD 外形/真实材料相符。当前没有给出统计置信区间，详见 [方法中的可辨识性说明](IDENTIFICATION_METHOD.md)。

## 4. 模型与闭环对比

完整仿真使用测试轨迹 350（历史保存的期望位置与其完全一致），时长 10 s、步长 1 ms。默认两个模型使用相同固定 armature、相同增益和摩擦参数，分别补偿自己的 bias。保存数据均有限，未触发数值异常；两张 PNG 已人工检查布局。

| 关节 | 名义跟踪 RMSE / rad | 辨识跟踪 RMSE / rad |
|---|---:|---:|
| J1 | 0.00061576 | 0.00049857 |
| J2 | 0.00103678 | 0.00080562 |
| J3 | 0.00085756 | 0.00081189 |
| J4 | 0.00089446 | 0.00070363 |
| J5 | 0.00029512 | 0.00028713 |
| J6 | 0.00052288 | 0.00050110 |
| J7 | 0.00041516 | 0.00041493 |

跟踪误差减小本身不证明模型更接近真实机器人；控制器使用各自模型补偿，实际硬件内环也不同。与实测的 q/dq/tau 分项误差见 [simulation_metrics.json](../results/simulation/simulation_metrics.json)，仿真假设和历史差别见 [模型导出与仿真](MODEL_EXPORT_AND_SIMULATION.md)。

## 5. 检查项目与复查命令

7 项测试覆盖：CSV 输出有限七轴样本；空/不足样本跳过且来源编号正确；平行轴往返；伪惯量往返；FFS 端点/形状；生成轨迹的端点导数与关节界限；URDF→Pinocchio 与 MDH Pinocchio 逆动力学一致、MuJoCo 与 Pinocchio + 固定 armature 一致。动力学检查在包含空格的临时路径执行。

```text
python tools/check_environment.py --verify-hashes
python -m unittest discover -s tests -v
python run_pipeline.py --profile quick
python run_pipeline.py --profile full
```

ROS 编译中只有现有 Franka 依赖的 deprecated coriolis 警告；未声称通过硬件执行、碰撞验证或运动安全认证。

## 6. 证据文件

- [Linux 环境检查](verification/linux_environment.json)
- [Windows 依赖解析](verification/windows_dependency_check.json)
- [独立目录执行记录](verification/portable_copy.json) 及 [日志](verification/portable_copy.log)
- [包内数据校验](verification/data_integrity.json) 与 [原来源核对](verification/original_source_check.json)
- [生成模式检查](verification/generation_modes.json)
- [7 项测试日志](verification/unit_tests.log)
- [完整执行日志](verification/full_pipeline.log)
- [ROS 编译日志](verification/ros2_build.log) 与 [launch 参数](verification/ros2_launch_arguments.log)
- [完整运行报告](../results/RUN_REPORT.md) 与 [运行清单](../results/run_manifest.json)

这些记录包含验证时的绝对路径，仅用于追溯，不是程序运行依赖。之后修改代码或配置，应重新运行受影响的检查并保留新的结果目录。
