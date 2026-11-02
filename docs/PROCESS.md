# 从已有数据到仿真对比的完整流程

先按 [Windows 安装指南](WINDOWS_SETUP.md) 创建环境，并在项目根目录执行 `python -m pip install -e .`。先按 [README](../README.md#原始数据下载与解压) 单独下载 raw 压缩包，解压到项目 `data/` 目录，使 CSV 位于 `data/raw/`。GitHub 不包含 raw 或 processed；processed 在首次运行时生成。以下离线命令在 Windows 和 Linux 共用，均不要求 ROS。路径相对当前项目根目录，可在 VS Code 终端执行。

```text
已有 FFS 系数 JSON → 真机速度控制执行和内部状态 CSV（已有副本）
                                  ↓
              按轨迹微分、平滑、抽样 → train/test NPZ
                                  ↓
                训练集 PC-IRLS + 物理约束 → 84 项参数
                                  ↓
                独立测试集力矩预测 → JSON、CSV、PNG
                                  ↓
               替换七连杆 URDF inertial → 读回校验 → MJCF
                                  ↓
          同一 FFS、同一控制律 → 两模型仿真、实测叠图、误差图
```

## 1. 先运行完整示例

```text
python tools/check_environment.py --verify-data
python run_pipeline.py --profile quick
python run_pipeline.py --profile full
```

| 模式 | 训练/测试 CSV | 抽样 stride | IRLS 上限 | 结果位置 |
|---|---|---:|---:|---|
| quick | 按文件名排序的前 5 个文件，各取一组 | 50 | 5 | `results/quick/` |
| full | 全部可用文件：185 个训练、498 个测试 | 10 | 50 | `results/` |

quick 只核对环境和整个工作链，允许尚未收敛；辨识结论应看 full。full 的上限与阈值在 `config/identification.yaml`。达到迭代上限而 `converged=false` 时，不能视为已收敛。

默认使用已有测试轨迹 **350** 与同名实测 CSV。历史仿真 NPZ 的期望位置与这条轨迹完全相符，证据见 [来源说明](DATA_AND_PROVENANCE.md)。仓库提供已有轨迹与现成结果，CSV 需从公开数据网站单独下载并解压；默认流程不重新生成轨迹、不接机器人。

预处理已完成时可复用：

```text
python run_pipeline.py --profile full --skip-prepare
python run_pipeline.py --profile full --skip-prepare --output-dir results/experiment_02
```

`--output-dir` 改变辨识、评估、导出和仿真的结果位置；预处理仍分别放在 `data/processed/` 或 `data/processed/quick/`。改变原始数据、stride 或滤波设置后要重新预处理。`--threads 4` 限定数值库线程数；低内存电脑可用 `--stride 20`，但这构成另一组采样实验。

默认结果目录会被下一次同名运行更新。要保留多次实验，给每次运行不同的 `--output-dir`。

## 2. 按步骤运行

### 2.1 轨迹生成：本次直接使用已有轨迹

既有轨迹位于 `data/trajectories/train/` 和 `data/trajectories/test/`，汇总文件位于 `data/trajectories/summaries/`。JSON 定义有限傅里叶系数，不是采集状态。

以后需要新轨迹时才执行：

```text
python scripts/generate_ffs_trajectories.py --num-train 10 --num-test 5 --seed 42
```

输出在 `data/trajectories/generated/<run_id>/`，包含 `train/`、`test/` 和 `ffs_summary_<timestamp>.json`。使用新的时间戳目录，避免覆盖已有快照。配置是 `config/ffs_generation.yaml`：5 个谐波，基频 0.1 Hz，时长 10 s，检查步长约 1 ms。train/test 初始随机系数范围分别为 `[-0.5,0.5]`、`[-0.3,0.3]`；最后一项系数用于满足端点导数约束，实际系数以输出 JSON 为准。

如只重新生成测试集并保留一份 summary 中的训练系数：

```text
python scripts/generate_ffs_trajectories.py --mode test --existing-summary data/trajectories/summaries/ffs_summary_20260514_174449.json --num-test 5 --seed 43
```

新目录也会写出保留的训练 JSON。保留 split 的系数不改变，但新目录、时间戳表示一次新的生成记录。生成器检查采样点的关节位置、速度、加速度界限；没有自碰撞、环境碰撞、力矩或连续时间极值验证。使用随机拒绝采样与必要的缩放，不执行最优实验设计。

### 2.2 真机采集：本次直接使用已有 CSV

已有数据在 `data/raw/train/`、`data/raw/test/`，逐文件来源与配对见 `data/MANIFEST.csv`。186 个训练 CSV、500 个测试 CSV 中，有 3 个文件只有一个样本，原件保留，预处理记录跳过原因。

重新采集使用 [可选 Linux ROS 2 扩展](../ros2_collector/README.md)。FFS 控制器发送七关节速度，记录器同步保存机器人内部状态、质量矩阵、科氏项和重力。不使用外置力传感器或 IMU。末端物理安装状态仍需操作人员核实，不能仅凭 CSV 列名认定空载。

新采集输出应放在 `data/acquisitions/<run_id>/{train,test}/` 等新目录，不能覆盖现有快照。不会自动进入默认 full 训练集；先核实采集条件与 JSON 配对，再用下一步的显式输入选项处理。

### 2.3 预处理

```text
python scripts/prepare_dataset.py --stride 10 --window 21 --polyorder 3
```

对每条 CSV 单独处理：整数纳秒时间戳先作差，再转秒；以实际时间对测得 `dq` 求 `gradient`，再对得到的加速度作 Savitzky–Golay 平滑；最后统一抽样。不会跨文件求导。不是直接使用理想 FFS 加速度，也不是先拼接再平滑。

输出 `data/processed/train.npz`、`test.npz`，及同名 JSON 报告。NPZ 中 `q/qd/qdd/tau/gravity` 为 `N×7`，`time` 为每条轨迹的局部时间，`source_id/source_files` 保留来源。JSON 记录文件数、跳过原因、采样间隔、时长和滤波设置。其他格式错误、非有限值或时间不递增会明确失败。

使用新数据的例子：

```text
python scripts/prepare_dataset.py --train-dir data/acquisitions/my_run/train --test-dir data/acquisitions/my_run/test --output-dir data/processed/my_run --stride 10
```

### 2.4 训练集辨识

```text
python scripts/identify_parameters.py
```

默认输入 `data/processed/train.npz`；输出到 `results/identification/`：

- `identified_params.npz`：名义/辨识向量，参数名、排列、惯性参考点、求解器等元信息。
- `parameter_table.csv`：逐参数名义值、辨识值及差值。
- `fit_report.json`：样本数、秩、零空间、奇异值、逐次 IRLS 历史、收敛、训练误差、各连杆物理惯性和解释备注。

默认严格求解半正定约束子问题，CLARABEL 不成功时尝试 SCS；均失败则停止。`--solver projection` 是需要显式选择的近似算法，不是默认回退。公式、正则项及结果解释见 [辨识方法](IDENTIFICATION_METHOD.md)。

### 2.5 未参与拟合的测试集对比

```text
python scripts/evaluate_parameters.py
```

输出到 `results/evaluation/`：`test_metrics.json`、`test_metrics.csv`、`test_rmse_comparison.png`，以及第一条测试 CSV 的 `example_torque_prediction.npz/.png`。

报告同时比较名义与辨识参数的两种预测：`full_dynamics` 是完整逆动力学的主指标；`gravity_replaced` 用 CSV 内部重力替换模型重力，作为与旧流程兼容的辅助指标。两者均报告各关节和总体 RMSE、MAE、R²。不可把重力替换结果单独用作七连杆质量/重心辨识成功的证据。

### 2.6 替换 URDF 并生成 XML

```text
python scripts/export_model.py
```

读取 `models/fr3_nominal.urdf` 的副本，输出：

```text
models/generated/fr3_identified.urdf
models/generated/fr3_ffs_nominal.xml
models/generated/fr3_ffs_identified.xml
```

只替换 `fr3_link1`～`fr3_link7` 的 `<inertial>`，将关于 link 原点的辨识惯量转换为质心惯量。随后读回 URDF 校验，再从读回数值生成 MJCF。原始模板不会被覆盖。详细转换和额外仿真惯量项见 [模型导出与仿真](MODEL_EXPORT_AND_SIMULATION.md)。

### 2.7 同轨迹闭环仿真

```text
python scripts/run_ffs_simulation.py --trajectory data/trajectories/test/trajectory_350_20260515_230214.json --csv data/raw/test/trajectory_350_20260515_230214.csv
```

输出到 `results/simulation/`：

| 文件 | 内容 |
|---|---|
| `ffs_simulation_comparison.png` | 七关节位置、速度、力矩和位置误差；实测数据以灰色叠加 |
| `error_zoom.png` | 两模型位置误差与各关节 RMSE |
| `simulation_nominal.npz` / `simulation_identified.npz` | 同时刻记录的 `time,q,dq,q_d,dq_d,tau` |
| `simulation_metrics.json` | 增益、步长、固定 armature、跟踪 RMSE、与实测重叠区间的误差 |
| `models/` | 本次仿真实际使用的两个 XML 和辨识 URDF |

该脚本重新执行“URDF 导出→读回→MJCF”，因此可以只运行这一条命令重建仿真。换轨迹时务必同时换配套 CSV；不传 `--csv` 时只尝试同名测试 CSV，找不到则仅画仿真。

## 3. 查看报告与解释结果

一键流程额外生成 `results/RUN_REPORT.md` 与 `run_manifest.json`。后者记录环境、输入配置/数据清单哈希、运行时长和轨迹路径。历史结果内的绝对路径只是当时的来源记录，不影响复制后重新运行。

先看测试集完整动力学误差，再看逐关节误差、物理约束和秩，最后看闭环图。仿真中两个模型分别补偿自身的重力与科氏项；误差减小只能说明该设置下控制响应不同，不能独立证明模型更接近真实机器人。当前数据的秩为 57/84，质量显著偏离先验的连杆已在报告注明，解释见 [验证记录](VALIDATION.md)。
