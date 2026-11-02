# Franka FR3 七连杆本体动力学辨识

可直接复制的 **Python 工程**。Windows 10/11 x64 + VS Code 可以运行；主流程不需要 ROS、WSL、Franka 驱动或 GPU。

对象是 **FR3 的 `fr3_link1`～`fr3_link7`**，以及七个关节的简单摩擦。

仓库包含代码、配置、FFS 轨迹、原始文件清单及已有结果。**GitHub 仓库不包含 `data/raw/` 原始 CSV，也不包含 `data/processed/` 预处理文件。** raw 数据从独立的公开数据网站下载；processed 文件会在运行预处理时自动生成。

## 原始数据下载与解压

**raw 数据下载链接：待补充（数据正在上传）。**

1. 克隆或下载本仓库，并进入 `franka_7link_dynamics_id` 项目根目录。
2. 从上述数据网站下载 raw 压缩包，将其解压到项目的 **`data/` 目录**，使压缩包中的 `raw/` 文件夹位于 `data/raw/`。
3. 检查解压后的最终结构如下；不要多出 `data/raw/raw/` 或 `data/压缩包名称/raw/` 一层。若压缩包直接包含 `train/`、`test/` 等目录，请先创建 `data/raw/`，再将这些目录放入其中。

```text
franka_7link_dynamics_id/
└── data/
    ├── MANIFEST.csv          仓库已提供：原始文件清单与 SHA-256
    ├── trajectories/        仓库已提供：FFS 轨迹 JSON
    └── raw/                 单独下载压缩包并解压得到
        ├── train/           训练 CSV
        ├── test/            测试 CSV
        └── unassigned/      原始快照中的未分类 CSV
```

安装下面的运行环境后，在项目根目录执行 `python tools/check_environment.py --verify-data`，按 `data/MANIFEST.csv` 核对原始数据文件的存在性和大小。需要进一步校验内容时运行 `python tools/check_environment.py --verify-hashes`。数据未下载完整时，相关检查会报告缺失文件；仅下载 GitHub 仓库还不能执行完整辨识流程。

无需下载或手动创建 `data/processed/`。首次运行 `python run_pipeline.py --profile quick` 或 `python run_pipeline.py --profile full` 会自动生成对应的 NPZ 与预处理报告；首次运行不要添加 `--skip-prepare`。`.gitignore` 已排除 raw、processed 和数据压缩包，避免将它们提交到 GitHub。

## 在 Windows 开始

先安装 [Miniforge x86-64](https://github.com/conda-forge/miniforge/releases) 和 [VS Code](https://code.visualstudio.com/)。建议 16 GB 以上内存、25 GB 以上可用磁盘空间。第一次安装依赖需要网络。

先按上文下载并解压 raw 数据，再打开 **Miniforge Prompt**，进入项目根目录，例如：

```bat
cd /d D:\work\franka_7link_dynamics_id
conda env create -f environment-windows.yml
conda activate franka-7link-id
python -m pip install -e .
python tools/check_environment.py --verify-data
python -m unittest discover -s tests -v
code .
```

在 VS Code 安装 Microsoft Python 扩展，运行 **Python: Select Interpreter**，选择 `franka-7link-id`。项目已带 `.vscode` 调试配置；也可以在终端运行：

```text
python run_pipeline.py --profile quick
python run_pipeline.py --profile full
```

`quick` 使用前五个训练/测试文件和较稀的采样，只用于核对环境和流程。`full` 使用全部可处理的已有训练/测试数据，完成预处理、PC-IRLS 辨识、测试评估、URDF/XML 导出和轨迹 350 的 MuJoCo 仿真。**已有轨迹直接复用，不需要重新生成或重新采集。**

完成后打开 [完整运行报告](results/RUN_REPORT.md) 或 [快速运行报告](results/quick/RUN_REPORT.md)。详细安装、PowerShell 用法、环境排错见 [Windows 与 VS Code 指南](docs/WINDOWS_SETUP.md)。

## 先看哪些文件

| 目的 | 文档 |
|---|---|
| 了解顺序、逐步运行、查看输出 | [完整流程](docs/PROCESS.md) |
| 理解数学模型、求解方法、参数可辨识性 | [辨识方法](docs/IDENTIFICATION_METHOD.md) |
| 查原始数据、轨迹配对、跳过的文件 | [数据与来源](docs/DATA_AND_PROVENANCE.md) |
| 理解 URDF 替换、XML、控制律和历史图的差别 | [模型导出与仿真](docs/MODEL_EXPORT_AND_SIMULATION.md) |
| 查每个模块、原项目脚本的对应关系 | [项目结构与迁移说明](docs/PROJECT_MAP.md) |
| 看实际验证结果与验证范围 | [验证记录](docs/VALIDATION.md) |
| 以后在 Linux 连接 FR3 新采数据 | [可选采集扩展](ros2_collector/README.md) |

## 项目结构

```text
franka_7link_dynamics_id/
├── README.md                     从这里开始
├── environment-windows.yml       Windows/Linux 共用的 Conda 环境
├── pyproject.toml                Python 项目安装信息
├── run_pipeline.py               一条命令运行已有数据的整个离线流程
├── .vscode/                      VS Code 环境和调试配置
├── config/                       FR3 模型、FFS、辨识配置
├── data/
│   ├── MANIFEST.csv              原始文件清单、配对、来源、SHA-256
│   ├── raw/{train,test,unassigned}/  单独下载的采集 CSV；不提交 Git
│   ├── trajectories/             已有 train/test JSON、summary、旧版本
│   └── processed/                运行时自动生成；不提交 Git
├── franka_7link_dynamics_id/      可阅读的 Python 算法模块
├── scripts/                      生成、预处理、辨识、评估、导出、仿真的独立入口
├── models/                       无外部 mesh 依赖的名义 URDF 和生成模型
├── results/                      完整结果；quick/ 为快速检查结果
├── reference/                    历史图、旧脚本、旧参数及 FR3 描述来源
├── docs/                         原理、运行和交接文档
├── tools/                        环境检查和数据清单工具
├── tests/                        数据与跨动力学引擎一致性检查
└── ros2_collector/               仅用于 Linux 真机采集的可选扩展
```

七连杆对应 70 个惯性参数，摩擦对应 14 个参数，输出共 84 项。**七连杆全参数并非都能被这批关节数据独立确定**：报告给出秩和零空间维数，最终结果是带名义先验和物理约束的代表解。某些质量明显偏离先验时，报告会注明，不能把它们当作逐连杆称重结果。

真实机器人重新采集需要 Linux、ROS 2 和 Franka 硬件环境；Windows 接收者复现本工程时不需要构建 `ros2_collector`。
