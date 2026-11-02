# Windows / VS Code 环境与交接

## 1. 环境要求

- Windows 10 或 11，x86-64；本环境文件不面向 Windows ARM。
- VS Code，以及 Microsoft 的 Python / Python Debugger 扩展。项目的 `.vscode/extensions.json` 已提供推荐项。
- Miniforge x86-64，或可访问 conda-forge 的 Miniconda。
- 建议至少 16 GB 内存；完整回归会占用数值矩阵内存。内存较小可先使用 quick 模式，或显式增大 stride 并记录该变化。
- 至少 25 GB 可用磁盘。单独下载的原始 CSV 约 10.4 GB（9.7 GiB），另需环境、处理数据和结果空间。GitHub 仓库不包含 raw 或 processed 数据。
- 安装时需要访问 conda-forge、Python 包源；安装完成后，已有数据的整个离线流程不需要网络。

不需要 ROS、WSL、机器人连接、GPU 或图形界面渲染。MuJoCo 只做动力学计算，Matplotlib 使用 Agg 输出 PNG。

## 2. 安装与选择解释器

下载并安装 [Miniforge](https://github.com/conda-forge/miniforge/releases) 的 Windows x86-64 版本。复制项目到一个普通目录，比如 `D:\work\franka_7link_dynamics_id`。目录名可以包含空格；终端进入这类目录时要加双引号。

先按 [README 的原始数据下载与解压说明](../README.md#原始数据下载与解压) 将 raw 压缩包解压到项目 `data/` 目录，确认 CSV 位于 `data/raw/train/` 和 `data/raw/test/`。processed 文件会在首次运行流水线时生成。

推荐在 **Miniforge Prompt** 里执行下面的 CMD 命令：

```bat
cd /d D:\work\franka_7link_dynamics_id
conda env create -f environment-windows.yml
conda activate franka-7link-id
python -m pip install -e .
python tools/check_environment.py --verify-data
python -m unittest discover -s tests -v
code .
```

`cd /d` 是 CMD 的切换磁盘写法。如果使用 PowerShell，进入目录的命令改为：

```powershell
Set-Location "D:\work\franka_7link_dynamics_id"
```

从已激活环境的 Miniforge Prompt 执行 `code .`，可以让 VS Code 继承正确的环境和 DLL 搜索路径。若 `code` 不在 PATH，可在 VS Code 的 File > Open Folder 打开此目录，然后选择解释器。

在 VS Code 按 `Ctrl+Shift+P`，选择 **Python: Select Interpreter**，选 `franka-7link-id`。之后新建集成终端，核对：

```text
python -c "import sys; print(sys.executable)"
python -c "import pinocchio, mujoco, cvxpy; print(pinocchio.__version__, mujoco.__version__, cvxpy.__version__)"
```

路径应指向 `franka-7link-id` 环境内的 `python.exe`。也可以在 Run and Debug 里选择“FR3: 快速跑通已有数据流程”或“FR3: 完整数据辨识与仿真”。两项配置都使用当前选定解释器。

## 3. 为什么使用这个环境文件

主要版本是 Python 3.10、Pinocchio 3.8、NumPy 2.2、SciPy 1.15、CVXPY 1.6、MuJoCo 3.3.7。Pinocchio 从 conda-forge 安装，不要执行 `pip install pinocchio`；那个名称不能代替这里需要的机器人动力学库。

Windows 的 Pinocchio/eigenpy 构建要求 NumPy 2.x，所以不要把环境中的 NumPy 降到 1.x。CVXPY、CLARABEL/SCS 已随环境安装，默认走半正定约束求解，无需再安装单独的“sdp 可选包”。

环境文件设置 `PYTHONNOUSERSITE=1`，防止同一 Python 版本的用户级旧库混入。本工程安装为 editable 项目；若之后移动了项目目录，需要在新目录再次运行 `python -m pip install -e .`。

Windows conda-forge 依赖解析已检查，记录在 [windows_dependency_check.json](verification/windows_dependency_check.json)。MuJoCo 3.3.7 有 CPython 3.10 的 Windows x64 wheel。当前工作机器是 Linux，**没有声称在 Windows 系统上实机执行过程序**；Linux 已用相同主要依赖版本完成实际运行，见 [验证记录](VALIDATION.md)。

## 4. 开始运行

```text
python run_pipeline.py --profile quick
python run_pipeline.py --profile full
```

快速结果在 `results/quick`，完整结果在 `results`。在 VS Code 预览 `RUN_REPORT.md`，或直接打开生成的 PNG。

若准备数据阶段已经成功，后续因环境或求解设置中断，可复用同一模式的预处理文件：

```text
python run_pipeline.py --profile full --skip-prepare
```

改变 stride、滤波窗口或原始数据之后，要重新运行预处理，不能使用旧的 NPZ。默认完整运行可覆盖本工程的处理结果；原始 CSV 和原工作区数据不会被改写。希望保留已有结果时，可指定一个新的输出目录：

```text
python run_pipeline.py --profile full --skip-prepare --output-dir results/experiment_02
```

该选项分开保存辨识、评估和仿真输出；预处理文件仍位于 `data/processed`。

## 5. 常见问题

| 现象 | 处理 |
|---|---|
| `conda` 未识别 | 用 Miniforge Prompt，不要先在未配置 Conda 的 PowerShell 中安装。之后也可在 Prompt 中执行 `conda run -n franka-7link-id python run_pipeline.py --profile quick`。 |
| `No module named franka_7link_dynamics_id` | 在项目根目录、正确环境中运行 `python -m pip install -e .`。移动目录后也要重做这一步。 |
| Pinocchio 导入出现 DLL load failed | 从已激活的 Miniforge Prompt 启动 VS Code，核对 Python 路径；不要把其他 ROS、旧 Python 或用户级库加入这个环境。重新运行环境检查。 |
| NumPy ABI / `_ARRAY_API` 错误 | 环境混用了 NumPy 1.x 编译的库。按本环境文件新建环境，并确认 `PYTHONNOUSERSITE=1`。 |
| 下载失败、连接重置 | 先恢复网络，再重试环境创建。若环境已经创建但 pip 阶段失败，激活它后重试 `python -m pip install -e .`。 |
| 缺 CSV/JSON 或数据大小不符 | 先按 README 单独下载并解压 raw 数据，确认目录结构正确并补齐缺失文件。`--verify-data` 按清单核对大小；`--verify-hashes` 会读取全部文件核对 SHA-256，耗时更长。 |
| 磁盘/内存不足 | 先跑 quick；完整模式可设置 `--stride 20` 或更大，但这是不同的采样设置，结果不能与默认 stride=10 直接当作同一次实验。 |
| 中文图表字体缺失 | 本工程图表标签使用英文，无需下载中文字体。Markdown 文档和路径使用 UTF-8。 |
| 报告 `converged=false` | quick 限制为 5 次迭代，允许此状态；完整模式会显示是否在最多 50 次内满足阈值。不能把达到迭代上限解释成已收敛。 |

## 6. 给下一位使用者的拷贝清单

通过 GitHub 获取 `README.md`、环境文件、全部代码/配置/文档、`models`、`data/trajectories`、`data/MANIFEST.csv`、`results` 和 `reference`。`data/raw` 从 README 所列的公开数据网站单独下载并解压；`data/processed` 不随仓库发布，可重新运行预处理生成。离线交接时也应将代码与 raw 数据包分别提供。

不需要复制旧工作区的 `build`、`install`、`log`、其他 ROS 包或系统环境。`__pycache__` 和 `*.egg-info` 是本地缓存/安装信息，可以省略。不要把 Linux 的 Conda 环境目录直接当作 Windows 环境复制；接收者应按 YAML 创建 Windows 环境。
