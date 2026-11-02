# 可选 Linux ROS 2 真机采集扩展

只有重新连接 FR3 采数据才需要本目录。Windows 接收者运行主工程时无需构建或安装任何 ROS 包。

本扩展假定 Ubuntu / ROS 2 Humble、已配置的 Franka ROS 2 驱动与 FR3 硬件接口、可用的 `controller_manager` 和关节状态发布器。七个硬件关节名为 `fr3_joint1`～`fr3_joint7`，模型/状态接口为 `fr3/robot_model`、`fr3/robot_state`。主工程和本扩展都针对 FR3，不是任意 Franka 型号的通用采集器。

**此次只验证编译、launch 参数和静态逻辑，没有在真实机器人执行。** 启动下面的采集 launch 会激活回起点控制器并产生真实运动；应由熟悉现场机器人、末端安装和运动空间的人员执行。FFS 生成器的关节限位检查不包含碰撞检查。

## 1. 构建位置与环境

主工程在工作区根目录，与 `src/` 并列，不需要移动回 `src/`。在系统 ROS 环境中构建本子包；不要使用离线 Conda 环境构建 ROS：

```bash
cd /home/guoqiang/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --base-paths franka_7link_dynamics_id/ros2_collector --packages-select franka_7link_dynamics_id_ros2
source install/setup.bash
ros2 launch franka_7link_dynamics_id_ros2 collect_ffs.launch.py --show-args
```

接收者把路径换为自己的 Linux ROS 工作区。`--base-paths` 显式定位子包；ROS 包名为 `franka_7link_dynamics_id_ros2`，与原插件分名。

编译依赖包括 `controller_interface`、`franka_msgs`、`franka_semantic_components`、`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`、`std_msgs`、Eigen3。运行还需 `controller_manager(_msgs)`、`rclpy`、`rcl_interfaces`、`sensor_msgs`、`launch(_ros)`。前提是原 Franka 驱动及其依赖已经按其官方方式构建成功；独立离线包不会把硬件驱动伪装成跨平台模块。

## 2. 选择轨迹

优先选择主工程 `data/trajectories/summaries/` 中已有 summary。需要新轨迹时，在离线 Python 环境、主工程根目录生成：

```text
python scripts/generate_ffs_trajectories.py --num-train 10 --num-test 5 --seed 42
```

新 summary 位于输出提示的 `data/trajectories/generated/<run_id>/`。只有这一个轨迹生成入口；旧 ROS 生成器已移入 `reference/trajectory_generation/`，不再安装。

`config/ros_controllers.yaml` 的回起点 `target_q` 必须等于 summary 的 `initial_q`。默认七关节姿态与已有本体数据一致。若更改姿态，应在启动前修改 YAML 并重建，不要只在轨迹 JSON 改一边。回起点控制器发送五次多项式位置曲线的速度导数，当前没有位置闭环纠偏；编排器会另外检查实测位置与速度，无法到达时停止。

本体数据的末端安装状态和机器人负载配置应单独记入实验记录。额外安装待测零件、外置力传感器或 IMU 后的数据不要混入本七连杆本体数据集。

## 3. 单条采集入口

先按已有驱动流程启动真实 FR3、控制器管理器和关节状态发布器。不要同时加载原包中同名 FFS/返回控制器；若已有活动运动控制器，应先按现场控制器管理流程处理接口占用。

下面是参数示例，不会由离线流程自动执行：

```bash
ros2 launch franka_7link_dynamics_id_ros2 collect_ffs.launch.py \
  summary_file:=/absolute/path/to/ffs_summary.json \
  output_base_dir:=/absolute/path/to/franka_7link_dynamics_id/data/acquisitions \
  trajectory_split:=train trajectory_index:=1
```

默认只选 train 的第 1 条。`trajectory_index:=0` 表示所选 split 的全部轨迹；`trajectory_split:=all` 表示同时选择 train/test。先核实单条记录后再决定批量采集。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `summary_file` | 必填 | 含系数和共有元信息的 summary 绝对路径 |
| `output_base_dir` | 用户目录的 `franka_7link_dynamics_id_runs/acquisitions` | 新 run 的父目录 |
| `namespace` | 空 | spawner、编排器的 ROS namespace |
| `controller_manager` | `controller_manager` | 相对当前 namespace 的管理器名，也可给绝对名 |
| `joint_states_topic` | `joint_states` | 同 namespace 的关节状态；全局 topic 可写 `/joint_states` |
| `controller_manager_timeout` | 15 s | spawner 等待管理器 |
| `trajectory_split` | train | train / test / all |
| `trajectory_index` | 1 | 1 起始序号，0 为全部 |
| `collection_rate_hz` | 1000 | `(0,1000]`，不可能高于真实硬件更新频率 |
| `return_to_start_timeout` | 20 s | 回起点实测检查时限 |
| `settle_time` | 3 s | 实测状态连续满足起点条件的持续时间，最少检查 0.5 s |
| `start_position_tolerance` | 0.01 rad | 七个关节均需满足的位置误差 |
| `start_velocity_tolerance` | 0.03 rad/s | 七个关节均需满足的速度界限 |

namespace 只负责 ROS 名称解析，不改变硬件关节名 `fr3_jointN`。管理器和控制器节点必须确实在相应 namespace；需要跨 namespace 读取关节状态时显式设置 topic。

## 4. 编排与失败处理

1. 等待回起点控制器 active、FFS 与采集控制器 inactive。
2. 检查 summary、系数维度、起点、周期和初速度，与返回控制器 `target_q` 对照。
3. 等待新鲜关节状态，确认位置和速度持续满足起点条件；不会单靠固定等待就声称到达。
4. 在 inactive 状态设置 FFS 参数、采集文件名及采样率，检查每个 SetParameters 返回的 `successful`。
5. 使用严格的同次控制器切换，将返回控制器换成 FFS + 采集；避免中途无人占用速度命令接口。
6. 收到完成信号或超时后切回返回控制器；检查切换返回值。失败时停止后续轨迹，并以非零状态退出。

这些检查是流程有效性检查，不替代硬件安全系统。编排器退出后返回控制器可能仍处于 active；由现场人员通过已有驱动方式停止会话。没有关节状态发布器、参数服务失败或起点不符时不会开始 FFS。

## 5. 输出和离线衔接

每次调用新建 `run_<日期_时间_微秒>/`，保存：

```text
summary_snapshot.json       本次实际使用的系数汇总原件
collection_report.json      summary 哈希、设置、逐轨迹成功状态
train/trajectory_...csv
test/trajectory_...csv
```

采集器只读 Franka 内部 state/model 接口，没有外置力传感器或 IMU 输入。CSV 用缓冲区与写盘线程记录；日志中的 dropped samples 应结合预处理的 `max_dt_s` 检查。原始 CSV 含 `q,dq,q_d,dq_d,tau_J,tau_J_d,tau_ext_hat_filtered,O_T_EE,mass,coriolis,gravity` 及时间戳。

不要把输出路径设为既有 `data/raw/train` 或 `data/raw/test`：控制器本身以截断方式创建文件。新 run 目录用于隔离这一行为，原快照保持可追溯。

如果分两次采集 train/test，离线时明确指向两个实际目录：

```text
python scripts/prepare_dataset.py --train-dir data/acquisitions/run_A/train --test-dir data/acquisitions/run_B/test --output-dir data/processed/new_run
python scripts/identify_parameters.py --train data/processed/new_run/train.npz --output-dir results/new_run/identification
python scripts/evaluate_parameters.py --test data/processed/new_run/test.npz --parameters results/new_run/identification/identified_params.npz --output-dir results/new_run/evaluation
```

继续导出和仿真时，同样通过 `--parameters` 传入这一轮 NPZ，并指定对应新轨迹 JSON 和 CSV。单条试采并不构成有充分激励的辨识数据集。
