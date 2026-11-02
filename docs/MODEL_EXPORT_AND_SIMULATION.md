# 从辨识参数替换 URDF 到 MuJoCo 控制对比

本页说明 `scripts/export_model.py` 与 `scripts/run_ffs_simulation.py` 真正执行的变换和控制。入口均使用 Python，不依赖 ROS、xacro、原工作区 mesh 或图形窗口。

## 1. 两种 XML 的关系

URDF 本身也是 XML，但不能只改扩展名就变成 MuJoCo 模型。

```text
identified_params.npz
   ↓ 检查 70+14 排列、参考点、名义模型与物理可行性
models/fr3_nominal.urdf
   ↓ 仅改七个 link 的 inertial，另存副本
fr3_identified.urdf
   ↓ 读回七连杆惯量，检查往返误差
fr3_ffs_nominal.xml / fr3_ffs_identified.xml（MJCF）
   ↓ MuJoCo 实际加载
同轨迹、同控制律仿真 → PNG、NPZ、JSON
```

主流程 `export_models` 先检查模板七连杆惯量等于辨识使用的名义先验，再写新 URDF，读回校验，最后用读回值生成 MJCF。不是从独立参数数组绕过 URDF 随意造另一个模型。

`run_ffs_simulation.py` 会在本次结果的 `models/` 中再执行同样导出，保证那里保存的 XML 就是本次实际使用的模型。

## 2. 模板及替换范围

`models/fr3_nominal.urdf` 保留 FR3 运动学、关节范围和七连杆惯性，外部 `package://.../meshes/...` 视觉依赖已换为简单几何，所以接收者不需要下载 franka_description。这是动力学验证模板，外观不是精细 CAD，碰撞几何也不用于本工程的碰撞规划。

原扩展 URDF、FR3 的 `inertials.yaml/kinematics.yaml/joint_limits.yaml/dynamics.yaml` 和上游许可放在 `reference/fr3_description/`。

导出只修改 `fr3_link1`～`fr3_link7` 的 `<inertial>`；关节变换、关节限制及其他 link 保持输入模板内容。没有将外部物体、Hand 或 IMU 的估计结果写入任一 link。源 URDF 与输出路径相同会被拒绝。

## 3. 平行轴变换

回归参数存的是关于 link 原点 O 的惯量和一阶质量矩：

```text
c   = h/m
I_C = I_O - m*((c^T*c)*E-c*c^T)
```

写入 URDF：

```xml
<link name="fr3_link1">
  <inertial>
    <origin xyz="cx cy cz" rpy="0 0 0"/>
    <mass value="m"/>
    <inertia ixx="I_Cxx" ixy="I_Cxy" ixz="I_Cxz"
             iyy="I_Cyy" iyz="I_Cyz" izz="I_Czz"/>
  </inertial>
</link>
```

上述只是字段含义示意，不是可执行的数字文件。`rpy=0` 使惯性坐标轴与 link 平行，原点放在质心。读取器只支持本 FR3 模板的零惯性 rpy，不是通用 URDF 转换器。当前先验和导出均满足这一条件。

导出前检查质量正且有限、质心有限、质心惯量对称/正定、伪惯量满足物理性。不会把无效惯量简单夹成 `1e-3` 来隐藏问题；MJCF 也设 `balanceinertia=false`，避免引擎自动改写不满足三角不等式的连杆惯量。

## 4. MJCF 运动学与惯量

MJCF 用相同 MDH 构造七层 body，旋转是 `Rx(alpha)*Rz(theta_offset)`，四元数用 MuJoCo 的 `w,x,y,z` 顺序；平移是 `[a,-d*sin(alpha),d*cos(alpha)]`。每个 hinge 轴为局部 z。

`<inertial pos="c" mass="m" fullinertia="..."/>` 写的是质心惯量，排列为 `[Ixx,Iyy,Izz,Ixy,Ixz,Iyz]`。这与 NPZ 的 10 项惯性排列不同，转换由代码完成。

模型固定基座，不加自由关节；七个 actuator 为关节力矩 motor，gear=1。名义/辨识模型使用同样关节限位、简单显示几何和积分设置。接触被关闭，不仿真桌面、碰撞或抓取。不会保留原视觉 mesh 的全部细节，但不会用显示几何自动推算覆盖显式惯量。

## 5. 固定电机/减速器 armature

两模型相同地加入关节侧固定惯量：

```text
armature = rotor_inertia * gear_ratio^2
         = [0.605721456, 0.605721456,
            0.462474144, 0.462474144,
            0.205544064, 0.205544064, 0.205544064] kg*m²
```

来源是归档的 FR3 `dynamics.yaml`；值记录在 `config/fr3_model.yaml` 的 `simulation_joint_armature`，每次仿真报告也记录。

它表示电机/减速器折算到关节的转动惯量，**不是七连杆惯量的辨识结果**，不参与 84 项参数拟合，也不写入 URDF 的 link 惯量。MuJoCo 的质量矩阵因此相当于七连杆质量矩阵再加 `diag(armature)`。测试会把这项加到 Pinocchio 逆动力学中再与 MuJoCo 比较。

保留它是为了明确建模传动侧惯量，避免在 1 ms、较高增益下把名义末端的小连杆惯量直接作为全部关节惯量导致数值发散。它也说明离线回归与闭环仿真的模型范围有差别：本工程没有研究如何从回归中同时分离这些电机惯量；辨识连杆项可能已经吸收部分未建模效应，不能据此宣称所有惯量成分都已唯一分离。

## 6. 仿真控制律

每个模型从同一初始关节位置、零速度开始，跟踪同一 FFS。默认步长 0.001 s，积分器 implicit，以同一时刻的状态和控制力矩保存数据。

```text
e       = q_des-q
de      = dq_des-dq
f(dq)   = mu_c*tanh(dq/0.4) + mu_v*dq
u       = K*e + D*de + bias_model(q,dq) + f(dq)
plant external joint torque = -f(dq)
```

`bias_model` 为各自模型的重力与科氏/离心项；没有 `M*qdd_des` 加速度前馈。两个模型均使用同一份辨识摩擦参数，plant 摩擦抵抗运动，控制器相应补偿。

```text
K = [1500,1500,1500,1500,900,720,360] Nm/rad
D = [3,3,3,3,1.5,1.5,1]             Nm*s/rad
```

仿真沿用较宽的摩擦过渡 `0.4 rad/s`；辨识/力矩评估的 `epsilon` 是 `0.05 rad/s`。该差别明确记录在报告，并非同一摩擦函数的无误差验证。当前 plant 摩擦与控制补偿匹配，因此不是摩擦失配的鲁棒性实验。

控制公式和增益相同，但各模型用自己的 bias 数值补偿，因此控制力矩并不相同。跟踪误差更小本身不能证明该模型更贴近真机。脚本不使用真实驱动器的力矩饱和、通信延迟、内部速度控制环或完整硬件限制；图中的 MuJoCo 力矩与实测 `tau_J` 的对照只能作为有限条件下的参考。

## 7. 如何运行和读取输出

```text
python scripts/export_model.py
python scripts/run_ffs_simulation.py --trajectory data/trajectories/test/trajectory_350_20260515_230214.json --csv data/raw/test/trajectory_350_20260515_230214.csv
```

`results/simulation/ffs_simulation_comparison.png` 的每行对应 J1～J7，每列是位置、速度、力矩、位置误差。灰色是实测，蓝色是名义，红色是辨识，位置面板另有期望曲线。`error_zoom.png` 单独放大两模型的跟踪误差。

`simulation_metrics.json` 中：

- `tracking_rmse`：各模型相对自身期望轨迹的位置误差。
- `comparison_with_measured`：仿真与实测的 `q/dq/tau` 每关节 RMSE，分别为 rad、rad/s、Nm。只在共同时间区间插值，不外插。
- `controller/models`：增益、步长、摩擦宽度、armature、实际 XML 路径。

CSV 的时间零点取第一条记录；实测轨迹与仿真未进行额外延迟辨识或最优相位对齐。实测误差使用其自身 `q-q_d`，仿真误差使用 `q-q_des`。仿真出现非有限状态或 MuJoCo 数值警告时明确停止，不生成新的成功图；若结果目录已有上一轮图，应结合本次退出状态与时间判断。

## 8. 与历史两张图的差别

原图与代码全部保留在 `reference/legacy_ffs_simulation/`，来源和哈希见该目录的 `PROVENANCE.json`。历史保存的期望轨迹对应 **350**；旧脚本默认 391 不能代表已保存那次运行。

| 历史流程的处理 | 当前工程的处理与影响 |
|---|---|
| 末端惯量曾夹到 `1e-3`，且允许 `balanceinertia=true` | 保留通过物理约束得到的实际连杆惯量，显式固定 armature |
| 只把摩擦补偿加到控制力矩，未匹配反向 plant 摩擦 | plant 中加入抵抗运动的摩擦，避免补偿变成人工反阻尼 |
| 模型变换/惯量与原点解释散落在脚本内 | 统一 MDH、平行轴变换与 URDF 读回，并做跨引擎动力学检查 |
| 状态与期望/时间的保存顺序可能错开一个步长 | 同时刻记录状态、期望及本步控制力矩 |
| 旧参数文件缺少新排列/参考点元信息 | 默认重新用已有数据求解，新下游拒绝不明格式 |

本工程复现的是可核查、可交接的完整方法流程，不承诺与历史图逐像素相同。两张历史图可先看效果来源；正式结论应依据当前测试集力矩预测、物理性、可辨识性和明确列出的仿真假设。
