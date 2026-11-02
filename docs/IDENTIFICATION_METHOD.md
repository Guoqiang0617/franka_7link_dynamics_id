# FR3 七连杆本体动力学辨识方法

本文分成两部分。第一部分从反问题和单摆例子开始，解释没有做过辨识的读者需要的数学概念；第二部分对应本工程的实际实现，说明数据预处理、参数排列、求解器设置、验证口径和结果边界。公式统一使用 LaTeX。复现命令见 [完整流程](PROCESS.md)。

**建议阅读顺序：** 先看第一部分理解 $y\approx A\phi$ 的来源，再看第二部分核对代码中的具体约定。只想运行项目时先读 [Windows 与 VS Code 指南](WINDOWS_SETUP.md)；想判断结果是否可信时同时读 [验证记录](VALIDATION.md) 和 [数据与来源](DATA_AND_PROVENANCE.md)。

## 导读：从数据到参数的完整数学链

### 1. 正问题和反问题

机器人动力学的正问题是：已知机器人参数和运动，计算关节需要的力矩：

$$
(q,\dot q,\ddot q,\phi)\longrightarrow\hat{\tau}.
$$

动力学辨识做的是反问题：已经观测到运动和力矩，反过来寻找参数：

$$
(q,\dot q,\ddot q,\tau)\longrightarrow\phi.
$$

其中，$q$ 是七个关节角，$\dot q$ 是角速度，$\ddot q$ 是角加速度，$\tau$ 是测得的七维关节力矩，$\hat{\tau}$ 是模型预测力矩，$\phi$ 是待估参数。

实际数据更接近：

$$
\tau
=
f(q,\dot q,\ddot q;\phi)
+
\varepsilon_{\mathrm{measurement}}
+
\varepsilon_{\mathrm{model}}.
$$

前一项包含传感器、量化和数值微分误差，后一项包含模型没有显式写入的电机转子惯量、减速器柔性、温度、外部负载和力矩零偏等。辨识器只能调整模型中已经定义的参数，所以未建模项可能被吸收到惯性或摩擦参数中。这是为什么预测误差下降不自动等于每个物理参数都被独立、准确测量。

### 2. 一个单摆例子

假设单摆的质量 $m$ 集中在距转轴 $l$ 的摆端，$q$ 从竖直向下方向测量，忽略摩擦，则驱动力矩可写为：

$$
\tau
=
ml^2\ddot q+mgl\sin q
=
\underbrace{\left(l^2\ddot q+gl\sin q\right)}_{\text{当前运动决定的已知系数}}
m.
$$

第 $k$ 个采样时刻就是一条关于 $m$ 的方程：

$$
\tau^{(k)}=a^{(k)}m.
$$

采集很多不同角度和加速度，就能估计 $m$。FR3 项目只是把这个结构扩大：未知数从 1 个变成 84 个，一条标量方程变成七个关节方程，标量系数变成动力学回归矩阵，许多状态上下堆叠成大方程组。

最核心的形式始终是：

$$
\text{观测力矩}
\approx
\text{由运动和几何计算的系数}
\times
\text{未知参数}.
$$

### 3. 84 个参数从哪里来

每个活动连杆使用 10 个惯性参数：

$$
\theta_i=
\begin{bmatrix}
m_i&
h_{x,i}&h_{y,i}&h_{z,i}&
I_{xx,O,i}&I_{xy,O,i}&I_{yy,O,i}&
I_{xz,O,i}&I_{yz,O,i}&I_{zz,O,i}
\end{bmatrix}^{\mathsf T}.
$$

这里 $m_i$ 是质量，$h_i=m_ic_i$ 是一阶质量矩，$I_{O,i}$ 是关于连杆坐标系原点的惯量矩阵。七个连杆共 70 个参数。

七个关节各有平滑库仑摩擦系数和黏性摩擦系数：

$$
\tau_{f,j}(\dot q_j)
=
\mu_{c,j}\tanh\left(\frac{\dot q_j}{\varepsilon}\right)
+
\mu_{v,j}\dot q_j,
\qquad
\varepsilon=0.05\ \mathrm{rad/s}.
$$

七个关节共 14 个参数，因此：

$$
\phi=
\begin{bmatrix}
\theta\\
\mu
\end{bmatrix}
\in\mathbb{R}^{84},
\qquad
70+14=84.
$$

84 表示模型有 84 个待估数字，不表示实验一定能把 84 个原始数字全部独立确定。

### 4. 一个运动状态如何变成七条方程

对第 $k$ 个状态，程序有 $q^{(k)}$、$\dot q^{(k)}$、$\ddot q^{(k)}$ 和 $\tau^{(k)}$。Pinocchio 根据当前运动状态和已知运动学计算：

$$
Y^{(k)}\in\mathbb{R}^{7\times70}.
$$

摩擦矩阵为：

$$
F^{(k)}\in\mathbb{R}^{7\times14},
$$

其中关节 $j$ 的两列是：

$$
F^{(k)}_{j,\mu_{c,j}}
=
\tanh\left(\frac{\dot q_j^{(k)}}{\varepsilon}\right),
\qquad
F^{(k)}_{j,\mu_{v,j}}
=
\dot q_j^{(k)}.
$$

拼接后：

$$
A^{(k)}
=
\begin{bmatrix}
Y^{(k)}&F^{(k)}
\end{bmatrix}
\in\mathbb{R}^{7\times84},
\qquad
\tau^{(k)}\approx A^{(k)}\phi.
$$

一个状态只有 7 条方程，必须用很多状态共同约束同一组参数：

$$
A=
\begin{bmatrix}
A^{(1)}\\
A^{(2)}\\
\vdots\\
A^{(N)}
\end{bmatrix},
\qquad
y=
\begin{bmatrix}
\tau^{(1)}\\
\tau^{(2)}\\
\vdots\\
\tau^{(N)}
\end{bmatrix},
\qquad
y\approx A\phi.
$$

代码将力矩按每个样本 J1 到 J7、再到下一个样本的顺序展平，所以矩阵行和目标元素严格对应。

### 5. 为什么对运动非线性、对参数线性

常见的刚体动力学写法是：

$$
\tau_{\mathrm{rigid}}
=
M(q)\ddot q+C(q,\dot q)\dot q+g(q).
$$

$M$、$C$ 和 $g$ 对姿态、速度和加速度的关系很复杂，但它们由同一组刚体惯性决定。对一个连杆，设原点线速度为 $v_O$，角速度为 $\omega$，一阶质量矩为 $h=mc$，原点惯量为 $I_O$：

$$
T_i
=
\frac12m_i\lVert v_{O,i}\rVert^2
+
v_{O,i}^{\mathsf T}(\omega_i\times h_i)
+
\frac12\omega_i^{\mathsf T}I_{O,i}\omega_i,
$$

$$
U_i=m_i a_i(q)+h_i^{\mathsf T}b_i(q).
$$

上面 $v_O$、$\omega$、$h$ 和 $I_O$ 使用同一个连杆坐标系表示。质量、质量矩和惯量只以一次项出现。把所有连杆相加并代入欧拉—拉格朗日方程：

$$
\tau_{\mathrm{rigid}}
=
\frac{\mathrm d}{\mathrm dt}
\left(\frac{\partial T}{\partial\dot q}\right)
-
\frac{\partial T}{\partial q}
+
\frac{\partial U}{\partial q},
$$

就得到：

$$
\tau_{\mathrm{rigid}}
=
Y(q,\dot q,\ddot q)\theta.
$$

所以运动可以以 $\sin q$、速度乘积等非线性方式进入 $Y$，但参数 $\theta$ 仍然只以一次方出现。程序先为每个状态计算 $Y$，再求解关于参数的回归问题。

### 6. 原点惯量和质心惯量

回归使用关于连杆坐标系原点的 $I_O$：

$$
I_O=
\begin{bmatrix}
I_{xx,O}&I_{xy,O}&I_{xz,O}\\
I_{xy,O}&I_{yy,O}&I_{yz,O}\\
I_{xz,O}&I_{yz,O}&I_{zz,O}
\end{bmatrix}.
$$

质心是：

$$
c=\frac{h}{m}.
$$

平行轴定理为：

$$
I_O
=
I_C+
m\left((c^{\mathsf T}c)I_3-cc^{\mathsf T}\right),
$$

因此：

$$
I_C
=
I_O-
m\left((c^{\mathsf T}c)I_3-cc^{\mathsf T}\right).
$$

CAD 先验若提供的是质心惯量，必须先转换为原点惯量才能进入回归；导出 URDF 时再转回质心惯量。混用二者会使惯性力矩预测产生系统误差。

### 7. 加速度如何从 CSV 得到

代码先把纳秒时间转换为相对秒：

$$
t_n
=
\left(ros\_time\_ns[n]-ros\_time\_ns[0]\right)10^{-9},
$$

再按实际时间间隔求导：

$$
\ddot q[n]
\approx
\operatorname{gradient}(\dot q,t)[n].
$$

之后对 $\ddot q$ 使用 Savitzky–Golay 平滑，默认窗口为 21 个原始样本、三阶多项式。$q$、$\dot q$ 和 $\tau_J$ 不作额外平滑，所有信号使用同一组 stride 索引抽样。

数值微分会放大速度噪声，边界和掉帧处还可能产生异常加速度。平滑能降低高频噪声，但不能修复错误时间戳或恢复丢失采样。IRLS 只能降低异常方程的影响，不能替代数据质量检查。

### 8. 为什么要设计激励轨迹

辨识需要的不只是很多行数据，而是“信息不同”的数据：

- 改变姿态，改变重力和几何项；
- 改变速度，改变科氏/离心项和摩擦项；
- 改变加速度，改变惯性项；
- 改变关节间的相位和谐波，减少回归列之间的相似性。

本工程使用有限傅里叶轨迹。令 $\omega=2\pi f_0$：

$$
\begin{aligned}
q_j(t)
&=
q_{j0}
+
\sum_{k=1}^{K}
\left[
\frac{a_{jk}}{k\omega}\sin(k\omega t)
-
\frac{b_{jk}}{k\omega}
\left(\cos(k\omega t)-1\right)
\right],\\
\dot q_j(t)
&=
\sum_{k=1}^{K}
\left[
a_{jk}\cos(k\omega t)+b_{jk}\sin(k\omega t)
\right],\\
\ddot q_j(t)
&=
\sum_{k=1}^{K}
\left[
-k\omega a_{jk}\sin(k\omega t)
+k\omega b_{jk}\cos(k\omega t)
\right].
\end{aligned}
$$

当前新生成配置为 $K=5$、$f_0=0.1\ \mathrm{Hz}$、$T=10\ \mathrm{s}$，最高谐波为 $0.5\ \mathrm{Hz}$。轨迹生成器还以 0.8 的余量检查位置、速度和加速度限制，并在不满足时拒绝采样或缩放。

FFS 改善激励，但不保证回归矩阵满秩，也不包含碰撞检查。项目中已有轨迹直接复用，不假设可以由新的随机种子逐项重建。

### 9. 可辨识性：为什么样本多不等于参数都能测出

考虑一个只有两个参数的玩具模型：

$$
y_k=x_{k1}\theta_1+x_{k2}\theta_2.
$$

如果所有采样都满足 $x_{k2}=2x_{k1}$，那么：

$$
y_k=x_{k1}(\theta_1+2\theta_2).
$$

数据只能确定组合 $\theta_1+2\theta_2$，不能分别确定两个参数。改变采样数量不会消除这个列相关，必须改变激励或引入结构先验。

在机器人辨识中，$A$ 的每一列是一个参数对全部观测的响应。代码对未加鲁棒权重的正规矩阵做特征分解：

$$
A^{\mathsf T}A=V\Lambda V^{\mathsf T}.
$$

若：

$$
\lambda_i<(10^{-5})^2\lambda_{\max},
$$

就把该方向归入数值零空间。若零空间维数为 $d$：

$$
\operatorname{rank}(A)=84-d.
$$

当前 full 结果为秩 $57/84$、零空间维数 27。这个数字表示 57 个参数组合方向在当前数据和数值尺度下有明显响应，不表示 57 个具体原始参数已经被逐个称量出来。

如果 $Av\approx0$，则：

$$
A(\phi+cv)=A\phi+cAv\approx A\phi.
$$

沿 $v$ 改变参数，预测力矩几乎不变。物理约束可以排除不合法解，但不能创造实验没有提供的信息；先验用于在剩余解中选择一个代表。

### 10. 数据、先验和物理约束的分工

三者作用不同：

1. 数据要求 $A\phi$ 接近测得力矩 $y$；
2. CAD 和摩擦先验在数据看不见的方向上提供参考；
3. 物理约束排除不可能的质量—质心—惯量组合。

带先验的加权最小二乘为：

$$
\min_{\phi}
\frac12(y-A\phi)^{\mathsf T}W(y-A\phi)
+
\frac12(\phi-\phi_0)^{\mathsf T}R(\phi-\phi_0).
$$

忽略不等式约束时：

$$
\left(A^{\mathsf T}WA+R\right)\phi
=
A^{\mathsf T}Wy+R\phi_0.
$$

零空间基为 $V_0$ 时，代码构造：

$$
R=
\lambda_{\mathrm{data}}I+
\left(\lambda_{\mathrm{null}}-\lambda_{\mathrm{data}}\right)V_0V_0^{\mathsf T},
$$

$$
\lambda_{\mathrm{data}}=10^{-4},
\qquad
\lambda_{\mathrm{null}}=1.
$$

这个设置倾向于让数据看得见的方向更多由测量决定，让数值零空间方向更靠近 CAD/摩擦先验；实际结果还受物理约束、样本数量、单位尺度和鲁棒权重影响。先验不是新增实验测量，也不是强行让所有结果等于 CAD。

### 11. 伪惯量约束为什么代表真实刚体

仅要求质量为正还不够；惯量必须和质量、质心相互兼容。定义：

$$
\Sigma
=
\frac12\operatorname{tr}(I_O)I_3-I_O,
$$

$$
P=
\begin{bmatrix}
\Sigma&h\\
h^{\mathsf T}&m
\end{bmatrix}.
$$

理论上真实刚体的伪惯量满足 $P\succeq0$；代码为了数值稳定，使用更严格的下界：

$$
P\succeq\delta I_4,
\qquad
\delta=10^{-6}.
$$

这里 $\succeq$ 表示矩阵半正定序，不是逐元素比较。由于 $P$ 的元素对 $m,h,I_O$ 是线性的，这个条件是线性矩阵不等式，可以和凸二次目标一起交给 CVXPY。

它能排除负质量、负二阶质量矩和违反惯量三角不等式的组合，但不能保证质心位于 CAD 外形内，也不能解决实验零空间。

### 12. PC-IRLS 每轮做什么

普通最小二乘让所有方程权重相同；时间戳异常、数值微分尖峰或瞬时力矩异常可能产生大残差。PC-IRLS 用 Tukey 权重降低异常方程影响。

当前参数为 $\phi^{(t)}$ 时：

$$
r^{(t)}=y-A\phi^{(t)}.
$$

共同尺度：

$$
s^{(t)}
=
\frac{4.685\,\operatorname{median}(|r^{(t)}|)}{0.6745}
+
10^{-8}.
$$

每条方程的权重：

$$
w_l^{(t)}
=
\begin{cases}
\left(1-(r_l^{(t)}/s^{(t)})^2\right)^2,
& |r_l^{(t)}|\le s^{(t)},\\
0,&|r_l^{(t)}|>s^{(t)}.
\end{cases}
$$

令 $W^{(t)}=\operatorname{diag}(w^{(t)})$，固定权重后求解：

$$
\begin{aligned}
\phi^{(t+1)}
=
\underset{\phi}{\operatorname{arg\,min}}\quad
&
\frac12(y-A\phi)^{\mathsf T}W^{(t)}(y-A\phi)\\
&
+\frac12(\phi-\phi_0)^{\mathsf T}R(\phi-\phi_0)
\\
\text{subject to}\quad
&
P_i(\phi)\succeq\delta I_4,\quad
\mu_{c,j}\ge0,\quad \mu_{v,j}\ge0.
\end{aligned}
$$

固定 $W^{(t)}$ 后，这是凸二次目标加凸约束。求解后比较：

$$
\rho^{(t)}
=
\frac{\lVert\phi^{(t+1)}-\phi^{(t)}\rVert}
{\lVert\phi^{(t)}\rVert+10^{-12}}.
$$

当 $\rho^{(t)}<10^{-4}$ 时认为外循环收敛；full 最多 50 轮，quick 最多 5 轮。固定权重的子问题是凸的，但权重会随参数改变，所以整个 IRLS 是迭代过程；收敛只表示参数更新足够小，不是全局最优证明。

### 13. 测试阶段如何判断结果是否有用

训练集用于求 $\phi$，测试集只用于评估。完整动力学预测：

$$
\hat\tau_{\mathrm{full}}
=
Y(q,\dot q,\ddot q)\theta+F(\dot q)\mu.
$$

对照模式用 CSV 中的重力力矩替换模型重力，并保留摩擦项：

$$
\hat\tau_{\mathrm{replaced}}
=
\left[Y(q,\dot q,\ddot q)-Y(q,0,0)\right]\theta
+
\tau_{\mathrm{gravity,csv}}
+
F(\dot q)\mu.
$$

完整动力学是主要指标，因为它完全使用辨识模型。gravity replaced 借用了机器人内部重力估计，不能单独证明辨识出的重力参数正确。

第 $j$ 个关节误差：

$$
e_{k,j}=\hat\tau_{k,j}-\tau_{k,j}.
$$

逐关节指标：

$$
\operatorname{RMSE}_j
=
\sqrt{\frac1N\sum_{k=1}^{N}e_{k,j}^2},
\qquad
\operatorname{MAE}_j
=
\frac1N\sum_{k=1}^{N}|e_{k,j}|,
$$

$$
R_j^2
=
1-
\frac{\sum_ke_{k,j}^2}
{\sum_k(\tau_{k,j}-\bar\tau_j)^2}.
$$

总体 RMSE：

$$
\operatorname{RMSE}_{\mathrm{all}}
=
\sqrt{\frac1{7N}\sum_{j=1}^{7}\sum_{k=1}^{N}e_{k,j}^2}.
$$

应分开回答三个问题：

1. 预测能力是否改善；
2. 参数是否满足刚体和摩擦物理约束；
3. 秩、零空间、先验偏离和未建模因素是否允许把参数当作真实物理属性。

例如测试误差下降但某个质量偏离 CAD，这两件事并不矛盾：模型可能找到了一个物理可行、预测更好的代表解，但质量变化位于弱可辨识方向。
## 第二部分：当前工程的实现约定

下面各节把第一部分的数学对象对应到本工程当前代码、数据格式和输出文件。参数范围、求解器和报告中的具体数字以当前配置和数据为准。

## 1. 对象与假设

对象是 Franka **FR3** 的 `fr3_link1`～`fr3_link7` 七个活动刚体，以及七个关节的简单摩擦。固定基座不估计；

模型假设关节运动学固定、连杆为刚体、重力为 `[0,0,-9.81] m/s²`，主要以七连杆惯性和关节摩擦解释测得力矩。电机转子惯量、减速器柔性、传动误差、温度依赖、力矩零偏等未作为待估参数。因此残差不仅来自测量噪声；未建模项也会被惯性或摩擦吸收。

数据列不能单独证明采集时末端空载。若原机器人装有额外物体，把该数据用于本模型会使结果偏离本体真实参数。包内保留本体 FFS 流程的数据快照，物理安装状态的证据边界见 [数据与来源](DATA_AND_PROVENANCE.md)。

## 2. 坐标与参数排列

### 2.1 七轴运动学

`config/fr3_model.yaml` 中的修正 DH 参数按 `[alpha,a,d,theta_offset]` 排列。关节固定变换为：

$$
\begin{aligned}
T_{\mathrm{fixed}} &= R_x(\alpha)\,T_x(a)\,R_z(\theta_{\mathrm{offset}})\,T_z(d),\\
T(q) &= T_{\mathrm{fixed}}\,R_z(q).
\end{aligned}
$$

旋转矩阵为 `Rx(alpha) Rz(theta_offset)`，平移为 `[a,-d*sin(alpha),d*cos(alpha)]`。关节轴为局部 z；关节后附着的刚体坐标系对应相应 URDF link 坐标系。测试检查了同一状态下 MDH Pinocchio、URDF Pinocchio、MuJoCo 的逆动力学一致性。

### 2.2 每连杆 10 项惯性参数

对连杆 i，用 link 坐标系表示：

$$
\begin{aligned}
\theta_i &=
\begin{bmatrix}
m & h_x & h_y & h_z & I_{xx,O} & I_{xy,O} & I_{yy,O} & I_{xz,O} & I_{yz,O} & I_{zz,O}
\end{bmatrix}^{\mathsf T},\\
h &= m\,c,\\
I_O &= I_C + m\left((c^{\mathsf T}c)E - cc^{\mathsf T}\right).
\end{aligned}
$$

`m` 单位 kg，质心 `c` 单位 m，一阶质量矩 `h` 单位 kg·m，惯量单位 kg·m²。`I_O` 关于 link 原点；`I_C` 关于质心，轴方向与 link 相同。这里存的是惯量张量的实际非对角元素，不要再次添加负号；顺序也不同于 MJCF `fullinertia` 顺序。

先验来自归档的 FR3 `inertials.yaml`，先经平行轴定理转换为 `I_O`，然后参与回归。不能把 URDF 的质心惯量直接当成原点惯量。

### 2.3 每关节 2 项摩擦

$$
\begin{aligned}
\tau_{f,j}(\dot q_j)
&= \mu_{c,j}\tanh\!\left(\frac{\dot q_j}{\varepsilon}\right)
  + \mu_{v,j}\dot q_j,\\
\varepsilon &= 0.05\ \mathrm{rad/s},\\
\mu_{c,j} &\ge 0,\qquad \mu_{v,j}\ge 0.
\end{aligned}
$$

`mu_c` 单位 Nm；`mu_v` 单位 Nm·s/rad。输出向量按以下顺序共 84 项：

$$
\phi =
\begin{bmatrix}
\theta_1 & \cdots & \theta_7 & \mu_{c,1} & \mu_{v,1} & \cdots & \mu_{c,7} & \mu_{v,7}
\end{bmatrix}^{\mathsf T},
\qquad 70+14=84.
$$

当前实现只有此 simple 模型；不提供另一套 Stribeck 98 项模型。NPZ 元信息标记 `parameter_layout=rigid70_friction14`、`inertia_reference=link_origin`。下游读取器会拒绝缺少这些标记的历史文件或其他排列，防止混用零件辨识结果。

## 3. 有限傅里叶轨迹

对每个关节 j，令 `omega=2*pi*f0`：

$$
\begin{aligned}
q_j(t) &= q_{j0} + \sum_k\left[
 \frac{a_{jk}}{k\omega}\sin(k\omega t)
 - \frac{b_{jk}}{k\omega}\bigl(\cos(k\omega t)-1\bigr)
\right],\\
\dot q_j(t) &= \sum_k\left[a_{jk}\cos(k\omega t)+b_{jk}\sin(k\omega t)\right],\\
\ddot q_j(t) &= \sum_k\left[-k\omega a_{jk}\sin(k\omega t)
 + k\omega b_{jk}\cos(k\omega t)\right].
\end{aligned}
$$

当前新生成配置为 `K=5`、`f0=0.1 Hz`、`T=10 s`，最高谐波 0.5 Hz。初始位置直接等于 `q_j0`。设置 `Σ a_jk=0` 和 `Σ k*b_jk=0`，再要求 `T*f0` 为整数，可使两端速度、加速度为零。生成器通过最后一个系数满足等式，所以“随机系数初始范围”不保证约束后的每一项仍在该范围。

新生成器按关节拒绝采样，以 0.8 的余量检查离散位置、速度和加速度界限；若用完次数仍不满足，会尝试缩放。既有轨迹不重新生成，也不假定能由新随机种子逐项重建。当前算法不是信息矩阵优化，也没有碰撞检查。

## 4. CSV、时间与加速度

辨识只读取 `ros_time_ns`、`period_ns`、`q_0..6`、`dq_0..6`、`tau_J_0..6`、`gravity_0..6`。关节列编号 0..6 对应 J1..J7。原始文件的其他列原样保留，但不进入拟合。

每条 CSV 独立进行：

1. 用 int64 纳秒时间先算 `t_ns-t_ns[0]`，再乘 `1e-9`，避免大绝对时间转浮点丢精度。
2. 检查数值有限，时间严格递增。少于 3 行的记录保留原件、跳过并报告；其他数据问题直接报错。
3. 执行 `numpy.gradient(dq, time, axis=0, edge_order=2)`，按实际不均匀时间间隔求导。
4. 对所得 `qdd` 作 Savitzky–Golay 平滑，默认窗口 21 个原始样本，三阶多项式；短文件缩短为可用奇数窗口。
5. 对所有信号取相同索引 `0,stride,2*stride,...`；full 默认 stride=10，quick=50。

`q`、`dq`、`tau_J` 没有另外平滑。这里是“先微分、再平滑加速度”，不是 SG 导数公式。SG 平滑以样本索引等距处理，明显掉帧时不能等同于精确的不规则时间滤波；可查预处理报告中的 `median_dt_s/max_dt_s`。微分、边界点和掉帧都会影响误差。本工程保留轨迹边界，不额外剪切异常段；IRLS 只降低异常方程的权重，不删除原始记录。

## 5. 线性动力学回归

刚体逆动力学：

$$
\begin{aligned}
\tau
&= M(q)\,\ddot q + C(q,\dot q)\,\dot q + g(q) + \tau_f(\dot q)\\
&= Y(q,\dot q,\ddot q)\,\theta + F(\dot q)\,\mu\\
&= A(q,\dot q,\ddot q)\,\phi.
\end{aligned}
$$

`Y` 为 Pinocchio `computeJointTorqueRegressor` 输出的 `7×70` 矩阵；`F` 为按关节排列的 `tanh(dq/epsilon)` 和 `dq` 列，大小 `7×14`。N 个状态按样本顺序堆成 `A∈R^(7N×84)` 与 `y=vec(tau_J)`。实际装入模型的占位惯量不决定回归矩阵；先验向量才提供具体名义值。

训练目标直接是测得 `tau_J`，回归中包括模型重力，**不先从目标减掉 gravity**。`tau_J_d` 和 `tau_ext_hat_filtered` 不能代替它。本工程沿用原数据接口中 `tau_J` 的测量力矩含义；换采集接口或控制模式后要重新核实信号定义。

## 6. 秩、零空间与先验

先对未加鲁棒权重的 $A^{\mathsf T}A$ 做对称特征分解，得到特征值 $\lambda_i$ 与正交特征向量。使用 `svd_threshold=1e-5`：

$$
\lambda_i < (10^{-5})^2\,\lambda_{\max}
\quad\Longrightarrow\quad
\text{null direction},
\qquad
\operatorname{rank}(A)=84-\operatorname{nullity}.
$$

这等价于对奇异值作相对阈值判断，但由正规矩阵计算，且没有额外列归一化；阈值、单位尺度和数值精度都会影响“数值秩”。不能把本次报告的 57 无条件当作所有 FR3 实验的普适理论基参数个数。

设 `V0` 包含零空间方向，构造：

$$
\begin{aligned}
R &= \lambda_{\mathrm{data}}E
 + \left(\lambda_{\mathrm{null}}-\lambda_{\mathrm{data}}\right)V_0V_0^{\mathsf T},\\
\lambda_{\mathrm{data}} &= 10^{-4},\qquad
\lambda_{\mathrm{null}}=1,\\
\phi_0 &= \text{名义 CAD 参数与摩擦先验}.
\end{aligned}
$$

数据可分辨的方向施加弱先验；零空间方向施加强先验。权重与不同物理单位未标准化，因此改变单位、抽样数量、噪声或数据范围时，应重新评估正则设置。默认值用于忠实复现当前数据流程，不是普适统计最优值。

若 $Av\approx0$，则 $\phi$ 与 $\phi+cv$ 预测力矩近似相同，单个参数无法仅由这些数据区分。物理约束能排除无效惯量，但不会自动创造缺失的信息；先验给出的是选解依据。

## 7. PC-IRLS 求解

PC-IRLS 在这里指 **Physically Consistent Iteratively Reweighted Least Squares**，即带物理约束的迭代加权最小二乘。初始化 `phi=phi0`。

### 7.1 Tukey 鲁棒权重

对每条标量关节方程 l，计算：

$$
\begin{aligned}
r &= y-A\phi,\\
s &= \frac{4.685\,\operatorname{median}(|r|)}{0.6745}+10^{-8},\\
w_l &=
\begin{cases}
\left(1-(r_l/s)^2\right)^2, & |r_l|\le s,\\
0, & |r_l|>s.
\end{cases}
\end{aligned}
$$

代码使用所有关节残差的共同尺度。这里的 `median(abs(r))` 是相对零的绝对残差中位数，**不是减去残差中位数后的中心化 MAD**，也没有逐关节估计噪声方差。Tukey 权重降低大残差方程的影响；它不能修复错误时间戳、信号定义或建模假设。

### 7.2 固定权重下的受约束二次问题

每次求解：

$$
\begin{aligned}
\min_{\phi}\quad
&\frac{1}{2}(y-A\phi)^{\mathsf T}W(y-A\phi)
+\frac{1}{2}(\phi-\phi_0)^{\mathsf T}R(\phi-\phi_0),\\
&W=\operatorname{diag}(w).
\end{aligned}
$$

等价的 Hessian 与线性项是：

$$
H=A^{\mathsf T}WA+R,
\qquad
b=A^{\mathsf T}Wy+R\phi_0,
\qquad
\min_{\phi}\ \frac12\phi^{\mathsf T}H\phi-b^{\mathsf T}\phi.
$$

实现分块累加 `H,b`，不额外复制整块加权回归矩阵。损失没有除以样本数，因此抽样改变也会改变数据项相对先验的有效比例。

### 7.3 伪惯量 LMI

对每个连杆，用原点惯量构造对参数线性的伪惯量：

$$
\begin{aligned}
\Sigma &= \frac12\operatorname{tr}(I_O)E-I_O,\\
P &= \begin{bmatrix}\Sigma & h\\ h^{\mathsf T} & m\end{bmatrix},\\
P &\succeq \delta E_4,\qquad \delta=10^{-6}.
\end{aligned}
$$

`>=` 表示矩阵半正定序，不是逐元素比较。这一条件约束质量及二阶质量矩，确保对应惯量物理一致，并蕴含质心惯量的三角不等式。δ 是当前 SI 数值尺度下的稳定化下界；不是带统一物理单位的材料常数，也不是几何包络约束。质心并未强制位于 CAD 外形内。

同时要求 14 个摩擦系数非负。CVXPY 优先使用 CLARABEL，再尝试 SCS；检查返回解的有限性、伪惯量最小特征值和摩擦残差。两者均不满足时停止，默认不会静默切换算法。

显式选择 `--solver projection` 时，先解未加 LMI 的二次问题，再截断伪惯量负/小特征值及负摩擦；这是近似投影步骤，不保证与受约束二次问题的最优解相同。结果会记录算法路径，不能与默认 SDP 当作同一次实验。

### 7.4 收敛与记录

$$
\operatorname{relative\_change}
=\frac{\lVert\phi_{\mathrm{new}}-\phi_{\mathrm{old}}\rVert}
{\lVert\phi_{\mathrm{old}}\rVert+10^{-12}},
\qquad
\operatorname{relative\_change}<10^{-4}
\ \Longrightarrow\ \text{收敛}.
$$

full 配置最多 50 次，quick 覆盖为 5 次。每次记录参数相对变化、普通未加权 RMSE、非零权重比例和求解器状态。IRLS 是重加权外循环；不能因每个凸子问题求解成功就声称整个鲁棒问题获得全局最优。

## 8. 训练、测试及统计含义

测试数据不参与参数求解。测试集同时给出两种力矩：

$$
\begin{aligned}
\text{full dynamics:}\quad
\hat\tau &=Y(q,\dot q,\ddot q)\theta+F(\dot q)\mu,\\
\text{gravity replaced:}\quad
\hat\tau &=\bigl(Y(q,\dot q,\ddot q)-Y(q,0,0)\bigr)\theta
+\tau_{\mathrm{gravity,csv}}+F(\dot q)\mu.
\end{aligned}
$$

完整动力学是主指标。重力替换用于对照旧验证流程，隔离模型重力差异；它借用了机器人内部重力估计，不能检验辨识重力参数是否准确。

每关节计算 $\mathrm{RMSE}=\sqrt{\operatorname{mean}(e^2)}$、$\mathrm{MAE}=\operatorname{mean}(|e|)$、$R^2=1-\mathrm{SSE}/\mathrm{SST}$。总体 RMSE 对全部样本与关节共同取均方根，不是简单平均七个 RMSE。较长记录获得更多样本权重。当前没有估计每参数标准差、置信区间、协方差或独立称重误差，不能从 `parameter_table.csv` 推出这些统计量。

现有 train/test 使用不同激励范围，但部分记录属于同一原采集体系，不应当成跨日期、温度、载荷的全面泛化验证。完整结果与逐关节数值见 [验证记录](VALIDATION.md)。

## 9. 如何理解这次七连杆结果

full 使用 143359 个训练状态、473975 个测试状态，数值秩 **57/84**、零空间维数 27。默认求解 25 次满足收敛阈值。测试完整动力学总体 RMSE 为名义 1.282252 Nm、辨识 1.018111 Nm。

部分连杆的质量明显偏离名义先验。报告逐连杆列出质量、质心、质心惯量、质量比和伪惯量特征值；质量比低于 0.5 或高于 2 会附解释备注。**物理可行、预测改善、参数接近真实值是三种不同判断**。本次得到的是当前建模、数据、先验和约束下的可行代表解，不是 84 项参数的唯一精确恢复，也不是逐连杆称重结果。

若研究目标要求独立确定各连杆真实质量/重心，需要额外结构先验、几何约束、独立测量或对基参数重新设计实验，并评估未建模电机等项。本工程没有为美化质量结果事后调先验。导出的 URDF 应作为离线对比模型使用。

## 10. 导出与验证衔接

先用 $c=h/m$ 和平行轴定理恢复 $I_C$，检查正质量、有限值、质心惯量与伪惯量物理性，写入七个 URDF `<inertial>`。读回校验后生成 MuJoCo XML。仿真额外固定的电机/减速器 armature 不参与上述回归；这使仿真模型比七连杆回归模型多一个明确记录的固定项。控制律、摩擦平滑宽度差异及历史图的修正详见 [模型导出与仿真](MODEL_EXPORT_AND_SIMULATION.md)。
