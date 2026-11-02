#!/usr/bin/env python3
"""
步骤 5: FFS 轨迹 MuJoCo 仿真（与真机控制一致）
==============================================

目标：
1. 从 FFS 数学公式生成轨迹（不使用 CSV 回放）
2. 实现与 FCI 一致的速度前馈 + 内部阻抗控制
3. 使用名义参数和辨识参数分别仿真
4. 与真机数据对比验证辨识效果
"""
import numpy as np
import mujoco
import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt

# 添加 utils 模块到路径
sys.path.append(str(Path(__file__).parent))


# ============================================================
# FFS 轨迹生成
# ============================================================

class FFSTrajectory:
    """FFS (Finite Fourier Series) 轨迹生成器"""

    def __init__(self, json_path):
        """
        从 JSON 文件加载 FFS 参数

        Args:
            json_path: FFS 轨迹 JSON 文件路径
        """
        with open(json_path, 'r') as f:
            data = json.load(f)

        self.q0 = np.array(data['initial_q'])  # (7,)
        self.ak = np.array(data['ak'])         # (7, 5)
        self.bk = np.array(data['bk'])         # (7, 5)
        self.omega_f = data['omega_f']         # rad/s
        self.duration = data['duration']       # seconds
        self.num_harmonics = data['num_harmonics']  # 5

        self.k = np.arange(1, self.num_harmonics + 1)  # [1, 2, 3, 4, 5]
        self.kw = self.k * self.omega_f  # (5,)

        print(f"Loaded FFS trajectory from {Path(json_path).name}")
        print(f"  Duration: {self.duration} s")
        print(f"  ω_f: {self.omega_f:.4f} rad/s")
        print(f"  Harmonics: {self.num_harmonics}")

    def compute_q(self, t):
        """
        计算位置 q(t)

        q(t) = q0 + Σ_{k=1}^N [a_k/(k·ω_f)·sin(k·ω_f·t) - b_k/(k·ω_f)·(cos(k·ω_f·t)-1)]

        Args:
            t: 时间（标量或数组）

        Returns:
            q: (N, 7) 或 (7,) 位置
        """
        t = np.atleast_1d(t)
        arg = np.outer(t, self.kw)  # (N, 5)
        s = np.sin(arg)  # (N, 5)
        c = np.cos(arg)  # (N, 5)

        # ak, bk: (7, 5) -> (5, 7)
        # kw: (5,) -> (5, 1)
        ak_scaled = (self.ak.T / self.kw[:, None])  # (5, 7)
        bk_scaled = (self.bk.T / self.kw[:, None])  # (5, 7)

        # s[:, :, None]: (N, 5, 1) * ak_scaled[None, :, :]: (1, 5, 7) -> (N, 5, 7)
        q = self.q0[None, :] + (s[:, :, None] * ak_scaled[None, :, :]).sum(axis=1) \
            - ((c - 1.0)[:, :, None] * bk_scaled[None, :, :]).sum(axis=1)

        return q.squeeze() if len(t) == 1 else q

    def compute_dq(self, t):
        """
        计算速度 dq(t)

        dq(t) = Σ_{k=1}^N [a_k·cos(k·ω_f·t) + b_k·sin(k·ω_f·t)]

        Args:
            t: 时间（标量或数组）

        Returns:
            dq: (N, 7) 或 (7,) 速度
        """
        t = np.atleast_1d(t)
        arg = np.outer(t, self.kw)  # (N, 5)
        s = np.sin(arg)  # (N, 5)
        c = np.cos(arg)  # (N, 5)

        # ak: (7, 5) -> (5, 7)
        dq = (c[:, :, None] * self.ak.T[None, :, :]).sum(axis=1) \
            + (s[:, :, None] * self.bk.T[None, :, :]).sum(axis=1)

        return dq.squeeze() if len(t) == 1 else dq

    def compute_ddq(self, t):
        """
        计算加速度 ddq(t)

        ddq(t) = Σ_{k=1}^N [k·ω_f·(-a_k·sin(k·ω_f·t) + b_k·cos(k·ω_f·t))]

        Args:
            t: 时间（标量或数组）

        Returns:
            ddq: (N, 7) 或 (7,) 加速度
        """
        t = np.atleast_1d(t)
        arg = np.outer(t, self.kw)  # (N, 5)
        s = np.sin(arg)  # (N, 5)
        c = np.cos(arg)  # (N, 5)

        # Scale by k·ω_f
        ddq = (c[:, :, None] * (self.bk.T * self.kw[:, None])[None, :, :]).sum(axis=1) \
            - (s[:, :, None] * (self.ak.T * self.kw[:, None])[None, :, :]).sum(axis=1)

        return ddq.squeeze() if len(t) == 1 else ddq


# ============================================================
# MuJoCo 模型构建（从 MDH + 辨识参数）
# ============================================================

def build_mujoco_xml(phi_params, friction_params, output_path):
    """
    从 Modified DH 参数和辨识参数构建 MuJoCo XML

    Args:
        phi_params: (70,) 刚体动力学参数 [m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz] × 7
        friction_params: (14,) 摩擦参数 [mu_c, mu_v] × 7
        output_path: 输出 XML 文件路径
    """
    # FR3 Classic Modified DH 参数 (d3=0.316)
    # [alpha, a, d, theta_offset]
    mdh = [
        [0, 0, 0.333, 0],
        [-np.pi/2, 0, 0, 0],
        [np.pi/2, 0, 0.316, 0],
        [np.pi/2, 0.0825, 0, 0],
        [-np.pi/2, -0.0825, 0.384, 0],
        [np.pi/2, 0, 0, 0],
        [np.pi/2, 0.088, 0, 0]
    ]

    # 解析参数（每个连杆 10 个参数）
    # φ 的惯量约定（与辨识回归矩阵一致）: 绕连杆坐标系原点(关节坐标系)的惯量 I^O
    # MuJoCo fullinertia 需要绕质心的惯量: I_com = I^O - m·(cᵀc·E₃ - c·cᵀ)
    inertial_params = []
    for i in range(7):
        idx = i * 10
        m = phi_params[idx]
        mc = phi_params[idx+1:idx+4]  # m·r_com
        com = mc / m if m > 1e-6 else np.zeros(3)

        Ixx = phi_params[idx+4]
        Ixy = phi_params[idx+5]
        Iyy = phi_params[idx+6]
        Ixz = phi_params[idx+7]
        Iyz = phi_params[idx+8]
        Izz = phi_params[idx+9]

        I_O = np.array([[Ixx, Ixy, Ixz],
                        [Ixy, Iyy, Iyz],
                        [Ixz, Iyz, Izz]])

        # Steiner 变换: I_com = I^O - m·(c^T·c·I₃ - c·c^T)
        S = np.dot(com, com) * np.eye(3) - np.outer(com, com)
        I_com = I_O - m * S

        # 问题：辨识参数中 CoM 离关节原点很远时，Steiner 校正后 I_com 特征值可能极小甚至负
        # 对策：钳位特征值，但使用更精细的阈值策略
        ev_com, V_com = np.linalg.eigh(I_com)

        # 自适应阈值：link7 最小(1e-3)，其他根据物理合理性(1e-6)
        # 1e-6 比原来的 1e-5 更宽松，减少对辨识参数的扭曲
        if i == 6:  # link7
            threshold = 1e-3  # 需要高阈值保证控制器稳定
        else:
            threshold = 1e-6  # 其他连杆：接受更小的惯量，减少人为修改

        ev_com = np.maximum(ev_com, threshold)
        I_com = V_com @ np.diag(ev_com) @ V_com.T

        inertial_params.append({
            'mass': m,
            'com': com,
            'inertia_full': [I_com[0, 0], I_com[1, 1], I_com[2, 2],
                             I_com[0, 1], I_com[0, 2], I_com[1, 2]]  # MuJoCo 格式
        })

    # 构建 XML
    xml_lines = [
        '<mujoco model="franka_fr3">',
        '  <compiler angle="radian" meshdir="." autolimits="true" balanceinertia="true"/>',
        '  ',
        '  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit" impratio="10">',
        '    <flag warmstart="enable"/>',
        '  </option>',
        '  ',
        '  <default>',
        '    <joint damping="0" frictionloss="0"/>',  # 摩擦由控制器处理
        '    <geom contype="0" conaffinity="0" group="1" rgba="0.8 0.8 0.8 1"/>',
        '  </default>',
        '  ',
        '  <worldbody>',
        '    <light pos="0 0 3" dir="0 0 -1"/>',
        '    <!-- Base fixed to world -->',
        '    <body name="base" pos="0 0 0">',
        '      <geom type="cylinder" size="0.06 0.02" rgba="0.5 0.5 0.5 1"/>',
        '      <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>',
    ]

    # 逐连杆构建（根据 MDH 参数）
    # MDH 变换 (与辨识所用 Pinocchio 模型一致): T = RotX(α)·TransX(a)·TransZ(d)·RotZ(θ)
    # MuJoCo 合成规则 (经验验证): child = parent·Trans(bpos)·Quat(bquat)·RotZ(θ)
    #   (joint pos=0, axis=Z; bpos/bquat 在关节旋转之前施加)
    # 取 bquat = RotX(α), bpos = RotX(α)·[a,0,d] = [a, -d·sinα, d·cosα]
    # 则 child = RotX(α)·TransX(a)·TransZ(d)·RotZ(θ)，与 Pinocchio 连杆系一致
    for i in range(7):
        alpha, a, d, theta_off = mdh[i]
        params = inertial_params[i]

        ca, sa = np.cos(alpha), np.sin(alpha)
        body_pos = f"{a:.6f} {-d * sa:.6f} {d * ca:.6f}"

        if alpha != 0:
            quat = axis_angle_to_quat([1, 0, 0], alpha)
            body_quat = f'quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"'
        else:
            body_quat = ""

        # body 先开，关节定义在 body 内部（MuJoCo 中 joint 属于其所在 body，
        # 连接该 body 与父 body）
        xml_lines.append(f'      <body name="link{i+1}" pos="{body_pos}" {body_quat}>')

        # 关节定义（阻尼/armature 由控制器处理，保持与辨识模型一致）
        xml_lines.append(f'        <joint name="joint{i+1}" type="hinge" pos="0 0 0" axis="0 0 1" '
                        f'range="-2.8973 2.8973" damping="0" armature="0" limited="true"/>')

        # 惯性属性（在 body 坐标系中）
        mass = params['mass']
        com = params['com']
        inertia = params['inertia_full']

        # fullinertia 用科学计数法避免小特征值(如 link7 ≈ 1e-7)被 6 位小数舍入为 0
        xml_lines.append(f'        <inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" '
                        f'mass="{mass:.6f}" '
                        f'fullinertia="{inertia[0]:.8e} {inertia[1]:.8e} {inertia[2]:.8e} '
                        f'{inertia[3]:.8e} {inertia[4]:.8e} {inertia[5]:.8e}"/>')

        # 简单可视化几何（圆柱体）
        xml_lines.append(f'        <geom type="cylinder" size="0.03 0.05" rgba="0.7 0.7 0.7 1"/>')

    # 闭合所有 body
    for _ in range(7):
        xml_lines.append('      </body>')

    xml_lines.append('    </body>')  # Close base body
    xml_lines.append('  </worldbody>')

    # 执行器（力矩控制）
    xml_lines.append('  ')
    xml_lines.append('  <actuator>')
    for i in range(7):
        xml_lines.append(f'    <motor name="motor{i+1}" joint="joint{i+1}" gear="1" ctrllimited="false"/>')
    xml_lines.append('  </actuator>')

    xml_lines.append('</mujoco>')

    # 写入文件
    with open(output_path, 'w') as f:
        f.write('\n'.join(xml_lines))

    print(f"Generated MuJoCo model: {output_path}")


def axis_angle_to_quat(axis, angle):
    """轴角转四元数 [w, x, y, z]"""
    axis = np.array(axis) / np.linalg.norm(axis)
    half_angle = angle / 2
    w = np.cos(half_angle)
    xyz = axis * np.sin(half_angle)
    return [w, xyz[0], xyz[1], xyz[2]]


# ============================================================
# FCI 风格的阻抗控制器（速度前馈）
# ============================================================

class FCIStyleController:
    """
    FCI 风格控制器：速度前馈 + 内部位置跟踪

    控制律：
    1. 接收速度命令 v_cmd = dq_d(t)
    2. 内部位置积分：q_d_int += v_cmd · dt
    3. 阻抗控制：τ = K·(q_d_int - q) + D·(v_cmd - dq) + G(q) + friction_hat
    """

    def __init__(self, model, friction_params, K_stiffness=None, D_damping=None):
        """
        Args:
            model: MuJoCo 模型
            friction_params: (14,) [mu_c, mu_v] × 7
            K_stiffness: 关节刚度（默认使用降低的稳定配置）
            D_damping: 关节阻尼（默认使用降低的稳定配置）
        """
        self.model = model
        self.data = mujoco.MjData(model)

        # 控制参数：经 tune_kd_scan.py + 震荡诊断确定（7 条轨迹 + 2 模型验证）
        # 1. D 上限受限：J7 惯量钳位 1e-3 → 理论边界 2·M/dt=2.0，但 J7 的 K=360 使
        #    wn·dt=0.6，有效阻尼边界更低（D7=1.5 在轨迹 200 上失稳，D7=1.0 全稳定）
        # 2. 摩擦补偿是速度正反馈：斜率 mu_c/tanh_w 抵消 D，tanh 宽度 0.4 保证
        #    J7 斜率 0.74 < D7=1.0（净阻尼 0.26 裕度）
        # 3. 震荡消除：速度高频残差从 ~0.10 降到 <0.003 rad/s（J7 曾最严重）
        # Kx60: RMSE 0.053 → ~0.001 rad（提升约 50 倍，全部轨迹稳定）
        self.K = np.array([1500.0, 1500.0, 1500.0, 1500.0, 900.0, 720.0, 360.0])
        self.D = np.array([3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.0])
        self.tanh_smooth_width = 0.4

        if K_stiffness is not None:
            self.K = K_stiffness if isinstance(K_stiffness, np.ndarray) else np.array([K_stiffness]*7)
        if D_damping is not None:
            self.D = D_damping if isinstance(D_damping, np.ndarray) else np.array([D_damping]*7)

        # 摩擦参数
        self.friction_params = friction_params.reshape(7, 2)  # (7, 2): [mu_c, mu_v]

        print(f"Initialized FCI-style controller")
        print(f"  K (stiffness): {self.K} N·m/rad")
        print(f"  D (damping): {self.D} N·m·s/rad")

    def reset(self, q_init):
        """重置控制器状态"""
        self.data.qpos[:7] = q_init
        self.data.qvel[:7] = 0
        mujoco.mj_forward(self.model, self.data)

    def compute_friction_compensation(self, dq):
        """
        计算摩擦补偿：mu_c·tanh(dq/w) + mu_v·dq

        w 是平滑宽度（0.4 rad/s）。宽度太小（如 0.05）时摩擦补偿对速度是强
        正反馈（斜率 mu_c/w），会抵消 D 阻尼并激励小惯量关节（J7）的高频震荡。
        """
        mu_c = self.friction_params[:, 0]
        mu_v = self.friction_params[:, 1]
        tau_friction = mu_c * np.tanh(dq / self.tanh_smooth_width) + mu_v * dq
        return tau_friction

    def control_step(self, q_d, v_cmd, dt):
        """
        执行一步控制（速度前馈）

        Args:
            q_d: (7,) 期望位置 q_d(t) - 直接使用而不是积分
            v_cmd: (7,) 速度命令 dq_d(t)
            dt: 时间步长

        Returns:
            tau: (7,) 控制力矩
        """
        # 1. 读取当前状态
        q = self.data.qpos[:7].copy()
        dq = self.data.qvel[:7].copy()

        # 2. 位置和速度误差（仿真无噪声，无需滤波）
        q_error = q_d - q
        dq_error = v_cmd - dq

        # 3. PD 控制项
        tau_pd = self.K * q_error + self.D * dq_error

        # 4. 重力补偿（使用 MuJoCo 的模型）
        self.data.qpos[:7] = q
        self.data.qvel[:7] = dq
        mujoco.mj_forward(self.model, self.data)
        tau_gravity = self.data.qfrc_bias[:7].copy()

        # 5. 摩擦补偿
        tau_friction = self.compute_friction_compensation(dq)

        # 6. 总控制力矩
        tau = tau_pd + tau_gravity + tau_friction

        return tau


# ============================================================
# 仿真主函数
# ============================================================

def run_ffs_simulation(trajectory_json, phi_nominal, phi_identified, friction_params,
                       exp_csv=None, dt=0.001, save_rate=1000.0):
    """
    运行 FFS 轨迹仿真

    Args:
        trajectory_json: FFS 轨迹 JSON 文件路径
        phi_nominal: (70,) 名义参数
        phi_identified: (70,) 辨识参数
        friction_params: (14,) 摩擦参数（名义或辨识）
        exp_csv: 真机实验 CSV 文件路径（可选，用于对比）
        dt: 仿真步长（秒）
        save_rate: 数据保存频率（Hz）

    Returns:
        results: dict with 'nominal' and 'identified' simulation data
    """
    print("="*60)
    print("FFS Trajectory MuJoCo Simulation")
    print("="*60)

    # 1. 加载 FFS 轨迹
    print("\n[1/5] Loading FFS trajectory...")
    ffs = FFSTrajectory(trajectory_json)

    # 2. 构建 MuJoCo 模型
    print("\n[2/5] Building MuJoCo models...")
    models_dir = Path(trajectory_json).parent.parent / 'src/franka_identification_controllers/mujoco_validation/models'
    models_dir.mkdir(parents=True, exist_ok=True)

    model_nominal_path = models_dir / 'fr3_ffs_nominal.xml'
    model_identified_path = models_dir / 'fr3_ffs_identified.xml'

    build_mujoco_xml(phi_nominal, friction_params, model_nominal_path)
    build_mujoco_xml(phi_identified, friction_params, model_identified_path)

    # 3. 仿真（名义参数）
    print("\n[3/5] Simulating with NOMINAL parameters...")
    model_nominal = mujoco.MjModel.from_xml_path(str(model_nominal_path))
    results_nominal = simulate_trajectory(model_nominal, ffs, friction_params, dt, save_rate)

    # 4. 仿真（辨识参数）
    print("\n[4/5] Simulating with IDENTIFIED parameters...")
    model_identified = mujoco.MjModel.from_xml_path(str(model_identified_path))
    results_identified = simulate_trajectory(model_identified, ffs, friction_params, dt, save_rate)

    # 5. 保存结果
    print("\n[5/5] Saving results...")
    output_dir = models_dir.parent / 'results' / 'ffs_simulation'
    output_dir.mkdir(parents=True, exist_ok=True)

    np.savez(output_dir / 'simulation_nominal.npz', **results_nominal)
    np.savez(output_dir / 'simulation_identified.npz', **results_identified)
    print(f"  Saved to {output_dir}")

    print("\n" + "="*60)
    print("Simulation complete!")
    print("="*60)

    return {
        'nominal': results_nominal,
        'identified': results_identified,
        'ffs': ffs
    }


def simulate_trajectory(model, ffs, friction_params, dt, save_rate):
    """
    仿真单个模型的轨迹跟踪

    Args:
        model: MuJoCo 模型
        ffs: FFSTrajectory 对象
        friction_params: (14,) 摩擦参数
        dt: 仿真步长
        save_rate: 保存频率（Hz）

    Returns:
        results: dict with 'time', 'q', 'dq', 'tau', 'q_d', 'dq_d'
    """
    # 创建控制器（使用稳定的降低增益）
    controller = FCIStyleController(
        model,
        friction_params
    )

    # 仿真参数
    duration = ffs.duration
    n_steps = int(duration / dt)
    save_every = max(1, int(1.0 / (save_rate * dt)))

    save_indices = np.arange(0, n_steps, save_every)
    n_save = len(save_indices)

    # 初始化数据记录
    time_log = np.zeros(n_save)
    q_log = np.zeros((n_save, 7))
    dq_log = np.zeros((n_save, 7))
    tau_log = np.zeros((n_save, 7))
    q_d_log = np.zeros((n_save, 7))
    dq_d_log = np.zeros((n_save, 7))

    # 重置到初始状态
    controller.reset(ffs.q0)

    save_idx = 0

    print(f"  Simulating {duration:.2f} s ({n_steps} steps, dt={dt*1000:.2f} ms)...")

    for step in range(n_steps):
        t = step * dt

        # 从 FFS 函数计算期望轨迹
        q_d = ffs.compute_q(t)
        dq_d = ffs.compute_dq(t)

        # PD 控制（位置+速度反馈）
        tau = controller.control_step(q_d, dq_d, dt)

        # 应用控制力矩
        controller.data.ctrl[:7] = tau

        # 仿真一步
        mujoco.mj_step(model, controller.data)

        # 保存数据
        if step in save_indices:
            time_log[save_idx] = t
            q_log[save_idx] = controller.data.qpos[:7].copy()
            dq_log[save_idx] = controller.data.qvel[:7].copy()
            tau_log[save_idx] = tau
            q_d_log[save_idx] = q_d
            dq_d_log[save_idx] = dq_d
            save_idx += 1

        # 进度显示
        if step % (n_steps // 10) == 0:
            print(f"    Progress: {100*step/n_steps:.0f}%")

    print("    Done!")

    return {
        'time': time_log,
        'q': q_log,
        'dq': dq_log,
        'tau': tau_log,
        'q_d': q_d_log,
        'dq_d': dq_d_log
    }


# ============================================================
# 主函数
# ============================================================

def resolve_trajectory(ws_path, traj_arg):
    """
    解析轨迹参数为 JSON 路径

    支持三种形式：
      1. 绝对/相对路径（存在则直接使用）
      2. test 目录下的文件名（如 trajectory_005_20260515_230214.json）
      3. 轨迹编号（如 5，自动匹配 trajectory_005_*.json，多个取最新）
    """
    test_dir = ws_path / 'ffs_data/test'

    p = Path(traj_arg)
    if p.exists():
        return p

    cand = test_dir / traj_arg
    if cand.exists():
        return cand

    if traj_arg.isdigit():
        matches = sorted(test_dir.glob(f'trajectory_{int(traj_arg):03d}_*.json'))
        if matches:
            return matches[-1]  # 文件名含时间戳，按名称排序即最新

    raise FileNotFoundError(f"Trajectory not found: {traj_arg} "
                            f"(searched path and {test_dir})")


def main():
    """主函数：以 trajectory 391 为例"""
    import argparse
    parser = argparse.ArgumentParser(description='FFS 轨迹 MuJoCo 仿真（与真机控制一致）')
    parser.add_argument('--trajectory', '-t', type=str,
                        default='trajectory_391_20260515_230214.json',
                        help='轨迹 JSON 路径，或 test 目录下文件名，或轨迹编号（如 5）')
    args = parser.parse_args()

    ws_path = Path.home() / 'franka_ros2_ws'

    # 1. 加载辨识参数
    params_file = ws_path / 'src/franka_identification_controllers/mujoco_validation/pc_irls_improved_params_simple.npz'

    if not params_file.exists():
        print(f"Error: Identified parameters not found: {params_file}")
        print("Please run pc_irls_improved.py first")
        return

    params = np.load(params_file)
    phi_nominal = params['phi_prior']  # (84,)
    phi_identified = params['phi_identified'].squeeze()  # (84,) - remove singleton dimension

    # 分离刚体参数和摩擦参数
    # phi 包含 84 个参数：70 个刚体 + 14 个摩擦
    phi_rigid_nominal = phi_nominal[:70]
    phi_rigid_identified = phi_identified[:70]
    friction_nominal = phi_nominal[70:84]
    friction_identified = phi_identified[70:84]

    print(f"Loaded identification results from {params_file.name}")
    print(f"  phi_nominal: {phi_nominal.shape}")
    print(f"  phi_identified: {phi_identified.shape}")
    print(f"  Rigid params (nominal): {phi_rigid_nominal.shape}")
    print(f"  Rigid params (identified): {phi_rigid_identified.shape}")
    print(f"  Friction params (identified): {friction_identified.shape}")

    # 2. 指定 FFS 轨迹（默认 trajectory 391，可用 --trajectory 指定）
    try:
        trajectory_json = resolve_trajectory(ws_path, args.trajectory)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return

    # 3. 运行仿真
    results = run_ffs_simulation(
        trajectory_json=trajectory_json,
        phi_nominal=phi_rigid_nominal,
        phi_identified=phi_rigid_identified,
        friction_params=friction_identified,
        dt=0.001,
        save_rate=1000.0
    )

    # 4. 绘制结果
    print("\nPlotting results...")
    plot_simulation_results(results, trajectory_json)

    print("\nNext: check plots in mujoco_validation/results/ffs_simulation/")


def plot_simulation_results(results, trajectory_json):
    """绘制仿真结果"""
    nominal = results['nominal']
    identified = results['identified']
    ffs = results['ffs']

    # 加载真机数据（如果存在）
    csv_path = Path(str(trajectory_json).replace('.json', '.csv'))
    exp_data = None
    if csv_path.exists():
        import pandas as pd
        df = pd.read_csv(csv_path)

        # 构建时间戳：从 ros_time_ns 转换为相对秒数
        t0 = df['ros_time_ns'].values[0]
        exp_data = {
            'time': (df['ros_time_ns'].values - t0) * 1e-9,
            'q': df[[f'q_{i}' for i in range(7)]].values,
            'dq': df[[f'dq_{i}' for i in range(7)]].values,
            'tau': df[[f'tau_J_{i}' for i in range(7)]].values,
            'q_d': df[[f'q_d_{i}' for i in range(7)]].values,
        }

    # 绘图
    fig, axes = plt.subplots(7, 4, figsize=(20, 20))

    for i in range(7):
        # 位置
        axes[i, 0].plot(nominal['time'], nominal['q_d'][:, i], 'k--', label='Desired', linewidth=1)
        if exp_data is not None:
            axes[i, 0].plot(exp_data['time'], exp_data['q'][:, i], 'gray', label='Real', alpha=0.7, linewidth=0.8)
        axes[i, 0].plot(nominal['time'], nominal['q'][:, i], 'b-', label='Nominal', linewidth=1)
        axes[i, 0].plot(identified['time'], identified['q'][:, i], 'r-', label='Identified', linewidth=1)
        axes[i, 0].set_ylabel(f'Joint {i+1} q [rad]')
        axes[i, 0].grid(True, alpha=0.3)
        if i == 0:
            axes[i, 0].legend(loc='upper right')

        # 速度
        axes[i, 1].plot(nominal['time'], nominal['dq_d'][:, i], 'k--', linewidth=1)
        if exp_data is not None:
            axes[i, 1].plot(exp_data['time'], exp_data['dq'][:, i], 'gray', alpha=0.7, linewidth=0.8)
        axes[i, 1].plot(nominal['time'], nominal['dq'][:, i], 'b-', linewidth=1)
        axes[i, 1].plot(identified['time'], identified['dq'][:, i], 'r-', linewidth=1)
        axes[i, 1].set_ylabel(f'Joint {i+1} dq [rad/s]')
        axes[i, 1].grid(True, alpha=0.3)

        # 力矩
        if exp_data is not None:
            axes[i, 2].plot(exp_data['time'], exp_data['tau'][:, i], 'gray', alpha=0.7, linewidth=0.8)
        axes[i, 2].plot(nominal['time'], nominal['tau'][:, i], 'b-', linewidth=1)
        axes[i, 2].plot(identified['time'], identified['tau'][:, i], 'r-', linewidth=1)
        axes[i, 2].set_ylabel(f'Joint {i+1} τ [Nm]')
        axes[i, 2].grid(True, alpha=0.3)

        # 跟踪误差
        if exp_data is not None:
            exp_err = exp_data['q'][:, i] - exp_data['q_d'][:, i]
            axes[i, 3].plot(exp_data['time'], exp_err, 'gray', label='Real', alpha=0.7, linewidth=0.8)
        nom_err = nominal['q'][:, i] - nominal['q_d'][:, i]
        id_err = identified['q'][:, i] - identified['q_d'][:, i]
        axes[i, 3].plot(nominal['time'], nom_err, 'b-', label='Nominal', linewidth=1)
        axes[i, 3].plot(identified['time'], id_err, 'r-', label='Identified', linewidth=1)
        axes[i, 3].axhline(0, color='k', linewidth=0.5, linestyle=':')
        axes[i, 3].set_ylabel(f'Joint {i+1} err [rad]')
        axes[i, 3].grid(True, alpha=0.3)
        if i == 0:
            axes[i, 3].legend(loc='upper right')

    axes[6, 0].set_xlabel('Time [s]')
    axes[6, 1].set_xlabel('Time [s]')
    axes[6, 2].set_xlabel('Time [s]')
    axes[6, 3].set_xlabel('Time [s]')

    plt.tight_layout()

    output_dir = Path(trajectory_json).parent.parent / 'src/franka_identification_controllers/mujoco_validation/results/ffs_simulation'
    output_dir.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_dir / 'ffs_simulation_comparison.png', dpi=150)
    plt.close(fig)
    print(f"  Saved plot: {output_dir / 'ffs_simulation_comparison.png'}")

    # 误差放大图（带 RMSE 标注）
    fig_zoom, axes_zoom = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        nom_err = nominal['q'][:, i] - nominal['q_d'][:, i]
        id_err = identified['q'][:, i] - identified['q_d'][:, i]

        if exp_data is not None:
            exp_err = exp_data['q'][:, i] - exp_data['q_d'][:, i]
            axes_zoom[i].plot(exp_data['time'], exp_err, 'gray', alpha=0.7, linewidth=0.8, label='Real')
            txt = (f"RMSE Real: {np.sqrt(np.mean(exp_err**2)):.6f} rad\n"
                   f"RMSE Nom:  {np.sqrt(np.mean(nom_err**2)):.6f} rad\n"
                   f"RMSE Id:   {np.sqrt(np.mean(id_err**2)):.6f} rad")
        else:
            txt = (f"RMSE Nom:  {np.sqrt(np.mean(nom_err**2)):.6f} rad\n"
                   f"RMSE Id:   {np.sqrt(np.mean(id_err**2)):.6f} rad")
        axes_zoom[i].plot(nominal['time'], nom_err, 'b-', linewidth=1, label='Nominal')
        axes_zoom[i].plot(identified['time'], id_err, 'r-', linewidth=1, label='Identified')
        axes_zoom[i].axhline(0, color='k', linewidth=0.5, linestyle=':')
        axes_zoom[i].set_ylabel(f'Joint {i+1} err [rad]')
        axes_zoom[i].set_ylim([-0.01, 0.01])
        axes_zoom[i].grid(True, alpha=0.3)
        axes_zoom[i].text(0.02, 0.97, txt, transform=axes_zoom[i].transAxes, fontsize=9,
                          verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='0.6'))
    axes_zoom[0].legend(loc='upper right')
    axes_zoom[6].set_xlabel('Time [s]')
    plt.tight_layout()
    plt.savefig(output_dir / 'error_zoom.png', dpi=100)
    plt.close(fig_zoom)
    print(f"  Saved plot: {output_dir / 'error_zoom.png'}")

    # 计算 RMSE
    print("\n" + "="*60)
    print("Tracking Error (RMSE per joint)")
    print("="*60)

    rmse_nom_q = np.sqrt(np.mean((nominal['q'] - nominal['q_d'])**2, axis=0))
    rmse_id_q = np.sqrt(np.mean((identified['q'] - identified['q_d'])**2, axis=0))

    print("Position RMSE [rad]:")
    print("  Nominal:    ", np.round(rmse_nom_q, 5))
    print("  Identified: ", np.round(rmse_id_q, 5))
    print("  Mean - Nominal:    ", round(float(np.mean(rmse_nom_q)), 5))
    print("  Mean - Identified: ", round(float(np.mean(rmse_id_q)), 5))

    if exp_data is not None:
        # 插值对齐
        from scipy.interpolate import interp1d

        exp_q_interp = interp1d(exp_data['time'], exp_data['q'], axis=0, fill_value='extrapolate')
        exp_q_aligned = exp_q_interp(nominal['time'])

        rmse_exp_nom = np.sqrt(np.mean((nominal['q'] - exp_q_aligned)**2, axis=0))
        rmse_exp_id = np.sqrt(np.mean((identified['q'] - exp_q_aligned)**2, axis=0))

        print("\nComparison with Real Robot [rad]:")
        print("  Nominal:    ", np.round(rmse_exp_nom, 5))
        print("  Identified: ", np.round(rmse_exp_id, 5))
        print("  Mean - Nominal:    ", round(float(np.mean(rmse_exp_nom)), 5))
        print("  Mean - Identified: ", round(float(np.mean(rmse_exp_id)), 5))


if __name__ == '__main__':
    main()
