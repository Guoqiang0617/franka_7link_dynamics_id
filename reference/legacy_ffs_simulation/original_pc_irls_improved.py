"""
pc_irls_improved.py — 改进的物理一致性 IRLS 动力学参数辨识

相比 test_lstsq_full_dynamics.py 的改进:
  1. 完整物理一致性: 4×4 伪惯量 LMI 约束 (J_i ≻ 0) via cvxpy SDP
  2. CAD 先验正则化: Tikhonov + 零空间自适应权重
  3. Stribeck 摩擦模型: 固定形状参数,4 个线性幅值参数
  4. 基参数分析: QR/SVD 秩分析 + 每参数不确定度报告

参数结构 (每关节 14 个参数):
  - 刚体惯性 [0:10]: m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz
  - Stribeck 摩擦 [10:14]: mu_c, mu_s, mu_v, mu_b
"""

import os
import sys
import csv
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from scipy.optimize import minimize

# ── 中文字体配置 ──────────────────────────────────────────────────────────
def _setup_chinese_font():
    """尝试找到系统中文字体并设置，避免方框乱码"""
    candidates = [
        'Microsoft YaHei', 'SimHei', 'SimSun', 'FangSong', 'KaiTi',
        'Noto Sans CJK SC', 'WenQuanYi Micro Hei',
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for font in candidates:
        if font in available:
            plt.rcParams['font.family'] = font
            plt.rcParams['axes.unicode_minus'] = False
            return font
    # 找不到候选字体时查找系统字体文件
    for fp in fm.findSystemFonts(fontpaths=None, fontext='ttf'):
        if any(kw in fp.lower() for kw in ['yahei', 'simhei', 'msyh', 'simsun']):
            prop = fm.FontProperties(fname=fp)
            plt.rcParams['font.family'] = prop.get_name()
            plt.rcParams['axes.unicode_minus'] = False
            return fp
    print("  警告: 未找到中文字体，图表中文可能显示为方框")
    return None

_CHINESE_FONT = _setup_chinese_font()

# 尝试导入 cvxpy (用于 SDP),若不可用则回退到投影法
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
    print("✓ cvxpy 可用,将使用 SDP 求解严格 LMI 约束")
except ImportError:
    CVXPY_AVAILABLE = False
    print("✗ cvxpy 不可用,将使用 PSD 投影近似法")

try:
    import pinocchio as pin
except ImportError:
    print("错误: 找不到 pinocchio 库。请在 pytorch_gpu01 环境中运行。")
    sys.exit(1)

# =====================================================================
# 用户配置区
# =====================================================================
CONFIG = {
    # 数据集路径
    'TRAIN_DATA_PATH': r'G:\gqWork\博士课题\动力学建模\Dynamics_model\LagDNN\data\franka_fr3\train.npz',
    'TEST_DATA_PATH': r'G:\gqWork\博士课题\动力学建模\Dynamics_model\LagDNN\data\franka_fr3\test.npz',

    # 配置文件路径 (CAD 先验)
    'CONFIG_YAML': r'G:\gqWork\博士课题\动力学建模\Dynamics_model\LagDNN\config\real_franka_fr3_7dof.yaml',

    # 截取样本数
    'NUM_TRAIN_SAMPLES': 416637,
    'NUM_TEST_SAMPLES': 102842,

    # 画图长度限制
    'PLOT_MAX_LEN': 10000,

    # IRLS 参数
    'IRLS_MAX_ITER': 15,
    'IRLS_CONVERGENCE_TOL': 1e-4,
    'TUKEY_CONSTANT': 4.685,  # Tukey biweight 常数

    # 正则化参数
    'LAMBDA_DATA': 1e-4,      # 可辨识方向的正则化权重
    'LAMBDA_NULL': 1.0,       # 零空间方向的正则化权重 (强制向 CAD 先验)
    'SVD_THRESHOLD': 1e-5,    # 零空间判定阈值 (相对最大奇异值)

    # 物理约束参数
    'MIN_MASS': 1e-6,         # 最小质量 (避免数值问题)
    'MIN_EIGENVALUE': 1e-6,   # 伪惯量最小特征值

    # Stribeck 摩擦参数
    'SMOOTH_SIGN_EPSILON': 0.05,  # tanh 光滑化参数 (rad/s)

    # 摩擦模型选择:
    #   'stribeck' — 4参数/关节 [mu_c, mu_s, mu_v, mu_b]，读取 friction_parameters
    #   'simple'   — 2参数/关节 [mu_c, mu_v]，          读取 friction_simple
    'FRICTION_MODEL': 'simple',

    # 机器人自由度
    'N_DOF': 7,
}
# =====================================================================


def build_pinocchio_model_from_mdh(mdh_params, gravity_vector):
    """根据 MDH 参数动态构建 Pinocchio 模型。"""
    model = pin.Model()
    model.gravity.linear = np.array(gravity_vector)

    parent_id = 0
    for i, (alpha, a, d, theta_offset) in enumerate(mdh_params):
        rot_x = pin.utils.rotate('x', alpha)
        trans_x = np.array([a, 0, 0])
        placement_x = pin.SE3(rot_x, trans_x)

        rot_z = pin.utils.rotate('z', theta_offset)
        trans_z = np.array([0, 0, d])
        placement_z = pin.SE3(rot_z, trans_z)

        placement = placement_x * placement_z
        joint_name = f"joint_{i+1}"
        joint_id = model.addJoint(parent_id, pin.JointModelRZ(), placement, joint_name)

        # 占位惯量 (辨识后不使用模型本身的惯量)
        inertia = pin.Inertia.FromSphere(1.0, 0.01)
        model.appendBodyToJoint(joint_id, inertia, pin.SE3.Identity())
        parent_id = joint_id

    return model


def load_and_preprocess_data(file_path, max_samples):
    """加载数据,截取长度"""
    if not file_path or not os.path.exists(file_path):
        return None

    data = np.load(file_path)
    total_samples = data['q'].shape[0]
    n_samples = min(max_samples, total_samples)

    q = data['q'][:n_samples]
    qd = data['qd'][:n_samples]
    qdd = data['qdd'][:n_samples]
    tau = data['tau'][:n_samples].copy()

    return q, qd, qdd, tau, n_samples


def get_param_labels_stribeck(n_dof):
    """返回 Stribeck 模型参数标签 (14 参数/关节)"""
    labels = []
    for i in range(n_dof):
        labels.extend([
            f'J{i}_m', f'J{i}_mcx', f'J{i}_mcy', f'J{i}_mcz',
            f'J{i}_Ixx', f'J{i}_Ixy', f'J{i}_Iyy', f'J{i}_Ixz', f'J{i}_Iyz', f'J{i}_Izz',
            f'J{i}_mu_c',   # 库仑摩擦幅值
            f'J{i}_mu_s',   # Stribeck 摩擦幅值
            f'J{i}_mu_v',   # 粘性摩擦系数
            f'J{i}_mu_b',   # 高速粘性 (三次项)
        ])
    return labels


def get_param_labels_simple(n_dof):
    """返回简单摩擦模型参数标签 (12 参数/关节)"""
    labels = []
    for i in range(n_dof):
        labels.extend([
            f'J{i}_m', f'J{i}_mcx', f'J{i}_mcy', f'J{i}_mcz',
            f'J{i}_Ixx', f'J{i}_Ixy', f'J{i}_Iyy', f'J{i}_Ixz', f'J{i}_Iyz', f'J{i}_Izz',
            f'J{i}_mu_c',   # 库仑/静摩擦
            f'J{i}_mu_v',   # 黏性摩擦 (damping)
        ])
    return labels


def get_param_labels(n_dof, friction_model='stribeck'):
    """统一入口：根据摩擦模型返回参数标签列表"""
    if friction_model == 'simple':
        return get_param_labels_simple(n_dof)
    return get_param_labels_stribeck(n_dof)


# =====================================================================
# CAD 先验加载
# =====================================================================

def load_cad_prior(yaml_path, n_dof, friction_model='stribeck'):
    """
    从 YAML 加载 CAD 先验，构建 phi_prior 向量

    friction_model='stribeck': 每关节14参数，摩擦 [mu_c, mu_s, mu_v, mu_b]
                                读取 friction_parameters，返回 (vs_arr, delta_arr)
    friction_model='simple'  : 每关节12参数，摩擦 [mu_c, mu_v]
                                读取 friction_simple，vs_arr=delta_arr=None
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    masses        = cfg['link_masses']
    coms          = cfg['link_com_positions']
    inertias_yaml = cfg['link_inertias']   # [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]

    n_fric = 2 if friction_model == 'simple' else 4
    phi_rigid_prior = np.zeros(10 * n_dof)
    phi_frict_prior = np.zeros(n_fric * n_dof)
    vs_arr    = np.zeros(n_dof)
    delta_arr = np.zeros(n_dof)

    for j in range(n_dof):
        m  = masses[j]
        cx, cy, cz = coms[j]
        Ixx_y, Iyy_y, Izz_y, Ixy_y, Ixz_y, Iyz_y = inertias_yaml[j]

        # 刚体参数 (pinocchio 顺序)
        phi_rigid_prior[10*j:10*j+10] = [
            m, m*cx, m*cy, m*cz,
            Ixx_y, Ixy_y, Iyy_y, Ixz_y, Iyz_y, Izz_y
        ]

        if friction_model == 'simple':
            # [friction, damping] → [mu_c, mu_v]
            mu_c, mu_v = cfg['friction_simple'][j]
            phi_frict_prior[2*j:2*j+2] = [mu_c, mu_v]
        else:
            # [mu_c, mu_s, mu_v, v_s, delta, chi, mu_b]
            mu_c, mu_s, mu_v, v_s, delta, chi, mu_b = cfg['friction_parameters'][j]
            phi_frict_prior[4*j:4*j+4] = [mu_c, mu_s, mu_v, mu_b]
            vs_arr[j]    = v_s
            delta_arr[j] = delta

    phi_prior = np.concatenate([phi_rigid_prior, phi_frict_prior])
    print(f"  CAD 先验加载完毕 [{friction_model}]: "
          f"{phi_prior.shape[0]} 个参数 (刚体 {10*n_dof}, 摩擦 {n_fric*n_dof})")
    return phi_prior, vs_arr, delta_arr


# =====================================================================
# Stribeck 回归矩阵构建
# =====================================================================

def build_stribeck_regressor(model, data_pin, q, qd, qdd, tau,
                              n_samples, n_dof, vs_arr, delta_arr):
    """
    构建 Stribeck 摩擦的增广回归矩阵 W,形状 (n_samples*n_dof, 14*n_dof)

    列布局:
      [0 : 10*n_dof]       - pinocchio 刚体回归矩阵 Y_rigid
      [10*n_dof : 14*n_dof] - Stribeck 摩擦基函数 (每关节4列)
        col 10*n_dof + 4*j + 0: (1-S_j)*smooth_sign(q̇_j)  -> mu_c
        col 10*n_dof + 4*j + 1:    S_j *smooth_sign(q̇_j)  -> mu_s
        col 10*n_dof + 4*j + 2:    q̇_j                    -> mu_v
        col 10*n_dof + 4*j + 3:    q̇_j^3                  -> mu_b

    smooth_sign(x) = tanh(x / SMOOTH_SIGN_EPSILON)
    S_j(i) = exp(-(|q̇_j(i)| / vs_j)^delta_j)   Stribeck 包络
    """
    eps_sign = CONFIG['SMOOTH_SIGN_EPSILON']
    total_params = 14 * n_dof
    W = np.zeros((n_samples * n_dof, total_params), dtype=np.float64)
    tau_flat = tau.reshape(-1, 1)

    print(f"  组装 Stribeck 回归矩阵 ({n_samples} 样本, {total_params} 参数)...")
    print_step = max(1, n_samples // 10)

    for i in range(n_samples):
        if i > 0 and i % print_step == 0:
            pct = 100.0 * i / n_samples
            print(f"    进度: {i}/{n_samples} ({pct:.1f}%)")

        # 刚体回归列
        Y_rigid = pin.computeJointTorqueRegressor(model, data_pin, q[i], qd[i], qdd[i])
        row0 = i * n_dof
        row1 = row0 + n_dof
        W[row0:row1, :10 * n_dof] = Y_rigid

        # Stribeck 摩擦列
        for j in range(n_dof):
            qdj = float(qd[i, j])
            # 光滑符号函数
            ss = np.tanh(qdj / eps_sign)
            # Stribeck 包络 S in [0,1]: S=1 at zero speed, S=0 at high speed
            S = np.exp(-(abs(qdj) / vs_arr[j]) ** delta_arr[j]) if abs(qdj) > 1e-12 else 1.0

            col_base = 10 * n_dof + 4 * j
            W[row0 + j, col_base + 0] = (1.0 - S) * ss   # mu_c
            W[row0 + j, col_base + 1] = S * ss            # mu_s
            W[row0 + j, col_base + 2] = qdj               # mu_v
            W[row0 + j, col_base + 3] = qdj ** 3          # mu_b

    return W, tau_flat, total_params


def build_simple_regressor(model, data_pin, q, qd, qdd, tau,
                            n_samples, n_dof):
    """
    构建简单两参数摩擦模型的回归矩阵 W (n_samples*n_dof, 12*n_dof)

    列布局 (与 stribeck 版一致的块布局):
      [0 : 10*n_dof]        - pinocchio 刚体回归矩阵 Y_rigid (完整)
      [10*n_dof : 12*n_dof] - 摩擦基函数 (每关节2列)
        col 10*n_dof + 2*j + 0: tanh(q̇_j/ε)  -> mu_c (库仑摩擦)
        col 10*n_dof + 2*j + 1: q̇_j          -> mu_v (黏性摩擦)
    """
    eps_sign     = CONFIG['SMOOTH_SIGN_EPSILON']
    total_params = 12 * n_dof
    W = np.zeros((n_samples * n_dof, total_params), dtype=np.float64)
    tau_flat = tau.reshape(-1, 1)

    print(f"  组装简单摩擦回归矩阵 ({n_samples} 样本, {total_params} 参数)...")
    print_step = max(1, n_samples // 10)

    fric_start = 10 * n_dof
    for i in range(n_samples):
        if i > 0 and i % print_step == 0:
            pct = 100.0 * i / n_samples
            print(f"    进度: {i}/{n_samples} ({pct:.1f}%)")

        Y_rigid = pin.computeJointTorqueRegressor(model, data_pin, q[i], qd[i], qdd[i])
        row0 = i * n_dof
        row1 = row0 + n_dof
        W[row0:row1, :10 * n_dof] = Y_rigid   # 完整刚体块

        for j in range(n_dof):
            qdj = float(qd[i, j])
            col_base = fric_start + 2 * j
            W[row0 + j, col_base + 0] = np.tanh(qdj / eps_sign)   # mu_c
            W[row0 + j, col_base + 1] = qdj                        # mu_v

    return W, tau_flat, total_params

def phi_rigid_to_pseudo_inertia(phi_i):
    """
    将 10 维刚体参数转为 4×4 伪惯量矩阵 J

    phi_i: [m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]

    J = [[Σ,   h],     Σ = 0.5*tr(I_c)*E3 − I_c
         [hᵀ,  m]]     h = [mcx, mcy, mcz]ᵀ

    J ≻ 0  ⟺  参数物理可实现
    """
    m   = phi_i[0]
    h   = phi_i[1:4]
    Ixx, Ixy, Iyy, Ixz, Iyz, Izz = phi_i[4], phi_i[5], phi_i[6], phi_i[7], phi_i[8], phi_i[9]

    I_c = np.array([[Ixx, Ixy, Ixz],
                    [Ixy, Iyy, Iyz],
                    [Ixz, Iyz, Izz]])
    tr_half = 0.5 * np.trace(I_c)  # = 0.5*(Ixx+Iyy+Izz)
    Sigma = tr_half * np.eye(3) - I_c

    J = np.zeros((4, 4))
    J[:3, :3] = Sigma
    J[:3, 3]  = h
    J[3, :3]  = h
    J[3, 3]   = m
    return J


def pseudo_inertia_to_phi_rigid(J):
    """
    从 4×4 伪惯量矩阵 J 恢复 10 维刚体参数

    返回: [m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
    """
    m = J[3, 3]
    h = J[:3, 3].copy()
    Sigma = J[:3, :3]

    # I_c = tr(Σ)*E3 − Σ  (因为 tr(Σ) = 0.5*tr(I_c))
    I_c = np.trace(Sigma) * np.eye(3) - Sigma

    Ixx = I_c[0, 0]
    Ixy = I_c[0, 1]
    Iyy = I_c[1, 1]
    Ixz = I_c[0, 2]
    Iyz = I_c[1, 2]
    Izz = I_c[2, 2]

    return np.array([m, h[0], h[1], h[2], Ixx, Ixy, Iyy, Ixz, Iyz, Izz])


def project_phi_to_physical(phi, n_dof, min_eig=1e-6):
    """
    【Fallback】将刚体参数投影到物理可行集(纯 numpy)

    对每个连杆:
      1. 组装伪惯量 J
      2. 特征值分解,截断负特征值到 min_eig
      3. 重建并恢复参数

    摩擦参数直接下界截断 (mu_c,mu_s,mu_v >= 0)
    """
    phi_out = phi.flatten().copy()

    # 刚体参数投影
    for j in range(n_dof):
        phi_i = phi_out[10*j: 10*j+10]
        J = phi_rigid_to_pseudo_inertia(phi_i)
        # 对称化,避免数值噪声
        J = 0.5 * (J + J.T)
        eigvals, eigvecs = np.linalg.eigh(J)
        eigvals_proj = np.maximum(eigvals, min_eig)
        J_proj = eigvecs @ np.diag(eigvals_proj) @ eigvecs.T
        phi_out[10*j: 10*j+10] = pseudo_inertia_to_phi_rigid(J_proj)

    # 摩擦参数下界 >= 0  (n_fric 自动推断: 2=simple, 4=stribeck)
    fric_start = 10 * n_dof
    n_fric = (len(phi_out) - fric_start) // n_dof
    for j in range(n_dof):
        base = fric_start + n_fric*j
        phi_out[base: base+n_fric] = np.maximum(phi_out[base: base+n_fric], 0.0)

    return phi_out.reshape(-1, 1)


# =====================================================================
# SDP 求解器 (cvxpy LMI 约束)
# =====================================================================

def _build_pseudo_inertia_basis():
    """
    预计算伪惯量的 10 个 4×4 基矩阵 A_k,使得:
        J_i = sum(phi_rigid_i[k] * A_k for k in range(10))

    参数顺序: [m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
    """
    basis = []

    # A_0: m
    A = np.zeros((4, 4)); A[3, 3] = 1.0
    basis.append(A)
    # A_1: mcx
    A = np.zeros((4, 4)); A[0, 3] = A[3, 0] = 1.0
    basis.append(A)
    # A_2: mcy
    A = np.zeros((4, 4)); A[1, 3] = A[3, 1] = 1.0
    basis.append(A)
    # A_3: mcz
    A = np.zeros((4, 4)); A[2, 3] = A[3, 2] = 1.0
    basis.append(A)
    # A_4: Ixx  →  Σ[0,0]+=-0.5, Σ[1,1]+=0.5, Σ[2,2]+=0.5
    A = np.zeros((4, 4)); A[0, 0] = -0.5; A[1, 1] = 0.5; A[2, 2] = 0.5
    basis.append(A)
    # A_5: Ixy  →  Σ[0,1]=Σ[1,0]=-1
    A = np.zeros((4, 4)); A[0, 1] = A[1, 0] = -1.0
    basis.append(A)
    # A_6: Iyy  →  Σ[0,0]+=0.5, Σ[1,1]+=-0.5, Σ[2,2]+=0.5
    A = np.zeros((4, 4)); A[0, 0] = 0.5; A[1, 1] = -0.5; A[2, 2] = 0.5
    basis.append(A)
    # A_7: Ixz  →  Σ[0,2]=Σ[2,0]=-1
    A = np.zeros((4, 4)); A[0, 2] = A[2, 0] = -1.0
    basis.append(A)
    # A_8: Iyz  →  Σ[1,2]=Σ[2,1]=-1
    A = np.zeros((4, 4)); A[1, 2] = A[2, 1] = -1.0
    basis.append(A)
    # A_9: Izz  →  Σ[0,0]+=0.5, Σ[1,1]+=0.5, Σ[2,2]+=-0.5
    A = np.zeros((4, 4)); A[0, 0] = 0.5; A[1, 1] = 0.5; A[2, 2] = -0.5
    basis.append(A)

    return basis  # list of 10 numpy (4,4) arrays


# 模块级预计算,只做一次
_PSEUDO_INERTIA_BASIS = _build_pseudo_inertia_basis()


def compute_null_space_regularizer(W, n_params, phi_prior,
                                    lam_data, lam_null, svd_thresh):
    """
    计算零空间自适应正则化矩阵 R 和向量 r:
        目标函数加项: phi^T R phi - 2 r^T phi
    其中:
        R = lam_data*I + (lam_null - lam_data) * V_null @ V_null^T
        r = R @ phi_prior

    W 的零空间方向将受 lam_null 强约束 (拉向 CAD 先验),
    可辨识方向受 lam_data 弱约束。

    注意: W 较大时直接 SVD 耗时,改用 (W^T W) 的特征分解代替。
    """
    print("  计算零空间自适应正则化 (基于 W^T W 特征分解)...")
    # W^T W 的特征分解比 W 的 SVD 快 (n_params << n_samples*n_dof)
    # W^T W 的特征值 = W 奇异值的平方
    HH = W.T @ W                              # (n_params, n_params)
    eigvals, eigvecs = np.linalg.eigh(HH)    # 升序
    s_sq = eigvals
    s_max_sq = s_sq.max()
    thresh_sq = (svd_thresh * np.sqrt(s_max_sq)) ** 2

    # 零空间: 特征值低于阈值的方向
    null_mask = (s_sq < thresh_sq)
    n_null = null_mask.sum()
    n_rank = (~null_mask).sum()
    print(f"    可辨识方向 (rank): {n_rank},  零空间维度: {n_null}")

    V_null = eigvecs[:, null_mask]           # (n_params, n_null)

    # 正则化矩阵 R
    R = lam_data * np.eye(n_params)
    if n_null > 0:
        R += (lam_null - lam_data) * (V_null @ V_null.T)

    r = R @ phi_prior.flatten()

    return R, r, n_rank, n_null, V_null


def solve_sdp_lmi(H_reg, g_reg, n_dof, min_eig=1e-6):
    """
    用 cvxpy SDP 解带 LMI 约束的二次规划:

        min   0.5 * phi^T H_reg phi - g_reg^T phi
        s.t.  J_i(phi_rigid_i) ≻ min_eig * I4    for i=0..n_dof-1
              phi_friction_j >= 0                  for j=0..n_dof-1

    H_reg = W_w^T W_w + R   (已含正则化)
    g_reg = W_w^T tau_w + r (已含先验)
    """
    n_params = H_reg.shape[0]
    phi_var = cp.Variable(n_params)

    obj = 0.5 * cp.quad_form(phi_var, H_reg) - g_reg @ phi_var

    constraints = []

    # LMI: 每个连杆的伪惯量 J_i ≻ min_eig*I4
    for j in range(n_dof):
        phi_rigid_j = phi_var[10*j: 10*j+10]
        # J_j 是 phi_rigid_j 的线性函数 (用预计算基矩阵)
        J_j = sum(_PSEUDO_INERTIA_BASIS[k] * phi_rigid_j[k]
                  for k in range(10))
        constraints.append(J_j >> min_eig * np.eye(4))

    # 摩擦下界: 所有摩擦系数 >= 0  (n_fric 自动推断: 2=simple, 4=stribeck)
    fric_start = 10 * n_dof
    n_fric = (phi_var.shape[0] - fric_start) // n_dof
    for j in range(n_dof):
        base = fric_start + n_fric*j
        constraints.append(phi_var[base: base+n_fric] >= 0.0)

    prob = cp.Problem(cp.Minimize(obj), constraints)

    # 优先 CLARABEL,回退 SCS
    for solver in [cp.CLARABEL, cp.SCS]:
        try:
            prob.solve(solver=solver, verbose=False)
            if prob.status in ('optimal', 'optimal_inaccurate') and phi_var.value is not None:
                return phi_var.value.reshape(-1, 1), prob.status
        except Exception:
            continue

    # 两个求解器都失败:返回 None 触发 fallback
    return None, prob.status


# =====================================================================
# PC-IRLS 主循环
# =====================================================================

def solve_pc_irls_improved(W, tau_flat, n_dof, phi_prior,
                            vs_arr, delta_arr, config):
    """
    改进的 PC-IRLS 算法:
      - Tukey biweight 鲁棒加权 (外循环)
      - 完整物理一致性: LMI 约束 via SDP (cvxpy 可用时) or PSD 投影
      - CAD 先验正则化: 零空间自适应权重

    返回: phi_final (n_params, 1)
    """
    n_params = W.shape[1]
    max_iter = config['IRLS_MAX_ITER']
    tol = config['IRLS_CONVERGENCE_TOL']
    lam_data = config['LAMBDA_DATA']
    lam_null = config['LAMBDA_NULL']
    svd_thresh = config['SVD_THRESHOLD']
    min_eig = config['MIN_EIGENVALUE']
    tukey_c = config['TUKEY_CONSTANT']

    print("\n[PC-IRLS] 开始迭代优化")
    print(f"  最大迭代: {max_iter}, 收敛容差: {tol}")
    print(f"  正则化: λ_data={lam_data}, λ_null={lam_null}")
    print(f"  Tukey 常数: {tukey_c}, 零空间阈值: {svd_thresh}")

    # 预计算零空间正则化器 (W 结构不变,只做一次)
    R, r, n_rank, n_null, V_null = compute_null_space_regularizer(
        W, n_params, phi_prior, lam_data, lam_null, svd_thresh
    )

    # 初始解: phi 统一为列向量 (n_params,1), tau_1d 保持1D (n,)
    phi = phi_prior.copy().reshape(-1, 1)
    tau_1d = tau_flat.flatten()   # 始终1D,避免下游广播灾难

    for it in range(max_iter):
        # 1. 计算残差与 MAD — flatten 确保 residual 是1D (n,)
        residual = tau_1d - (W @ phi).flatten()
        err = np.abs(residual)          # (n,)
        mad = np.median(err)
        tune = tukey_c * mad / 0.6745 + 1e-8

        # 2. Tukey biweight 权重 — weight/weight_sqrt 均为1D (n,)
        weight = np.zeros_like(err)
        mask = (err <= tune)
        weight[mask] = (1.0 - (err[mask] / tune) ** 2) ** 2
        weight_sqrt = np.sqrt(weight)   # (n,)

        # 3. 加权系统  weight_sqrt(n,) → (n,1) 与 W(n,p) 广播
        W_w = W * weight_sqrt[:, np.newaxis]   # (n,p)
        tau_w = tau_1d * weight_sqrt           # (n,)

        # 4. 二次型矩阵 + 正则化
        H_reg = W_w.T @ W_w + R
        g_reg = W_w.T @ tau_w + r

        # 5. 求解 (优先 SDP,失败则 fallback 投影法)
        phi_new = None
        if CVXPY_AVAILABLE:
            phi_new, status = solve_sdp_lmi(H_reg, g_reg, n_dof, min_eig)
            if phi_new is None:
                print(f"  iter {it+1}: SDP 失败 ({status}),回退到投影法")

        if phi_new is None:
            # Fallback: 无约束 lstsq + PSD 投影
            phi_new = np.linalg.lstsq(H_reg, g_reg, rcond=None)[0].reshape(-1, 1)
            phi_new = project_phi_to_physical(phi_new, n_dof, min_eig)

        # 6. 收敛检查
        delta_norm = np.linalg.norm(phi_new - phi)
        phi_norm = np.linalg.norm(phi) + 1e-12
        rel_change = delta_norm / phi_norm

        # 确保维度匹配：flatten 避免 broadcasting 灾难
        rmse = np.sqrt(np.mean((tau_1d - (W @ phi_new).flatten()) ** 2))
        print(f"  iter {it+1}/{max_iter}: rel_change={rel_change:.6f}, "
              f"RMSE={rmse:.4f} Nm, MAD={mad:.4f}")

        if rel_change < tol:
            print(f"  收敛于第 {it+1} 次迭代")
            phi = phi_new
            break

        phi = phi_new

    print("[PC-IRLS] 优化完成\n")
    return phi


# =====================================================================
# 基参数分析 + 不确定度报告
# =====================================================================

def analyze_base_parameters(W, tau_flat, phi, n_dof, phi_prior,
                             param_labels, vs_arr, delta_arr):
    """
    分析可辨识性与参数不确定度:
      1. W^T W 秩分析 → n_rank 个可辨识方向, n_null 个零空间
      2. 每个参数标准差估计 (Hessian 伪逆对角元)
      3. 打印对比表，含"来源"列:
           辨识       — 数据有足够信息，IRLS 有效更新
           先验(零空间) — W 在该方向无信息，值≈CAD 先验
           CAD固定    — vs/delta 形状参数，从不进入 φ 向量

    返回: 可辨识性指标字典
    """
    print("\n" + "="*75)
    print("基参数与不确定度分析")
    print("="*75)

    n_params      = W.shape[1]
    n_samples_dof = W.shape[0]

    # ── 1. 秩分析 ────────────────────────────────────────────────────
    print("\n1. 秩分析 (W^T W 特征分解)")
    HH = W.T @ W
    eigvals, eigvecs = np.linalg.eigh(HH)   # 升序
    s_sq     = eigvals
    s_max_sq = s_sq.max()
    svd_thresh = CONFIG['SVD_THRESHOLD']
    thresh_sq  = (svd_thresh * np.sqrt(s_max_sq)) ** 2

    null_mask = (s_sq < thresh_sq)
    n_null = int(null_mask.sum())
    n_rank = n_params - n_null

    print(f"   总参数数       : {n_params}")
    print(f"   可辨识方向(rank): {n_rank}")
    print(f"   零空间维度     : {n_null}")
    cond = np.sqrt(s_max_sq / (s_sq[~null_mask].min() + 1e-12))
    print(f"   条件数         : {cond:.2e}")

    # ── 2. 参数不确定度 ───────────────────────────────────────────────
    print("\n2. 参数不确定度估计")
    tau_arr  = tau_flat.flatten()
    residual = tau_arr - (W @ phi.flatten())
    sigma_res_sq = np.var(residual)
    dof_corr     = max(n_samples_dof - n_rank, 1)
    sigma_res_sq_unbiased = np.sum(residual ** 2) / dof_corr
    print(f"   残差 RMSE     = {np.sqrt(np.mean(residual**2)):.4f} Nm")
    print(f"   σ²_res (无偏) = {sigma_res_sq_unbiased:.4e}")

    V      = eigvecs
    S_pinv = np.where(~null_mask, 1.0 / np.maximum(s_sq, 1e-30), 0.0)
    H_pinv = V @ np.diag(S_pinv) @ V.T
    param_std = np.sqrt(np.maximum(sigma_res_sq_unbiased * np.diag(H_pinv), 0.0))

    # ── 3. 每个参数在原始坐标轴上的"零空间占比" ────────────────────
    # null_load[i] = 在第 i 个参数方向上，零空间分量占总方差的比例
    # 若 null_load[i] 接近 1.0 → 该参数几乎完全在零空间 → 值来自 CAD 先验
    V_null  = eigvecs[:, null_mask]   # (n_params, n_null)
    if n_null > 0:
        null_load = np.sum(V_null ** 2, axis=1)  # (n_params,)  ∈ [0,1]
    else:
        null_load = np.zeros(n_params)

    NULL_THRESH = 0.70   # 零空间占比 > 70% 视为"先验(零空间)"

    # ── 4. 打印对比表 ─────────────────────────────────────────────────
    print("\n3. 参数对比表")
    col_w = 110
    print("-" * col_w)
    print(f"{'参数':<22} {'CAD 先验':>12} {'辨识值':>12} {'标准差':>11} "
          f"{'相对误差%':>10} {'零空间占比':>10} {'来源':<14}")
    print("-" * col_w)

    phi_flat       = phi.flatten()
    phi_prior_flat = phi_prior.flatten()

    source_list = []
    for i in range(n_params):
        label     = param_labels[i] if i < len(param_labels) else f"param_{i}"
        prior_val = phi_prior_flat[i]
        ident_val = phi_flat[i]
        std_val   = param_std[i]
        nl        = null_load[i]

        # 相对误差
        if abs(prior_val) > 1e-9:
            rel_err = 100.0 * abs(ident_val - prior_val) / abs(prior_val)
            rel_err_str = f"{rel_err:>10.2f}"
        else:
            rel_err_str = f"{'—':>10}"

        # 来源判定
        if nl > NULL_THRESH:
            source = "先验(零空间)"
        else:
            source = "辨识"
        source_list.append(source)

        # 在标签上标注来源
        flag = " [CAD]" if source == "先验(零空间)" else ""

        print(f"{label+flag:<22} {prior_val:>12.6f} {ident_val:>12.6f} "
              f"{std_val:>11.4e} {rel_err_str} {nl:>10.3f} {source:<14}")

    print("-" * col_w)

    # ── 5. 固定 Stribeck 形状参数 (仅 stribeck 模型) ──────────────────
    has_stribeck = bool(np.any(vs_arr != 0.0))
    if has_stribeck:
        print("\n4. 固定 Stribeck 形状参数 [CAD固定，不参与辨识]")
        print(f"  {'关节':<8} {'vs (rad/s)':>12} {'delta':>10}")
        print("  " + "-" * 32)
        for j in range(n_dof):
            print(f"  J{j+1:<7} {vs_arr[j]:>12.6f} {delta_arr[j]:>10.4f}")

    # ── 6. 汇总统计 ───────────────────────────────────────────────────
    print("\n5. 汇总统计")
    n_ident   = source_list.count("辨识")
    n_prior   = source_list.count("先验(零空间)")
    n_fixed   = 2 * n_dof if has_stribeck else 0

    print(f"   辨识参数       : {n_ident} 个")
    print(f"   先验(零空间)   : {n_prior} 个  ← 值≈CAD，数据无信息")
    if has_stribeck:
        print(f"   CAD固定(形状)  : {n_fixed} 个  ← vs, delta 不进入 φ 向量")

    rigid_std  = param_std[:10*n_dof]
    fric_std   = param_std[10*n_dof:]
    print(f"   刚体参数 RMSE  : {np.sqrt(np.mean((phi_flat[:10*n_dof]-phi_prior_flat[:10*n_dof])**2)):.4e}")
    print(f"   摩擦参数 RMSE  : {np.sqrt(np.mean((phi_flat[10*n_dof:]-phi_prior_flat[10*n_dof:])**2)):.4e}")

    print("="*75 + "\n")

    return {
        'n_rank':           n_rank,
        'n_null':           n_null,
        'condition_number': cond,
        'param_std':        param_std,
        'null_load':        null_load,
        'source_list':      source_list,
        'sigma_res':        np.sqrt(sigma_res_sq),
    }


# =====================================================================
# 预测 + 评估
# =====================================================================

def predict_torque_stribeck(W_test, phi):
    """用辨识参数预测力矩,返回 (N*n_dof,) 展平向量"""
    return (W_test @ phi).flatten()


def predict_torque(W_test, phi):
    """W_test: (N*n_dof, n_params_total) → τ_pred: (N*n_dof,)"""
    return W_test @ phi


def compute_metrics(tau_true_flat, tau_pred_flat, n_dof):
    """
    逐关节计算 RMSE, MAE, R².
    tau_true_flat / tau_pred_flat: (N*n_dof,) — 展平格式 (sample0_j0, sample0_j1, ...)
    返回 dict 含 per_joint 和 overall 指标
    """
    N = len(tau_true_flat) // n_dof
    tau_true = tau_true_flat.reshape(N, n_dof)
    tau_pred = tau_pred_flat.reshape(N, n_dof)

    rmse_j = np.sqrt(np.mean((tau_true - tau_pred) ** 2, axis=0))
    mae_j  = np.mean(np.abs(tau_true - tau_pred), axis=0)
    ss_res = np.sum((tau_true - tau_pred) ** 2, axis=0)
    ss_tot = np.sum((tau_true - tau_true.mean(axis=0)) ** 2, axis=0)
    r2_j   = 1.0 - ss_res / (ss_tot + 1e-12)

    return {
        'rmse_per_joint': rmse_j,
        'mae_per_joint':  mae_j,
        'r2_per_joint':   r2_j,
        'rmse_overall':   float(np.sqrt(np.mean((tau_true - tau_pred) ** 2))),
        'mae_overall':    float(np.mean(np.abs(tau_true - tau_pred))),
        'r2_overall':     float(1.0 - np.sum(ss_res) / (np.sum(ss_tot) + 1e-12)),
    }


def print_evaluation_report(metrics_train, metrics_test, n_dof):
    """打印训练集 / 测试集评估报告"""
    print("\n" + "="*70)
    print("评估报告")
    print("="*70)
    header = f"{'关节':<8} {'训练RMSE':>10} {'训练MAE':>10} {'训练R²':>8} " \
             f"{'测试RMSE':>10} {'测试MAE':>10} {'测试R²':>8}"
    print(header)
    print("-" * 70)
    for j in range(n_dof):
        print(f"Joint {j+1:<3} "
              f"{metrics_train['rmse_per_joint'][j]:>10.4f} "
              f"{metrics_train['mae_per_joint'][j]:>10.4f} "
              f"{metrics_train['r2_per_joint'][j]:>8.4f} "
              f"{metrics_test['rmse_per_joint'][j]:>10.4f} "
              f"{metrics_test['mae_per_joint'][j]:>10.4f} "
              f"{metrics_test['r2_per_joint'][j]:>8.4f}")
    print("-" * 70)
    print(f"{'总体':<8} "
          f"{metrics_train['rmse_overall']:>10.4f} "
          f"{metrics_train['mae_overall']:>10.4f} "
          f"{metrics_train['r2_overall']:>8.4f} "
          f"{metrics_test['rmse_overall']:>10.4f} "
          f"{metrics_test['mae_overall']:>10.4f} "
          f"{metrics_test['r2_overall']:>8.4f}")
    print("="*70 + "\n")


# =====================================================================
# 绘图函数
# =====================================================================

def plot_and_save_comparison(tau_true_flat, tau_pred_flat, n_dof,
                              title_prefix, save_path, max_len=None,
                              tau_cad_flat=None):
    """逐关节力矩对比图 (真值 vs CAD计算值 vs 辨识计算值)

    tau_cad_flat: 可选，CAD先验参数计算的力矩；若提供则绘制三条曲线
    """
    if max_len is None:
        max_len = CONFIG['PLOT_MAX_LEN']

    N_total = len(tau_true_flat) // n_dof
    N_plot  = min(N_total, max_len)

    tau_true = tau_true_flat.reshape(N_total, n_dof)[:N_plot]
    tau_pred = tau_pred_flat.reshape(N_total, n_dof)[:N_plot]
    tau_cad  = (tau_cad_flat.reshape(N_total, n_dof)[:N_plot]
                if tau_cad_flat is not None else None)

    fig, axes = plt.subplots(n_dof, 1, figsize=(14, 2.5 * n_dof), sharex=True)
    if n_dof == 1:
        axes = [axes]

    t = np.arange(N_plot)
    for j, ax in enumerate(axes):
        ax.plot(t, tau_true[:, j], 'k-',  lw=0.9, alpha=0.85, label='真实测量值')
        if tau_cad is not None:
            ax.plot(t, tau_cad[:, j],  'b-',  lw=0.8, alpha=0.70, label='CAD参数计算值')
        ax.plot(t, tau_pred[:, j], 'r--', lw=0.8, alpha=0.85, label='辨识参数计算值')

        # 计算辨识参数的RMSE/R²
        rmse_id = float(np.sqrt(np.mean((tau_true[:, j] - tau_pred[:, j]) ** 2)))
        ss_res  = np.sum((tau_true[:, j] - tau_pred[:, j]) ** 2)
        ss_tot  = np.sum((tau_true[:, j] - tau_true[:, j].mean()) ** 2) + 1e-12
        r2_id   = float(1.0 - ss_res / ss_tot)

        title_str = f'关节 {j+1}  辨识: RMSE={rmse_id:.4f} Nm  R²={r2_id:.4f}'
        if tau_cad is not None:
            rmse_cad = float(np.sqrt(np.mean((tau_true[:, j] - tau_cad[:, j]) ** 2)))
            title_str += f'  |  CAD: RMSE={rmse_cad:.4f} Nm'

        ax.set_ylabel(f'J{j+1} (Nm)', fontsize=8)
        ax.set_title(title_str, fontsize=8, loc='left')
        ax.legend(loc='upper right', fontsize=7, framealpha=0.6)
        ax.grid(True, lw=0.4, alpha=0.5)

    axes[-1].set_xlabel('样本序号', fontsize=8)
    fig.suptitle(f'{title_prefix} — 力矩对比 (前 {N_plot} 帧)', fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  对比图已保存: {save_path}")


def plot_and_save_metrics_bar_chart(metrics_cad, metrics_ident, n_dof, save_path):
    """各关节 RMSE / R² 柱状图对比 (CAD参数 vs 辨识参数)"""
    joints = [f'J{j+1}' for j in range(n_dof)]
    x     = np.arange(n_dof)
    width = 0.35

    fig, axes = plt.subplots(1, 2, figsize=(14, 4))

    # RMSE 柱状图
    ax = axes[0]
    ax.bar(x - width/2, metrics_cad['rmse_per_joint'], width,
           label='CAD参数', color='steelblue', alpha=0.85)
    ax.bar(x + width/2, metrics_ident['rmse_per_joint'],  width,
           label='辨识参数',  color='tomato',    alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels(joints)
    ax.set_ylabel('RMSE (Nm)'); ax.set_title('各关节 RMSE 对比')
    ax.legend(); ax.grid(axis='y', lw=0.4, alpha=0.5)

    # R² 柱状图
    ax = axes[1]
    ax.bar(x - width/2, metrics_cad['r2_per_joint'], width,
           label='CAD参数', color='steelblue', alpha=0.85)
    ax.bar(x + width/2, metrics_ident['r2_per_joint'],  width,
           label='辨识参数',  color='tomato',    alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels(joints)
    ax.set_ylabel('R²'); ax.set_title('各关节 R² 对比')
    ax.set_ylim(0, 1.05)
    ax.legend(); ax.grid(axis='y', lw=0.4, alpha=0.5)

    fig.suptitle('CAD参数 vs 辨识参数 — 测试集评估对比', fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  指标柱状图已保存: {save_path}")


# =====================================================================
# 参数标签生成 + 保存
# =====================================================================

def build_param_labels(n_dof, friction_model='stribeck'):
    """生成参数标签列表 (块布局: 全部刚体 + 全部摩擦)"""
    rigid_names = ['m', 'mcx', 'mcy', 'mcz', 'Ixx', 'Ixy', 'Iyy', 'Ixz', 'Iyz', 'Izz']
    if friction_model == 'simple':
        fric_names = ['mu_c', 'mu_v']
    else:
        fric_names = ['mu_c', 'mu_s', 'mu_v', 'mu_b']
    labels = []
    for j in range(n_dof):
        for name in rigid_names:
            labels.append(f'L{j+1}_{name}')
    for j in range(n_dof):
        for name in fric_names:
            labels.append(f'L{j+1}_{name}')
    return labels


def save_identified_params(phi, phi_prior, vs_arr, delta_arr,
                            n_dof, param_labels, save_path):
    """
    将辨识参数保存到 .npz 文件，同时导出同名 .csv 文件。
    包含: phi_identified, phi_prior, vs_arr, delta_arr, param_labels
    """
    np.savez(save_path,
             phi_identified = phi,
             phi_prior      = phi_prior,
             vs_arr         = vs_arr,
             delta_arr      = delta_arr,
             param_labels   = np.array(param_labels),
             n_dof          = np.array(n_dof))
    print(f"  辨识参数已保存: {save_path}")

    # ── 同时导出 CSV ──────────────────────────────────────────────────
    csv_path = os.path.splitext(save_path)[0] + '.csv'
    phi_flat  = phi.flatten()
    prior_flat = phi_prior.flatten()
    diff      = phi_flat - prior_flat
    rel_diff  = np.where(np.abs(prior_flat) > 1e-12,
                         diff / np.abs(prior_flat) * 100.0,
                         np.nan)

    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['参数名', 'CAD先验值', '辨识值', '差值', '相对差(%)'])
        for i, label in enumerate(param_labels):
            writer.writerow([
                label,
                f'{prior_flat[i]:.8g}',
                f'{phi_flat[i]:.8g}',
                f'{diff[i]:.8g}',
                f'{rel_diff[i]:.4f}' if not np.isnan(rel_diff[i]) else 'N/A',
            ])
        # 附加 Stribeck 固定参数 (仅 stribeck 模型有非零 vs/delta)
        if np.any(vs_arr != 0.0):
            writer.writerow([])
            writer.writerow(['--- Stribeck固定参数 (CAD值,不参与辨识) ---'])
            writer.writerow(['参数名', 'CAD值'])
            for j in range(n_dof):
                writer.writerow([f'J{j+1}_vs',    f'{vs_arr[j]:.8g}'])
                writer.writerow([f'J{j+1}_delta', f'{delta_arr[j]:.8g}'])

    print(f"  参数CSV已保存: {csv_path}")


# =====================================================================
# 主函数
# =====================================================================

def main(friction_model=None):
    print("="*70)
    print("PC-IRLS 改进版 — Franka FR3 动力学参数辨识")
    print("="*70 + "\n")

    cfg      = CONFIG
    n_dof    = cfg['N_DOF']
    # 命令行参数优先于 CONFIG 默认值
    fric_model = friction_model if friction_model else cfg['FRICTION_MODEL']
    n_fric   = 2 if fric_model == 'simple' else 4
    n_pj     = 10 + n_fric                    # 每关节参数数 (12 或 14)
    n_params = n_pj * n_dof
    print(f"  摩擦模型: {fric_model}  (每关节 {n_pj} 参数, 总计 {n_params})")

    # ── 1. 加载 CAD 先验 ──────────────────────────────────────────────
    print("[1/7] 加载 CAD 先验参数...")
    phi_prior, vs_arr, delta_arr = load_cad_prior(
        cfg['CONFIG_YAML'], n_dof, friction_model=fric_model)
    print(f"  phi_prior shape: {phi_prior.shape}  ({n_pj} x {n_dof} = {n_params})")

    # ── 2. 加载训练数据 ────────────────────────────────────────────────
    print("\n[2/7] 加载训练数据...")
    train_data = np.load(cfg['TRAIN_DATA_PATH'])
    q_tr   = train_data['q']  [:cfg['NUM_TRAIN_SAMPLES']]
    qd_tr  = train_data['qd'] [:cfg['NUM_TRAIN_SAMPLES']]
    qdd_tr = train_data['qdd'][:cfg['NUM_TRAIN_SAMPLES']]
    tau_tr = train_data['tau'][:cfg['NUM_TRAIN_SAMPLES']]
    n_tr   = len(q_tr)
    print(f"  训练样本数: {n_tr}")

    # ── 2b. 构建 pinocchio 模型 ────────────────────────────────────────
    print("\n  构建 Pinocchio 模型...")
    with open(cfg['CONFIG_YAML'], 'r', encoding='utf-8') as f:
        yaml_cfg = yaml.safe_load(f)
    mdh_params     = yaml_cfg['mdh_parameters']
    gravity_vector = yaml_cfg.get('gravity_vector', [0.0, 0.0, -9.81])
    pin_model      = build_pinocchio_model_from_mdh(mdh_params, gravity_vector)
    pin_data       = pin_model.createData()
    print(f"  Pinocchio 模型构建完毕 ({pin_model.nq} DOF)")

    # ── 3. 构建训练回归矩阵 ────────────────────────────────────────────
    print("\n[3/7] 构建回归矩阵 W_train ...")
    if fric_model == 'simple':
        W_train, tau_tr_flat_raw, _ = build_simple_regressor(
            pin_model, pin_data, q_tr, qd_tr, qdd_tr, tau_tr, n_tr, n_dof)
    else:
        W_train, tau_tr_flat_raw, _ = build_stribeck_regressor(
            pin_model, pin_data, q_tr, qd_tr, qdd_tr, tau_tr,
            n_tr, n_dof, vs_arr, delta_arr)
    tau_tr_flat = tau_tr.flatten()
    print(f"  W_train shape: {W_train.shape}")

    # ── 4. PC-IRLS 辨识 ────────────────────────────────────────────────
    print("\n[4/7] 执行 PC-IRLS 改进版辨识...")
    phi_ident = solve_pc_irls_improved(
        W_train, tau_tr_flat, n_dof, phi_prior, vs_arr, delta_arr, cfg)

    # ── 5. 基参数分析 ──────────────────────────────────────────────────
    print("\n[5/7] 基参数分析与不确定度报告...")
    param_labels = build_param_labels(n_dof, friction_model=fric_model)
    analysis = analyze_base_parameters(
        W_train, tau_tr_flat, phi_ident, n_dof, phi_prior,
        param_labels, vs_arr, delta_arr)

    # ── 6. 加载测试数据 + 评估指标 ────────────────────────────────────
    print("\n[6/7] 加载测试数据并评估...")
    test_data = np.load(cfg['TEST_DATA_PATH'])
    q_te   = test_data['q']  [:cfg['NUM_TEST_SAMPLES']]
    qd_te  = test_data['qd'] [:cfg['NUM_TEST_SAMPLES']]
    qdd_te = test_data['qdd'][:cfg['NUM_TEST_SAMPLES']]
    tau_te = test_data['tau'][:cfg['NUM_TEST_SAMPLES']]
    n_te   = len(q_te)
    print(f"  测试样本数: {n_te}")

    if fric_model == 'simple':
        W_test, _, _ = build_simple_regressor(
            pin_model, pin_data, q_te, qd_te, qdd_te, tau_te, n_te, n_dof)
    else:
        W_test, _, _ = build_stribeck_regressor(
            pin_model, pin_data, q_te, qd_te, qdd_te, tau_te,
            n_te, n_dof, vs_arr, delta_arr)
    tau_te_flat = tau_te.flatten()

    tau_pred_tr = predict_torque(W_train, phi_ident)
    tau_pred_te = predict_torque(W_test,  phi_ident)

    # 计算 CAD 参数的预测力矩（用于三曲线对比）
    tau_cad_tr = predict_torque(W_train, phi_prior)
    tau_cad_te = predict_torque(W_test,  phi_prior)

    metrics_train = compute_metrics(tau_tr_flat, tau_pred_tr, n_dof)
    metrics_test  = compute_metrics(tau_te_flat,  tau_pred_te, n_dof)
    # CAD 参数在测试集上的指标（用于柱状图对比）
    metrics_cad_te = compute_metrics(tau_te_flat, tau_cad_te, n_dof)
    print_evaluation_report(metrics_train, metrics_test, n_dof)

    # ── 7. 保存参数 + 绘图 ─────────────────────────────────────────────
    print("\n[7/7] 保存参数与生成绘图...")
    out_dir = os.path.dirname(os.path.abspath(__file__))
    sfx = f'_{fric_model}'   # 文件名后缀，区分两种摩擦模型的输出

    save_identified_params(
        phi_ident, phi_prior, vs_arr, delta_arr, n_dof, param_labels,
        os.path.join(out_dir, f'pc_irls_improved_params{sfx}.npz'))

    plot_and_save_comparison(
        tau_tr_flat, tau_pred_tr, n_dof,
        title_prefix=f'训练集[{fric_model}]',
        save_path=os.path.join(out_dir, f'pc_irls_train_comparison{sfx}.png'),
        tau_cad_flat=tau_cad_tr)

    plot_and_save_comparison(
        tau_te_flat, tau_pred_te, n_dof,
        title_prefix=f'测试集[{fric_model}]',
        save_path=os.path.join(out_dir, f'pc_irls_test_comparison{sfx}.png'),
        tau_cad_flat=tau_cad_te)

    plot_and_save_metrics_bar_chart(
        metrics_cad_te, metrics_test, n_dof,
        save_path=os.path.join(out_dir, f'pc_irls_metrics_bar{sfx}.png'))

    print("\n" + "="*70)
    print("辨识完成!")
    print(f"  测试集 RMSE:  {metrics_test['rmse_overall']:.4f} Nm")
    print(f"  测试集 R²:    {metrics_test['r2_overall']:.4f}")
    print(f"  秩 / 总参数: {analysis['n_rank']} / {n_params}  "
          f"(零空间维度: {analysis['n_null']})")
    print(f"  条件数:       {analysis['condition_number']:.2e}")
    print(f"  输出目录:     {out_dir}")
    print("="*70 + "\n")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='PC-IRLS 动力学参数辨识')
    parser.add_argument('--friction', choices=['simple', 'stribeck'], default=None,
                        help="摩擦模型: simple(2参数) 或 stribeck(4参数); 缺省用 CONFIG 值")
    args = parser.parse_args()
    main(friction_model=args.friction)
