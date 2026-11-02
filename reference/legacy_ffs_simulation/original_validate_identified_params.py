#!/usr/bin/env python3
"""名义参数 (phi_prior) vs 辨识参数 (phi_identified) 在真机轨迹数据上的力矩预测对比。

对指定的每条 FFS 轨迹 CSV:
  1. 读取 q, dq, tau_J (实测关节力矩, 含重力分量, 已由准静态段验证)
     以及 FCI 内部模型的重力列 gravity_0~6 (G_csv)
  2. 数值微分 dq 得到 qdd (中心差分 + Savitzky-Golay 平滑)
  3. 用与辨识完全相同的回归矩阵 (Pinocchio computeJointTorqueRegressor,
     经典 FR3 MDH d3=0.316, 重力 [0,0,-9.81]) 预测力矩
  4. 对比指标 (重力替代):  tau_pred = H + C + G_csv + 摩擦   vs tau_J
     用 FCI 自己的重力列替代模型重力。实测表明 tau_J 始终跟随 G_csv,
     因此该指标不依赖任何负载假设, 纯粹检验辨识的惯量 + 摩擦质量
  5. 输出 RMSE / MAE / R² / 改善百分比 (逐关节 + 汇总), 可选绘图

用法示例:
  source /opt/ros/humble/setup.bash
  python validate_identified_params.py --split test --indices 1 5 10
  python validate_identified_params.py --split test --indices 390
  python validate_identified_params.py --split test --count 5 --seed 42
  python validate_identified_params.py --split train --indices 2 3 4 --no-plots

模型验证结论 (本脚本 MDH 与结构依据):
  - MDH (alpha,a,d,theta_offset) 采用经典 FR3 修正 DH (d3=0.316);
    用 phi_prior 复现无负载轨迹的 gravity 列误差 ~2e-6, mass 矩阵误差 ~0.0
  - 采集时未附加任何负载, 脚本不做负载拟合; G_csv 直接取自 CSV
"""

import argparse
import glob
import os
import re
import sys

import numpy as np

EPS_SIGN = 0.05  # 与 pc_irls_improved.py CONFIG['SMOOTH_SIGN_EPSILON'] 一致
N_DOF = 7
GRAVITY = np.array([0.0, 0.0, -9.81])

# 经典 Franka FR3 修正 DH (Craig): (alpha, a, d, theta_offset)
MDH_PARAMS = [
    (0.0, 0.0, 0.333, 0.0),
    (-np.pi / 2, 0.0, 0.0, 0.0),
    (np.pi / 2, 0.0, 0.316, 0.0),
    (np.pi / 2, 0.0825, 0.0, 0.0),
    (-np.pi / 2, -0.0825, 0.384, 0.0),
    (np.pi / 2, 0.0, 0.0, 0.0),
    (np.pi / 2, 0.088, 0.0, 0.0),
]

# CSV 列索引 (0-based)
COL_TIME, COL_PERIOD = 0, 1
COL_Q, COL_DQ, COL_TAU_J, COL_GRAV = 2, 9, 30, 123


def build_pinocchio_model(mdh_params, gravity_vector):
    """与 pc_irls_improved.py 完全相同的 7 关节模型 (占位惯量不影响回归矩阵)。"""
    import pinocchio as pin

    model = pin.Model()
    model.gravity.linear = np.array(gravity_vector)
    parent_id = 0
    for i, (alpha, a, d, theta_offset) in enumerate(mdh_params):
        placement = pin.SE3(
            pin.utils.rotate("x", alpha), np.array([a, 0.0, 0.0])
        ) * pin.SE3(pin.utils.rotate("z", theta_offset), np.array([0.0, 0.0, d]))
        joint_id = model.addJoint(parent_id, pin.JointModelRZ(), placement, f"joint_{i+1}")
        model.appendBodyToJoint(joint_id, pin.Inertia.FromSphere(1.0, 0.01), pin.SE3.Identity())
        parent_id = joint_id
    return model


def load_identified_params(npz_path):
    d = np.load(npz_path, allow_pickle=True)
    phi_prior = np.asarray(d["phi_prior"], dtype=np.float64).reshape(-1)
    phi_ident = np.asarray(d["phi_identified"], dtype=np.float64).reshape(-1)
    assert phi_prior.shape == (84,) and phi_ident.shape == (84,), (
        f"参数维度异常: prior={phi_prior.shape}, identified={phi_ident.shape}, 期望 (84,)"
    )
    return phi_prior, phi_ident


def resolve_trajectories(csv_dir, split, indices, count, seed):
    """返回 [(index, csv_path), ...]。同名 index 有多个文件时取 mtime 最新的。"""
    pattern = os.path.join(csv_dir, split, "trajectory_*.csv")
    by_index = {}
    for path in glob.glob(pattern):
        m = re.search(r"trajectory_(\d+)", os.path.basename(path))
        if not m:
            continue
        idx = int(m.group(1))
        if idx not in by_index or os.path.getmtime(path) > os.path.getmtime(by_index[idx]):
            by_index[idx] = path
    if not by_index:
        sys.exit(f"错误: 在 {os.path.join(csv_dir, split)} 下没有找到 trajectory_*.csv")

    if indices:
        chosen = []
        for i in indices:
            if i not in by_index:
                print(f"警告: {split} 中不存在 trajectory_{i:03d}, 跳过")
            else:
                chosen.append(i)
        if not chosen:
            sys.exit("错误: 指定的轨迹索引均不存在")
    elif count:
        available = sorted(by_index)
        rng = np.random.default_rng(seed)
        chosen = sorted(rng.choice(available, size=min(count, len(available)), replace=False))
    else:
        sys.exit("错误: 请用 --indices 指定轨迹索引, 或用 --count 随机抽取")
    return [(i, by_index[i]) for i in chosen]


def load_trajectory(path, max_samples):
    import pandas as pd

    usecols = [COL_TIME, COL_PERIOD] + list(range(COL_Q, COL_Q + 7)) + \
              list(range(COL_DQ, COL_DQ + 7)) + list(range(COL_TAU_J, COL_TAU_J + 7)) + \
              list(range(COL_GRAV, COL_GRAV + 7))
    df = pd.read_csv(path, usecols=usecols)
    if max_samples and max_samples > 0:
        df = df.iloc[:max_samples]
    period = df.iloc[:, 1].to_numpy(np.float64) / 1e9
    dt = float(np.mean(period))
    if np.std(period) / dt > 0.01:
        print(f"  警告: period_ns 波动较大 (std/mean={np.std(period)/dt:.3f}), 使用平均 dt={dt:.6f}s")
    q = df.iloc[:, 2:9].to_numpy(np.float64)
    dq = df.iloc[:, 9:16].to_numpy(np.float64)
    tau_j = df.iloc[:, 16:23].to_numpy(np.float64)
    grav_csv = df.iloc[:, 23:30].to_numpy(np.float64)
    return q, dq, tau_j, grav_csv, dt


def compute_qdd(dq, dt, window, polyorder):
    from scipy.signal import savgol_filter

    qdd = np.gradient(dq, dt, axis=0)
    window = min(window, len(dq) - (len(dq) % 2 == 0))
    if window > polyorder and len(dq) > window:
        qdd = savgol_filter(qdd, window, polyorder, axis=0, mode="interp")
    return qdd


def regressor(model, data, q, qd, qdd):
    """逐样本计算回归矩阵, 返回 (N, 7, 70)。"""
    import pinocchio as pin

    n = len(q)
    Y = np.zeros((n, N_DOF, 10 * N_DOF))
    print_step = max(1, n // 10)
    for i in range(n):
        if i > 0 and i % print_step == 0:
            print(f"    回归矩阵进度: {i}/{n} ({100.0*i/n:.0f}%)")
        Y[i] = pin.computeJointTorqueRegressor(model, data, q[i], qd[i], qdd[i])
    return Y


def friction_torque(dq, phi_fric):
    tau_f = np.zeros((len(dq), N_DOF))
    for j in range(N_DOF):
        mu_c, mu_v = phi_fric[2 * j], phi_fric[2 * j + 1]
        tau_f[:, j] = mu_c * np.tanh(dq[:, j] / EPS_SIGN) + mu_v * dq[:, j]
    return tau_f


def compute_metrics(tau_true, tau_pred):
    err = tau_pred - tau_true
    rmse = np.sqrt(np.mean(err**2, axis=0))
    mae = np.mean(np.abs(err), axis=0)
    ss_res = np.sum(err**2, axis=0)
    ss_tot = np.sum((tau_true - np.mean(tau_true, axis=0)) ** 2, axis=0)
    with np.errstate(divide="ignore", invalid="ignore"):
        r2 = 1.0 - np.where(ss_tot > 1e-12, ss_res / ss_tot, 0.0)
    return rmse, mae, r2


def fmt_row(name, rmse_n, rmse_i, mae_n, mae_i, r2_n, r2_i):
    if rmse_n > 1e-9:
        imp_s = f"{(rmse_n - rmse_i) / rmse_n * 100.0:+7.1f}%"
    else:
        imp_s = "   N/A "
    return (f"{name:8s} {rmse_n:10.4f} {rmse_i:10.4f} {imp_s:9s} | "
            f"{mae_n:9.4f} {mae_i:9.4f} | {r2_n:7.3f} {r2_i:7.3f}")


def print_table(title, rmse_n, rmse_i, mae_n, mae_i, r2_n, r2_i):
    print(title)
    print(f"{'关节':8s} {'RMSE名义':>10s} {'RMSE辨识':>10s} {'改善':>9s} | "
          f"{'MAE名义':>9s} {'MAE辨识':>9s} | {'R2名义':>7s} {'R2辨识':>7s}")
    print("-" * 84)
    for j in range(N_DOF):
        print(fmt_row(f"J{j+1}", rmse_n[j], rmse_i[j], mae_n[j], mae_i[j], r2_n[j], r2_i[j]))
    print("-" * 84)
    print(fmt_row("平均", rmse_n.mean(), rmse_i.mean(), mae_n.mean(), mae_i.mean(),
                  r2_n.mean(), r2_i.mean()))
    print()


def plot_trajectory(idx, path, dt, tau_j, tau_nom, tau_id, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(tau_j)) * dt
    stride = max(1, len(tau_j) // 2000)
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(f"trajectory {idx}: measured tau_J vs nominal / identified (gravity-replaced)")
    for j in range(7):
        ax = axes[j]
        ax.plot(t[::stride], tau_j[::stride, j], "k-", lw=0.7, label="measured tau_J")
        ax.plot(t[::stride], tau_nom[::stride, j], "b--", lw=0.7, label="nominal (phi_prior)")
        ax.plot(t[::stride], tau_id[::stride, j], "r-", lw=0.7, label="identified")
        ax.set_ylabel(f"J{j+1} [Nm]")
        ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    path_out = os.path.join(out_dir, f"trajectory_{idx:03d}.png")
    fig.savefig(path_out, dpi=110)
    plt.close(fig)
    print(f"  图已保存: {path_out}")


def plot_summary(results, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rmse_n = np.array([r["rmse_nom"] for r in results])
    rmse_i = np.array([r["rmse_id"] for r in results])
    x = np.arange(N_DOF)
    width = 0.38
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.5))
    ax1.bar(x - width / 2, rmse_n.mean(axis=0), width, label="nominal (phi_prior)", color="#4C72B0")
    ax1.bar(x + width / 2, rmse_i.mean(axis=0), width, label="identified", color="#C44E52")
    ax1.set_xticks(x, [f"J{j+1}" for j in range(7)])
    ax1.set_ylabel("mean RMSE [Nm]")
    ax1.set_title(f"gravity-replaced torque RMSE vs tau_J (avg over {len(results)} trajectories)")
    ax1.legend()
    ax1.grid(alpha=0.3, axis="y")

    imp = (rmse_n.mean(axis=0) - rmse_i.mean(axis=0)) / np.maximum(rmse_n.mean(axis=0), 1e-12) * 100
    colors = ["#2E8B57" if v >= 0 else "#C44E52" for v in imp]
    ax2.bar(x, imp, width * 2, color=colors)
    ax2.axhline(0, color="k", lw=0.8)
    ax2.set_xticks(x, [f"J{j+1}" for j in range(7)])
    ax2.set_ylabel("RMSE improvement [%]")
    ax2.set_title("identified vs nominal improvement (positive = better)")
    ax2.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    path_out = os.path.join(out_dir, "summary.png")
    fig.savefig(path_out, dpi=110)
    plt.close(fig)
    print(f"  汇总图已保存: {path_out}")


def main():
    parser = argparse.ArgumentParser(
        description="对比名义参数与辨识参数在真机轨迹上的力矩预测效果",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--split", default="test", choices=["train", "test"],
                        help="数据划分目录 (ffs_data 下的子目录)")
    parser.add_argument("--indices", type=int, nargs="+", default=None,
                        help="要对比的轨迹索引列表, 如 --indices 1 5 10")
    parser.add_argument("--count", type=int, default=None,
                        help="随机抽取的轨迹条数 (与 --indices 二选一)")
    parser.add_argument("--seed", type=int, default=0, help="--count 随机种子")
    parser.add_argument("--csv-dir", default=os.path.expanduser("~/franka_ros2_ws/ffs_data"))
    parser.add_argument("--npz", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     "pc_irls_improved_params_simple.npz"),
                        help="辨识结果 npz 路径")
    parser.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                         "results", "identified_vs_nominal"))
    parser.add_argument("--max-samples", type=int, default=0, help="每条轨迹最多使用样本数 (0=全部)")
    parser.add_argument("--qdd-window", type=int, default=21, help="qdd Savitzky-Golay 窗长")
    parser.add_argument("--qdd-polyorder", type=int, default=3)
    parser.add_argument("--no-plots", action="store_true", help="不生成图表")
    parser.add_argument("--no-csv", action="store_true", help="不保存结果 CSV")
    args = parser.parse_args()

    phi_prior, phi_ident = load_identified_params(args.npz)
    print(f"辨识结果: {args.npz}")
    print(f"  phi_prior (名义)  : 质量 {phi_prior[0]:.3f}..{phi_prior[60]:.3f} kg, "
          f"摩擦 mu_c={phi_prior[70]:.3f} mu_v={phi_prior[71]:.2f} (J1)")
    print(f"  phi_identified    : 质量 {phi_ident[0]:.3f}..{phi_ident[60]:.3f} kg, "
          f"摩擦 mu_c={phi_ident[70]:.3f} mu_v={phi_ident[71]:.2e} (J1)")

    model = build_pinocchio_model(MDH_PARAMS, GRAVITY)
    data = model.createData()

    trajs = resolve_trajectories(args.csv_dir, args.split, args.indices, args.count, args.seed)
    print(f"待对比轨迹 ({len(trajs)} 条): {[i for i, _ in trajs]}\n")

    results = []
    for idx, path in trajs:
        print(f"加载 {path} ...")
        q, dq, tau_j, grav_csv, dt = load_trajectory(path, args.max_samples)
        qdd = compute_qdd(dq, dt, args.qdd_window, args.qdd_polyorder)
        n = len(q)
        print(f"  {n} 样本, dt={dt*1000:.2f} ms; 计算回归矩阵...")
        Y_full = regressor(model, data, q, dq, qdd)
        Y_g = regressor(model, data, q, np.zeros_like(dq), np.zeros_like(qdd))

        tau_f_nom = friction_torque(dq, phi_prior[70:84])
        tau_f_id = friction_torque(dq, phi_ident[70:84])
        HCG_nom = Y_full @ phi_prior[:70]
        HCG_id = Y_full @ phi_ident[:70]
        G_nom = Y_g @ phi_prior[:70]
        G_id = Y_g @ phi_ident[:70]

        # 重力替代: 用 FCI 的 G_csv 替换模型重力, 不依赖任何负载假设
        tau_nom = HCG_nom - G_nom + grav_csv + tau_f_nom
        tau_id = HCG_id - G_id + grav_csv + tau_f_id

        rmse_n, mae_n, r2_n = compute_metrics(tau_j, tau_nom)
        rmse_i, mae_i, r2_i = compute_metrics(tau_j, tau_id)

        print("=" * 84)
        print(f"轨迹 {idx}: {os.path.basename(path)}  ({n} 样本, {n*dt:.1f} s)")
        print("=" * 84)
        print_table("指标 — 重力替代 (tau_pred = H + C + G_csv + 摩擦):",
                    rmse_n, rmse_i, mae_n, mae_i, r2_n, r2_i)

        results.append({"index": idx, "path": os.path.basename(path),
                        "rmse_nom": rmse_n, "rmse_id": rmse_i,
                        "mae_nom": mae_n, "mae_id": mae_i,
                        "r2_nom": r2_n, "r2_id": r2_i})

        if not args.no_plots:
            plot_dir = os.path.join(args.outdir, "plots")
            os.makedirs(plot_dir, exist_ok=True)
            plot_trajectory(idx, path, dt, tau_j, tau_nom, tau_id, plot_dir)

    # 汇总
    rmse_n = np.array([r["rmse_nom"] for r in results])
    rmse_i = np.array([r["rmse_id"] for r in results])
    mae_n = np.array([r["mae_nom"] for r in results])
    mae_i = np.array([r["mae_id"] for r in results])
    r2_n = np.array([r["r2_nom"] for r in results])
    r2_i = np.array([r["r2_id"] for r in results])

    print("=" * 84)
    print(f"汇总 ({len(results)} 条轨迹, 逐关节均值) — 指标 (重力替代)")
    print("=" * 84)
    print_table("", rmse_n.mean(axis=0), rmse_i.mean(axis=0),
                mae_n.mean(axis=0), mae_i.mean(axis=0), r2_n.mean(axis=0), r2_i.mean(axis=0))

    total_imp = (rmse_n.mean() - rmse_i.mean()) / rmse_n.mean() * 100.0
    print(f"总体 RMSE 改善: {total_imp:+.2f}%  "
          f"(名义 {rmse_n.mean():.4f} -> 辨识 {rmse_i.mean():.4f} Nm)")

    os.makedirs(args.outdir, exist_ok=True)
    if not args.no_csv:
        import csv as csv_mod
        with open(os.path.join(args.outdir, "aggregate_metrics.csv"), "w", newline="",
                  encoding="utf-8-sig") as f:
            w = csv_mod.writer(f)
            w.writerow(["trajectory", "joint",
                        "rmse_nom", "rmse_id", "improvement_pct",
                        "mae_nom", "mae_id", "r2_nom", "r2_id"])
            for r in results:
                for j in range(N_DOF):
                    imp = (r["rmse_nom"][j] - r["rmse_id"][j]) / r["rmse_nom"][j] * 100.0
                    w.writerow([r["index"], f"J{j+1}",
                                f"{r['rmse_nom'][j]:.6f}", f"{r['rmse_id'][j]:.6f}", f"{imp:.3f}",
                                f"{r['mae_nom'][j]:.6f}", f"{r['mae_id'][j]:.6f}",
                                f"{r['r2_nom'][j]:.6f}", f"{r['r2_id'][j]:.6f}"])
        print(f"结果 CSV 已保存: {os.path.join(args.outdir, 'aggregate_metrics.csv')}")

    if not args.no_plots:
        plot_summary(results, os.path.join(args.outdir, "plots"))


if __name__ == "__main__":
    main()
