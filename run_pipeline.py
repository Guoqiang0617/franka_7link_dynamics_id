#!/usr/bin/env python3
"""Run the copied FR3 project on Windows or Linux using the active Python environment."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from importlib.metadata import version


ROOT = Path(__file__).resolve().parent


def run(script, *arguments):
    command = [sys.executable, str(ROOT / script), *(str(x) for x in arguments)]
    print(f"\nRunning {Path(script).name}", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "full"), default="quick")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--stride", type=int, default=None)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--skip-prepare", action="store_true", help="Reuse the already prepared data for this profile.")
    parser.add_argument("--trajectory", type=Path, default=ROOT / "data/trajectories/test/trajectory_350_20260515_230214.json")
    parser.add_argument("--csv", type=Path, default=ROOT / "data/raw/test/trajectory_350_20260515_230214.csv")
    args = parser.parse_args()
    if args.threads < 1:
        parser.error("--threads must be positive")
    for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[name] = str(args.threads)
    os.environ["PYTHONUTF8"] = "1"
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.environ["PYTHONUNBUFFERED"] = "1"
    quick = args.profile == "quick"
    stride = args.stride or (50 if quick else 10)
    processed = ROOT / "data/processed" / "quick" if quick else ROOT / "data/processed"
    output = (args.output_dir or (ROOT / "results/quick" if quick else ROOT / "results")).resolve()
    generated_models = output / "models" if quick or args.output_dir else ROOT / "models/generated"
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    run("tools/check_environment.py")
    if not args.skip_prepare:
        options = ["--data-root", ROOT / "data", "--output-dir", processed, "--stride", stride]
        if quick:
            options += ["--max-files", 5]
        run("scripts/prepare_dataset.py", *options)
    for split in ("train", "test"):
        if not (processed / f"{split}.npz").is_file():
            raise FileNotFoundError(f"Missing prepared {split} data: {processed}; omit --skip-prepare.")
    options = ["--train", processed / "train.npz", "--output-dir", output / "identification"]
    if quick:
        options += ["--max-iterations", 5]
    run("scripts/identify_parameters.py", *options)
    parameters = output / "identification/identified_params.npz"
    run("scripts/evaluate_parameters.py", "--test", processed / "test.npz", "--parameters", parameters,
        "--output-dir", output / "evaluation")
    run("scripts/export_model.py", "--parameters", parameters,
        "--urdf-output", generated_models / "fr3_identified.urdf", "--mjcf-dir", generated_models)
    run("scripts/run_ffs_simulation.py", "--trajectory", args.trajectory.resolve(), "--csv", args.csv.resolve(),
        "--parameters", parameters, "--output-dir", output / "simulation")

    fit = json.loads((output / "identification/fit_report.json").read_text(encoding="utf-8"))
    evaluation = json.loads((output / "evaluation/test_metrics.json").read_text(encoding="utf-8"))
    manifest = {
        "completed_utc": datetime.now(timezone.utc).isoformat(),
        "profile": args.profile,
        "quick_profile_is_workflow_check_only": quick,
        "python": sys.version,
        "platform": platform.platform(),
        "versions": {name: version(name) for name in ("numpy", "scipy", "pandas", "cvxpy", "mujoco")},
        "elapsed_seconds": time.perf_counter() - started,
        "threads": args.threads,
        "processed_data": str(processed),
        "model_output": str(generated_models),
        "trajectory": str(args.trajectory.resolve()),
        "experimental_csv": str(args.csv.resolve()),
        "input_checksums": {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in ("data/MANIFEST.csv", "config/fr3_model.yaml", "config/identification.yaml")
        },
    }
    (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    lines = ["# FR3 七连杆离线流程运行报告", "", f"运行模式：`{args.profile}`。",
             "快速模式只核对流程，不应当作为完整数据辨识结论。" if quick else "本次使用包内完整 train/test 数据。",
             "", f"训练样本 {fit['training_sample_count']}；设计矩阵秩 {fit['rank']}/84，零空间维数 {fit['nullity']}。",
             f"求解器：{fit['solver']}；IRLS 迭代 {fit['iterations']} 次，收敛标记 {fit['converged']}。",
             "", "| 测试评估 | 名义总体 RMSE / Nm | 辨识总体 RMSE / Nm |", "|---|---:|---:|"]
    for mode, label in (("full_dynamics", "完整逆动力学"), ("gravity_replaced", "替换重力项")):
        lines.append(f"| {label} | {evaluation[mode]['nominal']['rmse_overall_nm']:.6f} | {evaluation[mode]['identified']['rmse_overall_nm']:.6f} |")
    lines += ["", "逐关节结果：[test_metrics.json](evaluation/test_metrics.json)。",
              "参数和物理可行性：[fit_report.json](identification/fit_report.json)。",
              "", "![测试集力矩对比](evaluation/test_rmse_comparison.png)",
              "", "![闭环仿真对比](simulation/ffs_simulation_comparison.png)",
              "", "![位置误差](simulation/error_zoom.png)", "",
              "仿真是同一控制律在两个模型上的敏感性对比。跟踪误差减小本身不证明模型更接近真机；请同时查看测试集力矩预测和 simulation_metrics.json 中与实测数据的误差。",
              "", "固定电机/减速器 armature 不参与七连杆参数估计。与历史参考图的设置差别见 docs/MODEL_EXPORT_AND_SIMULATION.md。"]
    if fit.get('interpretation_notes'):
        lines += ["", "辨识结果中有连杆质量明显偏离名义先验。它们是当前模型与物理约束下的可行代表解，不能当作逐连杆称重结果：", ""]
        lines.extend(f"- {note}" for note in fit['interpretation_notes'])
    (output / "RUN_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nCompleted {args.profile} pipeline in {manifest['elapsed_seconds']:.1f} s")
    print(f"Open {output / 'RUN_REPORT.md'}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"Pipeline stopped: {Path(exc.cmd[1]).name} failed (exit {exc.returncode}).", file=sys.stderr)
        sys.exit(exc.returncode)
