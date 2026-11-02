"""Held-out torque prediction with full and gravity-replaced dynamics."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .estimation import build_design_matrix, build_pinocchio_model, load_config
from .parameter_io import load_parameter_vectors


class MetricAccumulator:
    def __init__(self):
        self.count = 0
        self.squared = np.zeros(7)
        self.absolute = np.zeros(7)
        self.target_sum = np.zeros(7)
        self.target_squared = np.zeros(7)

    def add(self, target, prediction):
        error = prediction - target
        self.count += len(target)
        self.squared += np.sum(error**2, axis=0)
        self.absolute += np.sum(np.abs(error), axis=0)
        self.target_sum += np.sum(target, axis=0)
        self.target_squared += np.sum(target**2, axis=0)

    def report(self):
        if not self.count:
            raise ValueError("No held-out samples were evaluated.")
        variation = self.target_squared - self.target_sum**2 / self.count
        r2 = 1.0 - self.squared / np.maximum(variation, 1e-12)
        return {
            "rmse_per_joint_nm": np.sqrt(self.squared / self.count).tolist(),
            "mae_per_joint_nm": (self.absolute / self.count).tolist(),
            "r2_per_joint": r2.tolist(),
            "rmse_overall_nm": float(np.sqrt(self.squared.sum() / (7 * self.count))),
            "mae_overall_nm": float(self.absolute.sum() / (7 * self.count)),
            "r2_mean": float(np.mean(r2)),
            "sample_count": self.count,
        }


def evaluate_parameters(test_file, parameter_file, config_path, output_dir,
                        batch_size=1000, max_samples=None):
    if batch_size < 1:
        raise ValueError("batch_size must be positive.")
    config = load_config(config_path)
    prior, identified = load_parameter_vectors(parameter_file, config)
    with np.load(test_file) as dataset:
        arrays = {key: np.asarray(dataset[key], dtype=float)
                  for key in ("q", "qd", "qdd", "tau", "gravity")}
        time = np.asarray(dataset["time"], dtype=float)
        source_id = np.asarray(dataset["source_id"], dtype=int)
        source_files = dataset["source_files"].tolist()
    if max_samples is not None:
        if max_samples < 1:
            raise ValueError("max_samples must be positive.")
        selected = np.linspace(0, len(time) - 1, min(max_samples, len(time)), dtype=int)
        arrays = {key: value[selected] for key, value in arrays.items()}
        time, source_id = time[selected], source_id[selected]

    pin, model = build_pinocchio_model(config)
    metrics = {mode: {name: MetricAccumulator() for name in ("nominal", "identified")}
               for mode in ("full_dynamics", "gravity_replaced")}
    example = {"time": [], "measured": []}
    for mode in metrics:
        for name in metrics[mode]:
            example[f"{mode}_{name}"] = []
    for begin in range(0, len(time), batch_size):
        end = min(begin + batch_size, len(time))
        q, qd, qdd = (arrays[key][begin:end] for key in ("q", "qd", "qdd"))
        design = build_design_matrix(pin, model, q, qd, qdd, report_progress=False)
        gravity_design = build_design_matrix(
            pin, model, q, np.zeros_like(qd), np.zeros_like(qdd), report_progress=False
        )[:, :70]
        measured = arrays["tau"][begin:end]
        mask = source_id[begin:end] == source_id[0]
        example["time"].append(time[begin:end][mask])
        example["measured"].append(measured[mask])
        for name, values in (("nominal", prior), ("identified", identified)):
            full = (design @ values).reshape(-1, 7)
            replaced = full - (gravity_design @ values[:70]).reshape(-1, 7) + arrays["gravity"][begin:end]
            for mode, prediction in (("full_dynamics", full), ("gravity_replaced", replaced)):
                metrics[mode][name].add(measured, prediction)
                example[f"{mode}_{name}"].append(prediction[mask])
        if end == len(time) or (begin // batch_size) % 25 == 0:
            print(f"Evaluated {end}/{len(time)} held-out samples", flush=True)

    report = {
        "test_file": str(Path(test_file).resolve()),
        "parameter_file": str(Path(parameter_file).resolve()),
        "full_dynamics_formula": "Y(q,dq,qdd)*theta + friction(dq)",
        "gravity_replaced_formula": "(Y(q,dq,qdd)-Y(q,0,0))*theta + gravity_csv + friction(dq)",
        "example_source_csv": source_files[int(source_id[0])],
    }
    for mode, models in metrics.items():
        report[mode] = {name: metric.report() for name, metric in models.items()}
        n = np.asarray(report[mode]["nominal"]["rmse_per_joint_nm"])
        i = np.asarray(report[mode]["identified"]["rmse_per_joint_nm"])
        report[mode]["rmse_improvement_percent_per_joint"] = ((n-i)/np.maximum(n, 1e-12)*100).tolist()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "test_metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    with (output_dir / "test_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["evaluation", "joint", "nominal_rmse_nm", "identified_rmse_nm", "improvement_percent"])
        for mode in metrics:
            for joint in range(7):
                writer.writerow([mode, f"J{joint + 1}",
                    report[mode]["nominal"]["rmse_per_joint_nm"][joint],
                    report[mode]["identified"]["rmse_per_joint_nm"][joint],
                    report[mode]["rmse_improvement_percent_per_joint"][joint]])
    example = {key: np.concatenate(value, axis=0) for key, value in example.items()}
    np.savez_compressed(output_dir / "example_torque_prediction.npz", **example)
    _plots(report, example, output_dir)
    return report


def _plots(report, example, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x, width = np.arange(7), 0.38
    for axis, mode in zip(axes, ("full_dynamics", "gravity_replaced")):
        axis.bar(x-width/2, report[mode]["nominal"]["rmse_per_joint_nm"], width, label="FR3 nominal")
        axis.bar(x+width/2, report[mode]["identified"]["rmse_per_joint_nm"], width, label="Identified")
        axis.set_xticks(x, [f"J{j+1}" for j in x])
        axis.set_ylabel("Torque RMSE [Nm]")
        axis.set_title("Full inverse dynamics" if mode == "full_dynamics" else "Measured gravity substituted")
        axis.grid(axis="y", alpha=0.25)
        axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "test_rmse_comparison.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for joint, axis in enumerate(axes):
        axis.plot(example["time"], example["measured"][:, joint], color="0.3", lw=0.8, label="Measured tau_J")
        for name, color in (("nominal", "b"), ("identified", "r")):
            axis.plot(example["time"], example[f"full_dynamics_{name}"][:, joint], color, lw=0.7, label=name)
        axis.set_ylabel(f"J{joint+1} [Nm]")
        axis.grid(alpha=0.25)
    axes[0].legend()
    axes[0].set_title("Full torque prediction: " + report["example_source_csv"])
    axes[-1].set_xlabel("Time [s]")
    figure.tight_layout()
    figure.savefig(output_dir / "example_torque_prediction.png", dpi=120)
    plt.close(figure)
