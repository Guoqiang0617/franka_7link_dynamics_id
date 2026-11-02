from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

from .paths import data_root


REQUIRED_COLUMNS = (
    ["ros_time_ns", "period_ns"]
    + [f"q_{j}" for j in range(7)]
    + [f"dq_{j}" for j in range(7)]
    + [f"tau_J_{j}" for j in range(7)]
    + [f"gravity_{j}" for j in range(7)]
)


def _read_csv(path: Path) -> pd.DataFrame:
    header = pd.read_csv(path, nrows=0).columns
    missing = [column for column in REQUIRED_COLUMNS if column not in header]
    if missing:
        raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
    return pd.read_csv(path, usecols=REQUIRED_COLUMNS)


def _differentiate_velocity(time_s: np.ndarray, dq: np.ndarray, window: int, polyorder: int):
    if len(time_s) < 3:
        raise ValueError("Each trajectory needs at least three samples to estimate acceleration.")
    delta = np.diff(time_s)
    if not np.all(np.isfinite(delta)) or np.any(delta <= 0.0):
        raise ValueError("ros_time_ns must be finite and strictly increasing.")
    dt = float(np.median(delta))
    qdd = np.gradient(dq, time_s, axis=0, edge_order=2)

    usable_window = min(window, len(qdd) if len(qdd) % 2 else len(qdd) - 1)
    if usable_window > polyorder and usable_window >= 5:
        qdd = savgol_filter(
            qdd, usable_window, polyorder, axis=0, mode="interp"
        )
    return qdd, dt


def prepare_split(
    input_dir: Path,
    output_path: Path,
    stride: int = 10,
    window: int = 21,
    polyorder: int = 3,
    max_files: int | None = None,
) -> dict:
    input_dir = Path(input_dir)
    output_path = Path(output_path)
    if stride < 1:
        raise ValueError("stride must be at least 1")
    if window < 3 or window % 2 == 0:
        raise ValueError("window must be an odd integer of at least 3")
    if polyorder < 1 or polyorder >= window:
        raise ValueError("polyorder must be positive and less than window")

    files = sorted(input_dir.glob("trajectory_*.csv"))
    if max_files is not None:
        files = files[:max_files]
    if not files:
        raise FileNotFoundError(f"No trajectory_*.csv files found in {input_dir}")

    chunks = {name: [] for name in ("q", "qd", "qdd", "tau", "gravity", "time")}
    source_ids = []
    source_names = []
    source_counts = []
    skipped = []

    for path in files:
        frame = _read_csv(path)
        if len(frame) < 3:
            skipped.append({"file": path.name, "raw_samples": len(frame),
                            "reason": "fewer than three samples; acceleration cannot be estimated"})
            print(f"Skipped {path.name}: only {len(frame)} samples", flush=True)
            continue
        source_id = len(source_names)
        ros_time = frame["ros_time_ns"].to_numpy(dtype=np.int64)
        time_s = (ros_time - ros_time[0]).astype(np.float64) * 1e-9
        q = frame[[f"q_{j}" for j in range(7)]].to_numpy(dtype=np.float64)
        qd = frame[[f"dq_{j}" for j in range(7)]].to_numpy(dtype=np.float64)
        tau = frame[[f"tau_J_{j}" for j in range(7)]].to_numpy(dtype=np.float64)
        gravity = frame[[f"gravity_{j}" for j in range(7)]].to_numpy(dtype=np.float64)
        if not all(np.isfinite(array).all() for array in (time_s, q, qd, tau, gravity)):
            raise ValueError(f"{path}: non-finite time or signal value")

        qdd, dt = _differentiate_velocity(time_s, qd, window, polyorder)
        selected = np.arange(0, len(time_s), stride)
        for name, values in (
            ("q", q),
            ("qd", qd),
            ("qdd", qdd),
            ("tau", tau),
            ("gravity", gravity),
            ("time", time_s),
        ):
            chunks[name].append(values[selected])
        source_ids.append(np.full(len(selected), source_id, dtype=np.int32))
        source_names.append(path.name)
        source_counts.append(
            {
                "file": path.name,
                "raw_samples": int(len(time_s)),
                "prepared_samples": int(len(selected)),
                "median_dt_s": dt,
                "max_dt_s": float(np.diff(time_s).max()),
                "duration_s": float(time_s[-1]),
            }
        )
        print(
            f"{path.name}: {len(time_s)} raw -> {len(selected)} samples "
            f"(median dt {dt * 1000:.3f} ms)"
        )

    if not source_names:
        raise ValueError(f"No CSV in {input_dir} contained at least three valid samples.")
    arrays = {
        name: np.concatenate(values, axis=0) for name, values in chunks.items()
    }
    arrays["source_id"] = np.concatenate(source_ids)
    arrays["source_files"] = np.asarray(source_names, dtype="U128")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path, **arrays)

    report = {
        "input_dir": str(input_dir.resolve()),
        "output_file": str(output_path.resolve()),
        "files": source_counts,
        "input_file_count": len(files),
        "raw_file_count": len(source_names),
        "skipped_files": skipped,
        "prepared_sample_count": int(len(arrays["q"])),
        "stride": stride,
        "savgol_window": window,
        "savgol_polyorder": polyorder,
        "target_torque_column": "tau_J",
        "gravity_column_is_retained_for_test_evaluation": True,
        "acceleration_method": "gradient(dq, measured_time), then Savitzky-Golay smoothing",
    }
    report_path = output_path.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Saved {len(arrays['q'])} samples to {output_path}")
    return report


def prepare_package_data(
    root: Path | None = None,
    stride: int = 10,
    window: int = 21,
    polyorder: int = 3,
    train_dir: Path | None = None,
    test_dir: Path | None = None,
    output_dir: Path | None = None,
    max_files: int | None = None,
) -> tuple[dict, dict]:
    root = Path(root) if root is not None else data_root()
    train_dir = Path(train_dir) if train_dir else root / "raw" / "train"
    test_dir = Path(test_dir) if test_dir else root / "raw" / "test"
    output_dir = Path(output_dir) if output_dir else root / "processed"
    train = prepare_split(
        train_dir, output_dir / "train.npz", stride, window, polyorder, max_files
    )
    test = prepare_split(
        test_dir, output_dir / "test.npz", stride, window, polyorder, max_files
    )
    return train, test
