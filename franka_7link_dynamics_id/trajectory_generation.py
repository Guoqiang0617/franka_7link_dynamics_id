from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from .paths import model_config_path, package_root


def load_generation_config(path: Path | None = None) -> tuple[dict, dict]:
    with Path(path or (package_root() / "config/ffs_generation.yaml")).open(
        "r", encoding="utf-8"
    ) as stream:
        trajectory = yaml.safe_load(stream)
    with model_config_path().open("r", encoding="utf-8") as stream:
        robot = yaml.safe_load(stream)
    if int(trajectory["num_harmonics"]) < 2 or int(trajectory["max_sampling_attempts"]) < 1:
        raise ValueError("Use at least two harmonics and one sampling attempt.")
    if not 0.0 < float(trajectory["safety_margin"]) <= 1.0:
        raise ValueError("safety_margin must lie in (0, 1].")
    for key in ("base_frequency", "duration", "sample_rate"):
        if not np.isfinite(trajectory[key]) or trajectory[key] <= 0:
            raise ValueError(f"{key} must be positive and finite.")
    cycles = trajectory["duration"] * trajectory["base_frequency"]
    if not np.isclose(cycles, round(cycles)):
        raise ValueError("duration must contain an integer number of Fourier periods for zero endpoint derivatives.")
    return trajectory, robot


def _evaluate_coefficients(initial_q, ak, bk, omega, time_s):
    time_s = np.asarray(time_s, dtype=float)
    harmonic = np.arange(1, ak.shape[1] + 1, dtype=float) * omega
    phase = time_s[:, None] * harmonic[None, :]
    sine, cosine = np.sin(phase), np.cos(phase)
    q = initial_q + (
        sine[:, :, None] * (ak.T / harmonic[:, None])[None, :, :]
    ).sum(axis=1) - (
        (cosine - 1.0)[:, :, None]
        * (bk.T / harmonic[:, None])[None, :, :]
    ).sum(axis=1)
    qd = (cosine[:, :, None] * ak.T[None, :, :] + sine[:, :, None] * bk.T[None, :, :]).sum(axis=1)
    qdd = (
        -sine[:, :, None] * (ak.T * harmonic[:, None])[None, :, :]
        + cosine[:, :, None] * (bk.T * harmonic[:, None])[None, :, :]
    ).sum(axis=1)
    return q, qd, qdd


def _valid_trajectory(ak, bk, settings, robot):
    initial_q = np.asarray(settings["initial_q"], dtype=float)
    safety = float(settings["safety_margin"])
    q_limits = np.asarray(robot["joint_position_limits"], dtype=float)
    q_min = initial_q + safety * (q_limits[:, 0] - initial_q)
    q_max = initial_q + safety * (q_limits[:, 1] - initial_q)
    qd_max = safety * np.asarray(robot["joint_velocity_limits"], dtype=float)
    qdd_max = safety * np.asarray(robot["joint_acceleration_limits"], dtype=float)
    time_s = np.linspace(0.0, float(settings["duration"]),
                         int(np.ceil(settings["duration"] * settings["sample_rate"])) + 1)
    omega = 2.0 * np.pi * float(settings["base_frequency"])
    q, qd, qdd = _evaluate_coefficients(initial_q, ak, bk, omega, time_s)
    return (
        np.all(q >= q_min)
        and np.all(q <= q_max)
        and np.all(np.abs(qd) <= qd_max)
        and np.all(np.abs(qdd) <= qdd_max)
    )


def _sample_coefficients(rng, settings, robot, coefficient_range):
    harmonics = int(settings["num_harmonics"])
    bound = np.asarray(coefficient_range, dtype=float).reshape(2)
    if not np.isfinite(bound).all() or bound[0] >= bound[1]:
        raise ValueError("Coefficient ranges must contain increasing finite bounds.")
    ak, bk = np.zeros((7, harmonics)), np.zeros((7, harmonics))
    times = np.linspace(0.0, settings["duration"], int(np.ceil(settings["duration"] * settings["sample_rate"])) + 1)
    kw = np.arange(1, harmonics + 1) * 2 * np.pi * settings["base_frequency"]
    sine, cosine = np.sin(times[:, None] * kw), np.cos(times[:, None] * kw)
    initial = np.asarray(settings["initial_q"], dtype=float)
    limits = np.asarray(robot["joint_position_limits"], dtype=float)
    if np.any(initial < limits[:, 0]) or np.any(initial > limits[:, 1]):
        raise ValueError("initial_q is outside the robot joint position limits.")
    safety = settings["safety_margin"]
    # Joint bounds are independent. Sampling each joint separately avoids discarding
    # six useful excitations when only one joint fails a bound.
    for joint in range(7):
        def valid(a, b):
            q = initial[joint] + sine @ (a / kw) - (cosine - 1) @ (b / kw)
            qd = cosine @ a + sine @ b
            qdd = -sine @ (a * kw) + cosine @ (b * kw)
            return (np.all(q >= initial[joint] + safety * (limits[joint, 0] - initial[joint]))
                    and np.all(q <= initial[joint] + safety * (limits[joint, 1] - initial[joint]))
                    and np.max(np.abs(qd)) <= safety * robot["joint_velocity_limits"][joint]
                    and np.max(np.abs(qdd)) <= safety * robot["joint_acceleration_limits"][joint])
        accepted = False
        for _ in range(int(settings["max_sampling_attempts"])):
            a, b = rng.uniform(*bound, size=harmonics), rng.uniform(*bound, size=harmonics)
            a[-1] = -a[:-1].sum()
            b[-1] = -np.dot(np.arange(1, harmonics), b[:-1]) / harmonics
            if valid(a, b):
                accepted = True
                break
        if not accepted:
            for scale in 0.9 ** np.arange(1, 51):
                if valid(a * scale, b * scale):
                    a, b, accepted = a * scale, b * scale, True
                    break
        if not accepted:
            raise RuntimeError(f"Could not generate a bounded trajectory for joint {joint+1}.")
        ak[joint], bk[joint] = a, b
    return ak, bk


def _generate_split(
    split,
    count,
    timestamp,
    settings,
    robot,
    rng,
    output_dir,
    coefficient_range,
):
    split_dir = output_dir / split
    split_dir.mkdir(parents=True, exist_ok=True)
    common = {
        **settings,
        "omega_f": 2.0 * np.pi * float(settings["base_frequency"]),
        "joint_position_min": np.asarray(robot["joint_position_limits"])[:, 0].tolist(),
        "joint_position_max": np.asarray(robot["joint_position_limits"])[:, 1].tolist(),
        "joint_velocity_max": robot["joint_velocity_limits"],
        "joint_acceleration_max": robot["joint_acceleration_limits"],
    }
    trajectories = []
    for index in range(1, count + 1):
        ak, bk = _sample_coefficients(rng, settings, robot, coefficient_range)
        trajectory = {
            "index": index,
            "type": split,
            "ak": ak.tolist(),
            "bk": bk.tolist(),
        }
        trajectories.append(trajectory)
        payload = {**common, **trajectory}
        path = split_dir / f"trajectory_{index:03d}_{timestamp}.json"
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return trajectories, common


def _check_compatible(old_summary, settings):
    for key in ("num_harmonics", "base_frequency", "duration"):
        if not np.isclose(float(old_summary[key]), float(settings[key])):
            raise ValueError(
                f"Existing summary {key}={old_summary[key]} differs from "
                f"the configured value {settings[key]}."
            )
    if not np.allclose(old_summary["initial_q"], settings["initial_q"]):
        raise ValueError("Existing summary initial_q differs from the configured start pose.")


def generate_ffs_dataset(
    output_dir: Path,
    num_train: int = 10,
    num_test: int = 5,
    seed: int = 42,
    generation_mode: str = "all",
    existing_summary: Path | None = None,
    config_path: Path | None = None,
) -> Path:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    timestamp = run_id[:15]
    output_dir = Path(output_dir) / run_id
    if num_train < 0 or num_test < 0 or num_train + num_test == 0:
        raise ValueError("Trajectory counts must be non-negative, with at least one trajectory.")
    settings, robot = load_generation_config(config_path)
    if generation_mode not in ("all", "train", "test"):
        raise ValueError("generation_mode must be all, train or test.")
    old = None
    if generation_mode != "all":
        if existing_summary is None or not Path(existing_summary).is_file():
            raise FileNotFoundError("train/test-only generation needs --existing-summary.")
        old = json.loads(Path(existing_summary).read_text(encoding="utf-8"))
        _check_compatible(old, settings)

    rng = np.random.default_rng(seed)
    output_dir.mkdir(parents=True, exist_ok=True)

    if generation_mode in ("all", "train"):
        train_trajectories, metadata = _generate_split(
            "train",
            num_train,
            timestamp,
            settings,
            robot,
            rng,
            output_dir,
            settings["train_coefficient_range"],
        )
    else:
        train_trajectories = old["train_trajectories"]
        metadata = {
            key: old[key]
            for key in (
                "num_harmonics",
                "base_frequency",
                "duration",
                "sample_rate",
                "initial_q",
                "joint_position_min",
                "joint_position_max",
                "joint_velocity_max",
                "joint_acceleration_max",
                "safety_margin",
            )
        }

    if generation_mode in ("all", "test"):
        test_trajectories, _ = _generate_split(
            "test",
            num_test,
            timestamp,
            settings,
            robot,
            rng,
            output_dir,
            settings["test_coefficient_range"],
        )
    else:
        test_trajectories = old["test_trajectories"]

    summary = {
        **metadata,
        "timestamp": timestamp,
        "run_id": run_id,
        "generation_mode": generation_mode,
        "source_summary_file": str(existing_summary) if existing_summary else "",
        "seed": int(seed),
        "num_train_trajectories": len(train_trajectories),
        "num_test_trajectories": len(test_trajectories),
        "train_coefficient_range": settings["train_coefficient_range"],
        "test_coefficient_range": settings["test_coefficient_range"],
        "train_trajectories": train_trajectories,
        "test_trajectories": test_trajectories,
        "constraint_checks": "sampled joint position, velocity and acceleration only; no collision checking",
    }
    # A train/test-only run still carries every JSON referenced by the combined summary.
    for split, trajectories in (("train", train_trajectories), ("test", test_trajectories)):
        directory = output_dir / split
        directory.mkdir(exist_ok=True)
        for trajectory in trajectories:
            path = directory / f"trajectory_{trajectory['index']:03d}_{timestamp}.json"
            if not path.exists():
                path.write_text(json.dumps({**metadata, **trajectory,
                    "omega_f": 2*np.pi*float(metadata["base_frequency"])}, indent=2), encoding="utf-8")
    summary_path = output_dir / f"ffs_summary_{timestamp}.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary_path
