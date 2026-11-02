from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .inertial import inertial_origin_to_com
from .trajectory import FFSTrajectory


SIM_K = np.array([1500.0, 1500.0, 1500.0, 1500.0, 900.0, 720.0, 360.0])
SIM_D = np.array([3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.0])
SIM_FRICTION_SMOOTHING = 0.4


def _format(values):
    return " ".join(f"{float(value):.12g}" for value in values)



def _load_mjcf_model(path: Path):
    """Load generated MJCF through text for Unicode Windows paths."""
    import mujoco

    path = Path(path)
    if any(ord(character) > 127 for character in str(path)):
        return mujoco.MjModel.from_xml_string(path.read_text(encoding="utf-8"))
    return mujoco.MjModel.from_xml_path(str(path))
def write_mjcf(path: Path, rigid_parameters, config: dict, model_name: str) -> Path:
    try:
        import mujoco  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("MuJoCo is required to generate and run the MJCF model.") from exc

    parameters = np.asarray(rigid_parameters, dtype=float).reshape(7, 10)
    limits = np.asarray(config["joint_position_limits"], dtype=float).reshape(7, 2)
    mdh = np.asarray(config["mdh_parameters"], dtype=float).reshape(7, 4)
    armature = np.asarray(config["simulation_joint_armature"], dtype=float).reshape(7)
    lines = [
        f'<mujoco model="{model_name}">',
        '  <compiler angle="radian" autolimits="true" balanceinertia="false"/>',
        '  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit" impratio="10"/>',
        '  <default>',
        '    <joint damping="0" frictionloss="0"/>',
        '    <geom contype="0" conaffinity="0" group="1" rgba="0.55 0.62 0.68 1"/>',
        '  </default>',
        '  <worldbody>',
        '    <light pos="0 0 3" dir="0 0 -1"/>',
        '    <body name="base">',
        '      <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>',
    ]
    for joint, (alpha, a, d, theta_offset) in enumerate(mdh):
        mass, com, inertia_com = inertial_origin_to_com(parameters[joint])
        lower, upper = limits[joint]
        position = [a, -d * np.sin(alpha), d * np.cos(alpha)]
        # Quaternion product Rx(alpha) * Rz(theta_offset), in MuJoCo w,x,y,z order.
        ca, sa = np.cos(alpha / 2), np.sin(alpha / 2)
        ct, st = np.cos(theta_offset / 2), np.sin(theta_offset / 2)
        quaternion = [
            ca * ct, sa * ct, -sa * st, ca * st,
        ]
        full = [
            inertia_com[0, 0],
            inertia_com[1, 1],
            inertia_com[2, 2],
            inertia_com[0, 1],
            inertia_com[0, 2],
            inertia_com[1, 2],
        ]
        lines.extend(
            [
                f'      <body name="fr3_link{joint + 1}" pos="{_format(position)}" quat="{_format(quaternion)}">',
                f'        <joint name="joint{joint + 1}" type="hinge" axis="0 0 1" '
                f'range="{lower:.12g} {upper:.12g}" '
                f'armature="{armature[joint]:.12g}"/>',
                f'        <inertial pos="{_format(com)}" mass="{mass:.12g}" fullinertia="{_format(full)}"/>',
                '        <geom type="capsule" fromto="0 0 -0.04 0 0 0.04" size="0.035"/>',
            ]
        )
    lines.extend("      </body>" for _ in range(7))
    lines.extend(
        [
            '    </body>',
            '  </worldbody>',
            '  <actuator>',
        ]
    )
    for joint in range(7):
        lines.append(
            f'    <motor name="motor{joint + 1}" joint="joint{joint + 1}" gear="1"/>'
        )
    lines.extend(["  </actuator>", "</mujoco>"])

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _simulate(model, trajectory: FFSTrajectory, friction, timestep, save_rate):
    import mujoco

    data = mujoco.MjData(model)
    data.qpos[:7] = trajectory.initial_q
    data.qvel[:7] = 0.0
    mujoco.mj_forward(model, data)
    dt = float(timestep)
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(save_rate) or save_rate <= 0:
        raise ValueError("timestep and save_rate must be positive finite numbers.")
    save_stride = max(1, int(round(1.0 / (save_rate * dt))))
    steps = int(np.ceil(trajectory.duration / dt))
    count = steps // save_stride + 1 + int(steps % save_stride != 0)
    result = {
        "time": np.zeros(count),
        "q": np.zeros((count, 7)),
        "dq": np.zeros((count, 7)),
        "q_d": np.zeros((count, 7)),
        "dq_d": np.zeros((count, 7)),
        "tau": np.zeros((count, 7)),
    }
    index = 0
    for step in range(steps + 1):
        time_s = min(step * dt, trajectory.duration)
        q_d, dq_d, _ = trajectory.evaluate([time_s])
        mujoco.mj_forward(model, data)
        error = q_d[0] - data.qpos[:7]
        velocity_error = dq_d[0] - data.qvel[:7]
        friction_torque = friction[:, 0] * np.tanh(
            data.qvel[:7] / SIM_FRICTION_SMOOTHING
        ) + friction[:, 1] * data.qvel[:7]
        torque = (
            SIM_K * error
            + SIM_D * velocity_error
            + data.qfrc_bias[:7]
            + friction_torque
        )
        # The plant's friction opposes motion; the controller compensates that same model.
        # Without this negative plant torque, positive compensation becomes artificial antidamping.
        data.qfrc_applied[:7] = -friction_torque
        data.ctrl[:7] = torque
        if not np.isfinite(torque).all():
            raise RuntimeError(f"Non-finite control torque at t={time_s:.6f} s.")
        if step % save_stride == 0 or step == steps:
            if index < count:
                result["time"][index] = time_s
                result["q"][index] = data.qpos[:7]
                result["dq"][index] = data.qvel[:7]
                result["q_d"][index] = q_d[0]
                result["dq_d"][index] = dq_d[0]
                result["tau"][index] = torque
                index += 1
        if step < steps:
            mujoco.mj_step(model, data)
            bad = (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                   mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADCTRL)
            if any(data.warning[int(code)].number for code in bad) or not np.isfinite(data.qpos).all():
                raise RuntimeError(f"MuJoCo became unstable at t={time_s:.6f} s; no success plot was generated.")
    return {key: value[:index] for key, value in result.items()}


def _load_experiment_csv(path):
    import pandas as pd

    columns = ["ros_time_ns"] + [f"{signal}_{joint}" for signal in ("q", "dq", "q_d", "tau_J") for joint in range(7)]
    frame = pd.read_csv(path, usecols=columns)
    time_ns = frame["ros_time_ns"].to_numpy(dtype=np.int64)
    time_s = (time_ns - time_ns[0]).astype(np.float64) * 1e-9
    if len(time_s) < 2 or np.any(np.diff(time_s) <= 0):
        raise ValueError("Experimental CSV needs at least two strictly increasing timestamps.")
    return {
        "time": time_s,
        "q": frame[[f"q_{j}" for j in range(7)]].to_numpy(dtype=float),
        "dq": frame[[f"dq_{j}" for j in range(7)]].to_numpy(dtype=float),
        "q_d": frame[[f"q_d_{j}" for j in range(7)]].to_numpy(dtype=float),
        "tau": frame[[f"tau_J_{j}" for j in range(7)]].to_numpy(dtype=float),
    }


def _plot_results(nominal, identified, experiment, output_dir):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(7, 4, figsize=(19, 18), sharex="col")
    for joint in range(7):
        panels = axes[joint]
        panels[0].plot(nominal["time"], nominal["q_d"][:, joint], "k--", lw=0.8, label="FFS desired")
        if experiment is not None:
            panels[0].plot(experiment["time"], experiment["q"][:, joint], color="0.55", lw=0.6, label="Measured")
        panels[0].plot(nominal["time"], nominal["q"][:, joint], "b-", lw=0.7, label="Nominal")
        panels[0].plot(identified["time"], identified["q"][:, joint], "r-", lw=0.7, label="Identified")
        panels[0].set_ylabel(f"J{joint + 1} q [rad]")

        if experiment is not None:
            panels[1].plot(experiment["time"], experiment["dq"][:, joint], color="0.55", lw=0.6)
        panels[1].plot(nominal["time"], nominal["dq"][:, joint], "b-", lw=0.7)
        panels[1].plot(identified["time"], identified["dq"][:, joint], "r-", lw=0.7)
        panels[1].set_ylabel("dq [rad/s]")

        if experiment is not None:
            panels[2].plot(experiment["time"], experiment["tau"][:, joint], color="0.55", lw=0.6, label="Measured")
        panels[2].plot(nominal["time"], nominal["tau"][:, joint], "b-", lw=0.7, label="Nominal")
        panels[2].plot(identified["time"], identified["tau"][:, joint], "r-", lw=0.7, label="Identified")
        panels[2].set_ylabel("tau [Nm]")

        nominal_error = nominal["q"][:, joint] - nominal["q_d"][:, joint]
        identified_error = identified["q"][:, joint] - identified["q_d"][:, joint]
        if experiment is not None:
            panels[3].plot(
                experiment["time"],
                experiment["q"][:, joint] - experiment["q_d"][:, joint],
                color="0.55",
                lw=0.6,
                label="Measured",
            )
        panels[3].plot(nominal["time"], nominal_error, "b-", lw=0.7, label="Nominal")
        panels[3].plot(identified["time"], identified_error, "r-", lw=0.7, label="Identified")
        panels[3].axhline(0.0, color="k", lw=0.4, ls=":")
        panels[3].set_ylabel("error [rad]")
        for panel in panels:
            panel.grid(alpha=0.25)
        if joint == 0:
            for panel in (panels[0], panels[2], panels[3]):
                panel.legend(loc="upper right", fontsize=7)
    for axis in axes[-1]:
        axis.set_xlabel("Time [s]")
    figure.suptitle("FR3 FFS closed-loop simulation: nominal vs identified dynamics")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.975))
    figure.savefig(output_dir / "ffs_simulation_comparison.png", dpi=140)
    plt.close(figure)

    zoom, zoom_axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    zoom_metrics = {}
    for joint, axis in enumerate(zoom_axes):
        nominal_error = nominal["q"][:, joint] - nominal["q_d"][:, joint]
        identified_error = identified["q"][:, joint] - identified["q_d"][:, joint]
        axis.plot(nominal["time"], nominal_error, "b-", lw=0.8, label="Nominal")
        axis.plot(identified["time"], identified_error, "r-", lw=0.8, label="Identified")
        axis.set_ylabel(f"J{joint + 1} [rad]")
        axis.grid(alpha=0.25)
        axis.set_title(
            f"RMSE nominal {np.sqrt(np.mean(nominal_error**2)):.5f} rad | "
            f"identified {np.sqrt(np.mean(identified_error**2)):.5f} rad",
            loc="left",
            fontsize=9,
        )
        zoom_metrics[f"J{joint + 1}"] = {
            "nominal_rmse_rad": float(np.sqrt(np.mean(nominal_error**2))),
            "identified_rmse_rad": float(np.sqrt(np.mean(identified_error**2))),
        }
    zoom_axes[0].legend(loc="upper right")
    zoom_axes[-1].set_xlabel("Time [s]")
    zoom.tight_layout()
    zoom.savefig(output_dir / "error_zoom.png", dpi=120)
    plt.close(zoom)
    return zoom_metrics


def run_ffs_simulation(
    trajectory_json: Path,
    parameter_file: Path,
    config: dict,
    output_dir: Path,
    csv_path: Path | None = None,
    timestep: float = 0.001,
    save_rate: float = 1000.0,
):
    import mujoco

    trajectory_json = Path(trajectory_json)
    output_dir = Path(output_dir)
    from .parameter_io import load_parameter_vectors
    prior, identified = load_parameter_vectors(parameter_file, config)
    trajectory = FFSTrajectory(trajectory_json)
    model_dir = output_dir / "models"
    from .model_export import export_models
    from .paths import package_root
    nominal_xml, identified_xml = export_models(
        parameter_file, config, package_root() / "models/fr3_nominal.urdf",
        model_dir / "fr3_identified.urdf", model_dir,
    )
    friction = identified[70:84].reshape(7, 2)

    nominal_model = _load_mjcf_model(nominal_xml)
    nominal_model.opt.timestep = timestep
    identified_model = _load_mjcf_model(identified_xml)
    identified_model.opt.timestep = timestep
    nominal = _simulate(nominal_model, trajectory, friction, timestep, save_rate)
    identified_result = _simulate(
        identified_model, trajectory, friction, timestep, save_rate
    )
    np.savez_compressed(output_dir / "simulation_nominal.npz", **nominal)
    np.savez_compressed(output_dir / "simulation_identified.npz", **identified_result)

    experiment = _load_experiment_csv(csv_path) if csv_path else None
    tracking = _plot_results(nominal, identified_result, experiment, output_dir)
    result = {
        "trajectory": str(trajectory_json.resolve()),
        "experimental_csv": str(Path(csv_path).resolve()) if csv_path else None,
        "controller": {
            "K": SIM_K.tolist(),
            "D": SIM_D.tolist(),
            "friction_tanh_width_rad_s": SIM_FRICTION_SMOOTHING,
            "timestep_s": timestep,
            "fixed_joint_armature_kg_m2": np.asarray(
                config["simulation_joint_armature"], dtype=float
            ).tolist(),
            "plant_friction": "opposing torque, matched by compensation in both models",
            "bias_compensation": "each model's gravity plus Coriolis/centrifugal bias",
        },
        "tracking_rmse": tracking,
        "models": {
            "nominal": str(nominal_xml.resolve()),
            "identified": str(identified_xml.resolve()),
        },
    }
    if experiment is not None:
        result["comparison_with_measured"] = {}
        for name, simulation in (("nominal", nominal), ("identified", identified_result)):
            mask = ((simulation["time"] >= experiment["time"][0]) &
                    (simulation["time"] <= experiment["time"][-1]))
            if not np.any(mask):
                raise ValueError("No overlapping time interval between simulation and CSV.")
            t = simulation["time"][mask]
            comparison = {"overlap_sample_count": int(mask.sum())}
            for signal in ("q", "dq", "tau"):
                actual = np.column_stack([np.interp(t, experiment["time"], experiment[signal][:, j]) for j in range(7)])
                comparison[f"{signal}_rmse_per_joint"] = np.sqrt(np.mean((simulation[signal][mask]-actual)**2,axis=0)).tolist()
            result["comparison_with_measured"][name] = comparison
    (output_dir / "simulation_metrics.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(f"Saved comparison figure: {output_dir / 'ffs_simulation_comparison.png'}")
    return result
