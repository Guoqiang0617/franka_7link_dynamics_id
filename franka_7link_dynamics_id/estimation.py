from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from .inertial import (
    cad_prior_vector,
    parameter_names,
    parameters_from_pseudo_inertia,
    pseudo_inertia,
)

N_DOF = 7
N_RIGID_PARAMETERS = 70
FRICTION_EPSILON = 0.05


def build_pinocchio_model(config: dict):
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise RuntimeError(
            "Pinocchio is required to compute the rigid-body torque regressor. "
            "Create and activate the environment from environment-windows.yml."
        ) from exc

    model = pin.Model()
    model.gravity.linear = np.asarray(
        config.get("gravity_vector", [0.0, 0.0, -9.81]), dtype=float
    )
    parent_id = 0
    for index, (alpha, a, d, theta_offset) in enumerate(config["mdh_parameters"]):
        placement = pin.SE3(
            pin.utils.rotate("x", alpha), np.array([a, 0.0, 0.0])
        ) * pin.SE3(
            pin.utils.rotate("z", theta_offset), np.array([0.0, 0.0, d])
        )
        joint_id = model.addJoint(
            parent_id, pin.JointModelRZ(), placement, f"joint_{index + 1}"
        )
        model.appendBodyToJoint(
            joint_id, pin.Inertia.FromSphere(1.0, 0.01), pin.SE3.Identity()
        )
        parent_id = joint_id
    return pin, model


def build_design_matrix(
    pin, model, q, qd, qdd, epsilon=FRICTION_EPSILON, report_progress=True
):
    q = np.asarray(q, dtype=float)
    qd = np.asarray(qd, dtype=float)
    qdd = np.asarray(qdd, dtype=float)
    if q.shape != qd.shape or q.shape != qdd.shape or q.ndim != 2 or q.shape[1] != 7:
        raise ValueError("q, qd and qdd must each have shape (samples, 7)")

    data = model.createData()
    n_samples = len(q)
    n_parameters = N_RIGID_PARAMETERS + 2 * N_DOF
    matrix = np.zeros((n_samples * N_DOF, n_parameters), dtype=np.float64)
    step = max(1, n_samples // 20)
    for sample in range(n_samples):
        matrix[sample * N_DOF : (sample + 1) * N_DOF, :N_RIGID_PARAMETERS] = (
            pin.computeJointTorqueRegressor(model, data, q[sample], qd[sample], qdd[sample])
        )
        if report_progress and sample and sample % step == 0:
            print(f"Regressor: {sample}/{n_samples} samples")

    rows = np.arange(n_samples * N_DOF).reshape(n_samples, N_DOF)
    for joint in range(N_DOF):
        matrix[rows[:, joint], N_RIGID_PARAMETERS + 2 * joint] = np.tanh(
            qd[:, joint] / epsilon
        )
        matrix[rows[:, joint], N_RIGID_PARAMETERS + 2 * joint + 1] = qd[:, joint]
    return matrix


def _project_to_physical(parameters: np.ndarray, min_eigenvalue: float) -> np.ndarray:
    projected = np.asarray(parameters, dtype=float).reshape(-1).copy()
    for joint in range(N_DOF):
        start = joint * 10
        matrix = pseudo_inertia(projected[start : start + 10])
        matrix = 0.5 * (matrix + matrix.T)
        values, vectors = np.linalg.eigh(matrix)
        matrix = vectors @ np.diag(np.maximum(values, min_eigenvalue)) @ vectors.T
        projected[start : start + 10] = parameters_from_pseudo_inertia(matrix)
    projected[N_RIGID_PARAMETERS:] = np.maximum(
        projected[N_RIGID_PARAMETERS:], 0.0
    )
    return projected


def _pseudo_inertia_basis() -> list[np.ndarray]:
    basis = []
    for index in range(10):
        parameter = np.zeros(10)
        parameter[index] = 1.0
        basis.append(pseudo_inertia(parameter))
    return basis


def _solve_sdp(hessian, gradient, min_eigenvalue):
    try:
        import cvxpy as cp
    except ImportError:
        return None, "cvxpy unavailable"

    variable = cp.Variable(hessian.shape[0])
    symmetric_hessian = 0.5 * (hessian + hessian.T)
    objective = 0.5 * cp.quad_form(
        variable, cp.psd_wrap(symmetric_hessian)
    ) - gradient @ variable
    constraints = []
    basis = _pseudo_inertia_basis()
    for joint in range(N_DOF):
        start = joint * 10
        inertia = sum(
            basis[index] * variable[start + index] for index in range(10)
        )
        constraints.append(inertia >> min_eigenvalue * np.eye(4))
    for joint in range(N_DOF):
        start = N_RIGID_PARAMETERS + 2 * joint
        constraints.append(variable[start : start + 2] >= 0.0)

    problem = cp.Problem(cp.Minimize(objective), constraints)
    failures = []
    for solver_name in ("CLARABEL", "SCS"):
        solver = getattr(cp, solver_name, None)
        if solver is None:
            continue
        try:
            options = (
                dict(tol_gap_abs=1e-9, tol_feas=1e-9, tol_gap_rel=1e-9, max_iter=300)
                if solver_name == "CLARABEL"
                else dict(eps=1e-7, max_iters=50000)
            )
            problem.solve(solver=solver, verbose=False, **options)
        except Exception as exc:
            failures.append(f"{solver_name}: {type(exc).__name__}: {exc}")
            continue
        if problem.status in ("optimal", "optimal_inaccurate") and variable.value is not None:
            values = np.asarray(variable.value, dtype=float).reshape(-1)
            minimum = min(
                np.linalg.eigvalsh(pseudo_inertia(values[j * 10 : (j + 1) * 10])).min()
                for j in range(N_DOF)
            )
            if np.isfinite(values).all() and minimum >= min_eigenvalue - 1e-8 and values[70:].min() >= -1e-8:
                # Remove only floating-point negative zeros in nonnegative friction variables.
                values[70:] = np.maximum(values[70:], 0.0)
                return values, f"{solver_name}: {problem.status}"
            failures.append(f"{solver_name}: constraint residual too large (pseudo inertia {minimum:.3e})")
        else:
            failures.append(f"{solver_name}: {problem.status}")
    return None, "; ".join(failures)


def _nullspace_regularizer(matrix, prior, data_weight, null_weight, threshold):
    normal = matrix.T @ matrix
    normal = 0.5 * (normal + normal.T)
    eigenvalues, eigenvectors = np.linalg.eigh(normal)
    maximum = max(float(eigenvalues.max()), 0.0)
    if maximum <= 0.0:
        null_mask = np.ones_like(eigenvalues, dtype=bool)
    else:
        null_mask = eigenvalues < (threshold**2) * maximum
    null_vectors = eigenvectors[:, null_mask]
    regularizer = data_weight * np.eye(matrix.shape[1])
    if null_vectors.size:
        regularizer += (null_weight - data_weight) * (
            null_vectors @ null_vectors.T
        )
    return regularizer, regularizer @ prior, eigenvalues, null_mask


def fit_pc_irls(
    matrix,
    target,
    prior,
    max_iterations=15,
    tolerance=1e-4,
    tukey_constant=4.685,
    data_weight=1e-4,
    null_weight=1.0,
    svd_threshold=1e-5,
    min_eigenvalue=1e-6,
    solver="sdp",
):
    matrix = np.asarray(matrix, dtype=float)
    target = np.asarray(target, dtype=float).reshape(-1)
    prior = np.asarray(prior, dtype=float).reshape(-1)
    if matrix.shape != (target.size, prior.size):
        raise ValueError(
            f"Matrix/target/prior shape mismatch: {matrix.shape}, "
            f"{target.shape}, {prior.shape}"
        )
    if not target.size or not all(np.isfinite(x).all() for x in (matrix, target, prior)):
        raise ValueError("Regressor, target and prior must be nonempty and finite.")
    if max_iterations < 1 or solver not in ("sdp", "projection"):
        raise ValueError("Use positive max_iterations and solver='sdp' or 'projection'.")

    regularizer, prior_term, eigenvalues, null_mask = _nullspace_regularizer(
        matrix, prior, data_weight, null_weight, svd_threshold
    )
    phi = prior.copy()
    history = []
    converged = False
    for iteration in range(max_iterations):
        residual = target - matrix @ phi
        scale = tukey_constant * np.median(np.abs(residual)) / 0.6745 + 1e-8
        weights = np.zeros_like(residual)
        inlier = np.abs(residual) <= scale
        ratio = residual[inlier] / scale
        weights[inlier] = (1.0 - ratio**2) ** 2

        # Accumulate small blocks so fitting the full data does not duplicate the large regressor.
        hessian = regularizer.copy()
        gradient = prior_term.copy()
        for begin in range(0, len(target), 20000):
            end = min(begin + 20000, len(target))
            block = matrix[begin:end]
            weighted_block = block * weights[begin:end, None]
            hessian += block.T @ weighted_block
            gradient += block.T @ (weights[begin:end] * target[begin:end])
        if solver == "sdp":
            candidate, status = _solve_sdp(hessian, gradient, min_eigenvalue)
            if candidate is None:
                raise RuntimeError(
                    f"Constrained PC-IRLS solve failed: {status}. "
                    "Check cvxpy/CLARABEL in the documented environment. "
                    "Projection is available only by explicitly selecting --solver projection."
                )
            solver_path = f"cvxpy SDP ({status})"
        else:
            candidate = np.linalg.lstsq(hessian, gradient, rcond=None)[0]
            candidate = _project_to_physical(candidate, min_eigenvalue)
            solver_path = "explicit PSD projection approximation"

        relative_change = np.linalg.norm(candidate - phi) / (
            np.linalg.norm(phi) + 1e-12
        )
        rmse = float(np.sqrt(np.mean((target - matrix @ candidate) ** 2)))
        print(
            f"PC-IRLS {iteration + 1}/{max_iterations}: "
            f"relative change={relative_change:.3e}, RMSE={rmse:.5f} Nm, "
            f"solver={solver_path}"
        )
        phi = candidate
        history.append({
            "iteration": iteration + 1,
            "relative_change": float(relative_change),
            "rmse_nm": rmse,
            "nonzero_weight_fraction": float(np.mean(weights > 0.0)),
            "solver": solver_path,
        })
        if relative_change < tolerance:
            converged = True
            break

    diagnostics = {
        "iterations": iteration + 1,
        "solver": solver_path,
        "rank": int((~null_mask).sum()),
        "nullity": int(null_mask.sum()),
        "condition_number": _condition_number(eigenvalues, null_mask),
        "rmse_nm": float(np.sqrt(np.mean((target - matrix @ phi) ** 2))),
        "null_mask": null_mask,
        "singular_values": np.sqrt(np.maximum(eigenvalues, 0.0)).tolist(),
        "converged": converged,
        "history": history,
    }
    return phi, diagnostics


def _condition_number(eigenvalues, null_mask) -> float:
    identifiable = eigenvalues[~null_mask]
    if not identifiable.size:
        return float("inf")
    return float(np.sqrt(max(eigenvalues.max(), 0.0) / max(identifiable.min(), 1e-30)))


def prediction_metrics(target, prediction) -> dict:
    target = np.asarray(target, dtype=float).reshape(-1, 7)
    prediction = np.asarray(prediction, dtype=float).reshape(-1, 7)
    error = prediction - target
    rmse = np.sqrt(np.mean(error**2, axis=0))
    mae = np.mean(np.abs(error), axis=0)
    total = np.sum((target - target.mean(axis=0)) ** 2, axis=0)
    residual = np.sum(error**2, axis=0)
    r2 = 1.0 - residual / np.maximum(total, 1e-12)
    return {
        "rmse_per_joint_nm": rmse.tolist(),
        "mae_per_joint_nm": mae.tolist(),
        "r2_per_joint": r2.tolist(),
        "rmse_overall_nm": float(np.sqrt(np.mean(error**2))),
        "mae_overall_nm": float(np.mean(np.abs(error))),
        "r2_mean": float(np.mean(r2)),
    }


def load_config(path: Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if len(config.get("mdh_parameters", [])) != N_DOF:
        raise ValueError("FR3 model config must define exactly seven MDH joints.")
    if len(config.get("cad_prior", {}).get("links", [])) != N_DOF:
        raise ValueError("FR3 model config must define exactly seven link priors.")
    armature = np.asarray(config.get("simulation_joint_armature", []), dtype=float)
    if (
        armature.shape != (N_DOF,)
        or not np.isfinite(armature).all()
        or np.any(armature < 0.0)
    ):
        raise ValueError(
            "FR3 model config must define seven non-negative simulation armature values."
        )
    return config


def run_identification(
    train_npz: Path,
    config_path: Path,
    output_dir: Path,
    max_samples: int | None = None,
    max_iterations: int | None = None,
    identification_config: Path | None = None,
    solver: str | None = None,
):
    train_npz = Path(train_npz)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config(config_path)
    settings_path = Path(identification_config) if identification_config else Path(config_path).with_name("identification.yaml")
    with settings_path.open("r", encoding="utf-8") as stream:
        settings = yaml.safe_load(stream)
    if max_iterations is not None:
        settings["max_iterations"] = max_iterations
    if solver is not None:
        settings["solver"] = solver
    with np.load(train_npz) as dataset:
        q = np.asarray(dataset["q"], dtype=float)
        qd = np.asarray(dataset["qd"], dtype=float)
        qdd = np.asarray(dataset["qdd"], dtype=float)
        tau = np.asarray(dataset["tau"], dtype=float)
        if max_samples is not None and max_samples > 0:
            selected = np.linspace(0, len(q) - 1, min(max_samples, len(q)), dtype=int)
            q, qd, qdd, tau = (array[selected] for array in (q, qd, qdd, tau))

    pin, model = build_pinocchio_model(config)
    print(f"Building 84-parameter regressor for {len(q)} samples.")
    matrix = build_design_matrix(pin, model, q, qd, qdd)
    prior = cad_prior_vector(config)
    parameters, diagnostics = fit_pc_irls(
        matrix, tau.reshape(-1), prior, **settings
    )
    predicted = matrix @ parameters
    metrics = prediction_metrics(tau, predicted.reshape(-1, 7))
    diagnostics.pop("null_mask")
    diagnostics.update(metrics)
    physical = []
    interpretation_notes = []
    from .inertial import inertial_origin_to_com

    for joint in range(7):
        values = parameters[joint * 10 : (joint + 1) * 10]
        mass, com, inertia_com = inertial_origin_to_com(values)
        cad = config['cad_prior']['links'][joint]
        mass_ratio = mass / cad['mass']
        if mass_ratio < 0.5 or mass_ratio > 2.0:
            interpretation_notes.append(
                f"fr3_link{joint+1}: fitted mass is {mass_ratio:.3g} times its nominal prior. "
                "This is a physically feasible parameter representative, not an independently measured link mass."
            )
        physical.append({
            "link": f"fr3_link{joint + 1}", "mass_kg": mass,
            "mass_ratio_to_nominal": mass_ratio,
            "com_m": com.tolist(), "inertia_com_kg_m2": inertia_com.tolist(),
            "pseudo_inertia_min_eigenvalue": float(np.linalg.eigvalsh(pseudo_inertia(values)).min()),
        })

    np.savez_compressed(
        output_dir / "identified_params.npz",
        phi_prior=prior,
        phi_identified=parameters,
        parameter_names=np.asarray(parameter_names(), dtype="U64"),
        friction_model=np.asarray("simple"),
        rank=np.asarray(diagnostics["rank"]),
        nullity=np.asarray(diagnostics["nullity"]),
        solver=np.asarray(diagnostics["solver"]),
        schema_version=np.asarray(1),
        parameter_layout=np.asarray("rigid70_friction14"),
        inertia_reference=np.asarray("link_origin"),
        model_config_sha256=np.asarray(hashlib.sha256(Path(config_path).read_bytes()).hexdigest()),
    )
    report = {
        "model": config["model"],
        "scope": "fr3_link1 through fr3_link7 rigid inertias plus joint friction",
        "training_file": str(train_npz.resolve()),
        "parameter_count": int(len(parameters)),
        "rigid_parameter_count": N_RIGID_PARAMETERS,
        "friction_parameter_count": 14,
        "training_sample_count": len(q),
        "training_target": "raw measured tau_J, including model gravity in the regressor",
        "settings": settings,
        "nominal_train_metrics": prediction_metrics(tau, (matrix @ prior).reshape(-1, 7)),
        "physical_links": physical,
        "interpretation_notes": interpretation_notes,
        **diagnostics,
    }
    (output_dir / "fit_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    with (output_dir / "parameter_table.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["parameter", "CAD_prior", "identified", "delta"])
        for name, cad, value in zip(parameter_names(), prior, parameters):
            writer.writerow([name, f"{cad:.12g}", f"{value:.12g}", f"{value - cad:.12g}"])

    return report
