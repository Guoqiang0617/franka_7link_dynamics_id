from __future__ import annotations

import numpy as np


def six_to_matrix(values) -> np.ndarray:
    xx, xy, yy, xz, yz, zz = np.asarray(values, dtype=float)
    return np.array(
        [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]], dtype=float
    )


def matrix_to_six(matrix) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    return np.array(
        [matrix[0, 0], matrix[0, 1], matrix[1, 1],
         matrix[0, 2], matrix[1, 2], matrix[2, 2]],
        dtype=float,
    )


def parallel_axis_matrix(com) -> np.ndarray:
    com = np.asarray(com, dtype=float).reshape(3)
    return float(com @ com) * np.eye(3) - np.outer(com, com)


def inertial_com_to_origin(mass: float, com, inertia_com) -> np.ndarray:
    """Return inertia about the link origin from a COM inertia tensor."""
    return six_to_matrix(inertia_com) + float(mass) * parallel_axis_matrix(com)


def inertial_origin_to_com(parameters) -> tuple[float, np.ndarray, np.ndarray]:
    """Decode [m, m*c, I_origin] into mass, COM and COM inertia."""
    values = np.asarray(parameters, dtype=float).reshape(10)
    if not np.isfinite(values).all():
        raise ValueError("Rigid-body parameters must all be finite.")
    mass = float(values[0])
    if mass <= 0.0:
        raise ValueError(f"Rigid-body mass must be positive, got {mass:g}")
    com = values[1:4] / mass
    inertia_origin = six_to_matrix(values[4:10])
    inertia_com = inertia_origin - mass * parallel_axis_matrix(com)
    inertia_com = 0.5 * (inertia_com + inertia_com.T)
    eigenvalues = np.linalg.eigvalsh(inertia_com)
    if eigenvalues.min() < -1e-8:
        raise ValueError(
            "The identified COM inertia is not positive semidefinite "
            f"(minimum eigenvalue {eigenvalues.min():.3e})."
        )
    pseudo_minimum = float(np.linalg.eigvalsh(pseudo_inertia(values)).min())
    if pseudo_minimum < -1e-8:
        raise ValueError(
            "Rigid-body inertia violates the physical triangle inequalities "
            f"(minimum pseudo-inertia eigenvalue {pseudo_minimum:.3e})."
        )
    return mass, com, inertia_com


def rigid_parameters_from_config(link: dict) -> np.ndarray:
    mass = float(link["mass"])
    com = np.asarray(link["com"], dtype=float).reshape(3)
    inertia_origin = inertial_com_to_origin(
        mass, com, link["inertia_com"]
    )
    return np.concatenate(
        ([mass], mass * com, matrix_to_six(inertia_origin))
    )


def pseudo_inertia(parameters) -> np.ndarray:
    """Construct the 4x4 pseudo-inertia matrix from link-origin parameters."""
    values = np.asarray(parameters, dtype=float).reshape(10)
    mass = values[0]
    h = values[1:4]
    inertia = six_to_matrix(values[4:10])
    sigma = 0.5 * np.trace(inertia) * np.eye(3) - inertia
    out = np.zeros((4, 4), dtype=float)
    out[:3, :3] = sigma
    out[:3, 3] = h
    out[3, :3] = h
    out[3, 3] = mass
    return out


def parameters_from_pseudo_inertia(matrix) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float).reshape(4, 4)
    mass = matrix[3, 3]
    h = matrix[:3, 3]
    sigma = matrix[:3, :3]
    inertia = np.trace(sigma) * np.eye(3) - sigma
    return np.concatenate(([mass], h, matrix_to_six(inertia)))


def cad_prior_vector(config: dict, friction_model: str = "simple") -> np.ndarray:
    if friction_model != "simple":
        raise ValueError("This package currently supports the simple friction model only.")
    rigid = np.concatenate(
        [rigid_parameters_from_config(link) for link in config["cad_prior"]["links"]]
    )
    friction = np.asarray(config["friction_simple"], dtype=float).reshape(14)
    return np.concatenate((rigid, friction))


def parameter_names(friction_model: str = "simple") -> list[str]:
    names = []
    for joint in range(1, 8):
        names.extend(
            [
                f"link{joint}_mass",
                f"link{joint}_h_x",
                f"link{joint}_h_y",
                f"link{joint}_h_z",
                f"link{joint}_Ixx_O",
                f"link{joint}_Ixy_O",
                f"link{joint}_Iyy_O",
                f"link{joint}_Ixz_O",
                f"link{joint}_Iyz_O",
                f"link{joint}_Izz_O",
            ]
        )
    if friction_model == "simple":
        for joint in range(1, 8):
            names.extend([f"joint{joint}_mu_c", f"joint{joint}_mu_v"])
    return names
