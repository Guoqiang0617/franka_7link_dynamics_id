#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.estimation import load_config
from franka_7link_dynamics_id.paths import model_config_path, package_root
from franka_7link_dynamics_id.simulation import run_ffs_simulation


def main():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Compare nominal and identified FR3 models in MuJoCo."
    )
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument(
        "--parameters",
        type=Path,
        default=root / "results/identification/identified_params.npz",
    )
    parser.add_argument("--config", type=Path, default=model_config_path())
    parser.add_argument("--output-dir", type=Path, default=root / "results/simulation")
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--save-rate", type=float, default=1000.0)
    args = parser.parse_args()

    csv_path = args.csv
    if csv_path is None:
        candidate = root / "data/raw/test" / f"{args.trajectory.stem}.csv"
        if candidate.is_file():
            csv_path = candidate
    config = load_config(args.config)
    run_ffs_simulation(
        args.trajectory,
        args.parameters,
        config,
        args.output_dir,
        csv_path,
        args.timestep,
        args.save_rate,
    )


if __name__ == "__main__":
    main()
