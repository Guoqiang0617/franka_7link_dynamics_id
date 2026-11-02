#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.estimation import run_identification
from franka_7link_dynamics_id.paths import model_config_path, package_root


def main():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Fit the seven FR3 link inertias and seven joint friction models."
    )
    parser.add_argument("--train", type=Path, default=root / "data/processed/train.npz")
    parser.add_argument("--config", type=Path, default=model_config_path())
    parser.add_argument(
        "--output-dir", type=Path, default=root / "results/identification"
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=None,
        help="Evenly subsample across all prepared samples for a quick check.",
    )
    parser.add_argument("--max-iterations", type=int, default=None)
    parser.add_argument("--identification-config", type=Path, default=None)
    parser.add_argument("--solver", choices=("sdp", "projection"), default=None)
    args = parser.parse_args()
    report = run_identification(
        args.train, args.config, args.output_dir, args.max_samples, args.max_iterations,
        args.identification_config, args.solver,
    )
    print(
        f"Finished: rank {report['rank']}/84, nullity {report['nullity']}, "
        f"train RMSE {report['rmse_overall_nm']:.5f} Nm"
    )


if __name__ == "__main__":
    main()
