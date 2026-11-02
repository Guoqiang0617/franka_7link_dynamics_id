#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.paths import package_root
from franka_7link_dynamics_id.trajectory_generation import generate_ffs_dataset


def main():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Generate bounded FR3 finite Fourier series trajectory sets."
    )
    parser.add_argument("--output-dir", type=Path, default=root / "data/trajectories/generated")
    parser.add_argument("--num-train", type=int, default=10)
    parser.add_argument("--num-test", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--mode", choices=("all", "train", "test"), default="all")
    parser.add_argument("--existing-summary", type=Path, default=None)
    parser.add_argument("--config", type=Path, default=root / "config/ffs_generation.yaml")
    args = parser.parse_args()
    summary = generate_ffs_dataset(
        args.output_dir,
        args.num_train,
        args.num_test,
        args.seed,
        args.mode,
        args.existing_summary,
        args.config,
    )
    print(f"Saved FFS trajectory summary: {summary}")


if __name__ == "__main__":
    main()
