#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.dataset import prepare_package_data
from franka_7link_dynamics_id.paths import data_root


def main():
    parser = argparse.ArgumentParser(
        description="Convert the FR3 FFS robot-state CSVs into identification NPZ files."
    )
    parser.add_argument("--data-root", type=Path, default=None)
    parser.add_argument("--train-dir", type=Path, default=None)
    parser.add_argument("--test-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--window", type=int, default=21)
    parser.add_argument("--polyorder", type=int, default=3)
    parser.add_argument("--max-files", type=int, default=None, help="Limit files per split for a quick check.")
    args = parser.parse_args()

    root = args.data_root if args.data_root else data_root()
    prepare_package_data(
        root=root,
        stride=args.stride,
        window=args.window,
        polyorder=args.polyorder,
        train_dir=args.train_dir,
        test_dir=args.test_dir,
        output_dir=args.output_dir,
        max_files=args.max_files,
    )


if __name__ == "__main__":
    main()
