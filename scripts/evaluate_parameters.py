#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.evaluation import evaluate_parameters
from franka_7link_dynamics_id.paths import model_config_path, package_root


def main():
    root = package_root()
    parser = argparse.ArgumentParser(description='Compare nominal and identified torque on held-out FR3 data.')
    parser.add_argument('--test', type=Path, default=root / 'data/processed/test.npz')
    parser.add_argument('--parameters', type=Path, default=root / 'results/identification/identified_params.npz')
    parser.add_argument('--config', type=Path, default=model_config_path())
    parser.add_argument('--output-dir', type=Path, default=root / 'results/evaluation')
    parser.add_argument('--batch-size', type=int, default=1000)
    parser.add_argument('--max-samples', type=int, default=None)
    args = parser.parse_args()
    result = evaluate_parameters(args.test, args.parameters, args.config,
                                 args.output_dir, args.batch_size, args.max_samples)
    for mode in ('full_dynamics', 'gravity_replaced'):
        print(f"{mode}: nominal {result[mode]['nominal']['rmse_overall_nm']:.5f} Nm, "
              f"identified {result[mode]['identified']['rmse_overall_nm']:.5f} Nm")


if __name__ == '__main__':
    main()
