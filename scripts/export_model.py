#!/usr/bin/env python3
import argparse
from pathlib import Path

from franka_7link_dynamics_id.model_export import export_models
from franka_7link_dynamics_id.paths import model_config_path, package_root
from franka_7link_dynamics_id.estimation import load_config


def main():
    root = package_root()
    parser = argparse.ArgumentParser(description='Replace seven FR3 URDF inertials and export matching MuJoCo XMLs.')
    parser.add_argument('--parameters', type=Path, default=root / 'results/identification/identified_params.npz')
    parser.add_argument('--urdf', type=Path, default=root / 'models/fr3_nominal.urdf')
    parser.add_argument('--urdf-output', type=Path, default=root / 'models/generated/fr3_identified.urdf')
    parser.add_argument('--mjcf-dir', type=Path, default=root / 'models/generated')
    parser.add_argument('--config', type=Path, default=model_config_path())
    parser.add_argument('--link-prefix', default='fr3_link')
    args = parser.parse_args()
    paths = export_models(args.parameters, load_config(args.config), args.urdf,
                          args.urdf_output, args.mjcf_dir, args.link_prefix)
    for path in paths:
        print(f'Generated: {path}')


if __name__ == '__main__':
    main()
