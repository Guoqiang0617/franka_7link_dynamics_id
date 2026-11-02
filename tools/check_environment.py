#!/usr/bin/env python3
"""Check the active environment and the files needed by the offline workflow."""
import argparse
import csv
import importlib
import hashlib
import json
from pathlib import Path
import platform
import sys


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-data", action="store_true", help="Also check all manifest paths and file sizes.")
    parser.add_argument("--verify-hashes", action="store_true", help="Also read all packaged data and verify SHA-256 hashes.")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    failures, versions = [], {}
    for name in ("numpy", "scipy", "pandas", "yaml", "matplotlib", "pinocchio", "cvxpy", "mujoco", "franka_7link_dynamics_id"):
        try:
            module = importlib.import_module(name)
            versions[name] = getattr(module, "__version__", "unknown")
        except Exception as exc:
            failures.append(f"{name}: {exc}")
    for path in ("config/fr3_model.yaml", "config/identification.yaml", "models/fr3_nominal.urdf", "data/MANIFEST.csv",
                 "data/raw/train", "data/raw/test", "data/trajectories/test/trajectory_350_20260515_230214.json"):
        if not (ROOT / path).exists():
            failures.append(f"Missing project file/directory: {path}")
    if "cvxpy" in versions:
        import cvxpy as cp
        installed = cp.installed_solvers()
        if not set(installed).intersection(("CLARABEL", "SCS")):
            failures.append("cvxpy needs CLARABEL or SCS for semidefinite constraints.")
    if "pinocchio" in versions and "numpy" in versions:
        import numpy as np
        import pinocchio as pin
        try:
            model = pin.Model()
            model.addJoint(0, pin.JointModelRZ(), pin.SE3.Identity(), "test")
            regressor = pin.computeJointTorqueRegressor(model, model.createData(), np.zeros(1), np.zeros(1), np.zeros(1))
            if np.asarray(regressor).size != 10:
                failures.append(f"Unexpected Pinocchio regressor shape: {regressor.shape}")
        except Exception as exc:
            failures.append(f"Pinocchio regressor: {exc}")
    if (args.verify_data or args.verify_hashes) and (ROOT / "data/MANIFEST.csv").is_file():
        with (ROOT / "data/MANIFEST.csv").open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                path = ROOT / row["package_path"]
                if not path.is_file() or path.stat().st_size != int(row["size_bytes"]):
                    failures.append(f"Missing or incomplete copied data: {row['package_path']}")
                elif args.verify_hashes:
                    if not row.get('sha256'):
                        failures.append(f"No hash in manifest: {row['package_path']}")
                        continue
                    digest = hashlib.sha256()
                    with path.open('rb') as data_stream:
                        for block in iter(lambda: data_stream.read(8 * 1024 * 1024), b''):
                            digest.update(block)
                    if digest.hexdigest() != row['sha256']:
                        failures.append(f"SHA-256 mismatch: {row['package_path']}")
    report = {"python": sys.version, "platform": platform.platform(), "versions": versions,
              "success": not failures, "failures": failures}
    print(json.dumps(report, indent=2, ensure_ascii=True))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if failures:
        print("Activate franka-7link-id, then run: python -m pip install -e .", file=sys.stderr)
        return 1
    print("Environment and project files are ready.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
