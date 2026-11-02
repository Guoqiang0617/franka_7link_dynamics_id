#!/usr/bin/env python3
"""
FFS (Finite Fourier Series) Trajectory Generator

Generates FFS trajectories for Franka robot identification experiments.
Saves ak, bk parameters and trajectory metadata to JSON files.

Supported generation modes:
  - all:   regenerate both train and test trajectories.
  - train: regenerate train trajectories and keep test trajectories from an existing summary.
  - test:  regenerate test trajectories and keep train trajectories from an existing summary.

FFS formulas:
  Position: q_j(t) = q_j0 + Σ_{k=1}^{N} [ a_{j,k}/(k·ω_f)·sin(k·ω_f·t)
                                           - b_{j,k}/(k·ω_f)·(cos(k·ω_f·t) - 1) ]
  Velocity: dq_j(t) = Σ_{k=1}^{N} [ a_{j,k}·cos(k·ω_f·t) + b_{j,k}·sin(k·ω_f·t) ]
  Acceleration: ddq_j(t) = Σ_{k=1}^{N} [ -a_{j,k}·k·ω_f·sin(k·ω_f·t)
                                          + b_{j,k}·k·ω_f·cos(k·ω_f·t) ]
"""

import json
import os
import shutil
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node


class FFSTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('ffs_trajectory_generator')

        # Declare parameters
        self.declare_parameter('num_harmonics', 5)
        self.declare_parameter('base_frequency', 0.1)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('sample_rate', 1000.0)
        self.declare_parameter('num_train_trajectories', 10)
        self.declare_parameter('num_test_trajectories', 5)
        self.declare_parameter(
            'output_base_dir',
            os.path.join(os.path.expanduser('~'), 'franka_7link_dynamics_id_runs', 'trajectories'))
        self.declare_parameter('initial_q', [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])
        self.declare_parameter('joint_position_min',
                               [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
        self.declare_parameter('joint_position_max',
                               [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
        self.declare_parameter('joint_velocity_max',
                               [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.declare_parameter('joint_acceleration_max',
                               [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        self.declare_parameter('safety_margin', 0.8)
        self.declare_parameter('train_ak_range', [-0.5, 0.5] * 7)
        self.declare_parameter('train_bk_range', [-0.5, 0.5] * 7)
        self.declare_parameter('test_ak_range', [-0.8, 0.8] * 7)
        self.declare_parameter('test_bk_range', [-0.8, 0.8] * 7)
        self.declare_parameter('max_sampling_attempts', 1000)

        # New workflow parameters
        self.declare_parameter('generation_mode', 'all')
        self.declare_parameter('existing_summary_file', '')
        self.declare_parameter('output_summary_file', '')
        self.declare_parameter('backup_existing_summary', True)

        # Get parameters
        self.num_harmonics = self.get_parameter('num_harmonics').value
        self.base_frequency = self.get_parameter('base_frequency').value
        self.duration = self.get_parameter('duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.num_train = self.get_parameter('num_train_trajectories').value
        self.num_test = self.get_parameter('num_test_trajectories').value
        self.output_base_dir = self.get_parameter('output_base_dir').value
        self.initial_q = np.array(self.get_parameter('initial_q').value)
        self.joint_pos_min = np.array(self.get_parameter('joint_position_min').value)
        self.joint_pos_max = np.array(self.get_parameter('joint_position_max').value)
        self.joint_vel_max = np.array(self.get_parameter('joint_velocity_max').value)
        self.joint_acc_max = np.array(self.get_parameter('joint_acceleration_max').value)
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_attempts = self.get_parameter('max_sampling_attempts').value

        self.generation_mode = self.get_parameter('generation_mode').value.lower()
        self.existing_summary_file = self.get_parameter('existing_summary_file').value
        self.output_summary_file = self.get_parameter('output_summary_file').value
        self.backup_existing_summary = self.get_parameter('backup_existing_summary').value

        if self.generation_mode not in ('all', 'train', 'test'):
            raise ValueError("generation_mode must be one of: all, train, test")

        # Parse range parameters (nested lists)
        self.train_ak_range = self._parse_range_param('train_ak_range')
        self.train_bk_range = self._parse_range_param('train_bk_range')
        self.test_ak_range = self._parse_range_param('test_ak_range')
        self.test_bk_range = self._parse_range_param('test_bk_range')

        self.omega_f = 2.0 * np.pi * self.base_frequency
        self.num_joints = 7

        # Apply safety margin to limits
        self.safe_pos_min = self.initial_q + self.safety_margin * (self.joint_pos_min - self.initial_q)
        self.safe_pos_max = self.initial_q + self.safety_margin * (self.joint_pos_max - self.initial_q)
        self.safe_vel_max = self.safety_margin * self.joint_vel_max
        self.safe_acc_max = self.safety_margin * self.joint_acc_max

    def _parse_range_param(self, param_name):
        """Parse range parameter - handles both flat list and nested list formats."""
        raw = self.get_parameter(param_name).value
        if isinstance(raw[0], list):
            return np.array(raw)
        else:
            # Flat list: reshape to [7, 2]
            return np.array(raw).reshape(self.num_joints if hasattr(self, 'num_joints') else 7, 2)

    def compute_position(self, ak, bk, t):
        """Compute FFS position for all joints at time t.
        q_j(t) = q_j0 + Σ [ a_{j,k}/(k·ω_f)·sin(k·ω_f·t) - b_{j,k}/(k·ω_f)·(cos(k·ω_f·t)-1) ]
        """
        q = self.initial_q.copy()
        for k in range(self.num_harmonics):
            k_omega = (k + 1) * self.omega_f
            q += ak[:, k] / k_omega * np.sin(k_omega * t) \
                 - bk[:, k] / k_omega * (np.cos(k_omega * t) - 1.0)
        return q

    def compute_velocity(self, ak, bk, t):
        """Compute FFS velocity for all joints at time t.
        dq_j(t) = Σ [ a_{j,k}·cos(k·ω_f·t) + b_{j,k}·sin(k·ω_f·t) ]
        """
        dq = np.zeros(self.num_joints)
        for k in range(self.num_harmonics):
            k_omega = (k + 1) * self.omega_f
            dq += ak[:, k] * np.cos(k_omega * t) + bk[:, k] * np.sin(k_omega * t)
        return dq

    def compute_acceleration(self, ak, bk, t):
        """Compute FFS acceleration for all joints at time t.
        ddq_j(t) = Σ [ -a_{j,k}·k·ω_f·sin(k·ω_f·t) + b_{j,k}·k·ω_f·cos(k·ω_f·t) ]
        """
        ddq = np.zeros(self.num_joints)
        for k in range(self.num_harmonics):
            k_omega = (k + 1) * self.omega_f
            ddq += -ak[:, k] * k_omega * np.sin(k_omega * t) \
                   + bk[:, k] * k_omega * np.cos(k_omega * t)
        return ddq

    def check_constraints(self, ak, bk):
        """Check if the trajectory satisfies all joint constraints.
        Evaluates at dense time steps over the full duration.
        """
        dt = 1.0 / self.sample_rate
        t_array = np.arange(0, self.duration, dt)

        for t in t_array:
            q = self.compute_position(ak, bk, t)
            dq = self.compute_velocity(ak, bk, t)
            ddq = self.compute_acceleration(ak, bk, t)

            # Check position limits
            if np.any(q < self.safe_pos_min) or np.any(q > self.safe_pos_max):
                return False

            # Check velocity limits
            if np.any(np.abs(dq) > self.safe_vel_max):
                return False

            # Check acceleration limits
            if np.any(np.abs(ddq) > self.safe_acc_max):
                return False

        return True

    def sample_trajectory(self, ak_range, bk_range):
        """Sample ak, bk parameters using rejection sampling.
        Returns (ak, bk) that satisfy all constraints, or scaled version if max attempts reached.
        """
        ak = np.zeros((self.num_joints, self.num_harmonics))
        bk = np.zeros((self.num_joints, self.num_harmonics))

        for attempt in range(self.max_attempts):
            # Sample ak and bk from uniform distribution
            ak = np.zeros((self.num_joints, self.num_harmonics))
            bk = np.zeros((self.num_joints, self.num_harmonics))

            for j in range(self.num_joints):
                # Sample the first N-1 harmonics
                ak[j, :-1] = np.random.uniform(ak_range[j, 0], ak_range[j, 1], self.num_harmonics - 1)
                bk[j, :-1] = np.random.uniform(bk_range[j, 0], bk_range[j, 1], self.num_harmonics - 1)

                # Calculate the last harmonic to satisfy:
                # 1. sum(a_ik) = 0  => dq(0) = dq(T) = 0
                # 2. sum(k * b_ik) = 0 => ddq(0) = ddq(T) = 0
                ak[j, -1] = -np.sum(ak[j, :-1])

                k_indices = np.arange(1, self.num_harmonics)
                bk[j, -1] = -np.sum(k_indices * bk[j, :-1]) / self.num_harmonics

            if self.check_constraints(ak, bk):
                self.get_logger().info(f'  Valid trajectory found at attempt {attempt + 1}')
                return ak, bk

        # If max attempts reached, scale down until constraints are satisfied
        self.get_logger().warn(
            f'  Max attempts ({self.max_attempts}) reached. Scaling down coefficients...')
        scale = 0.9
        while scale > 0.01:
            ak_scaled = ak * scale
            bk_scaled = bk * scale
            if self.check_constraints(ak_scaled, bk_scaled):
                self.get_logger().info(f'  Scaled trajectory valid at scale={scale:.3f}')
                return ak_scaled, bk_scaled
            scale *= 0.9

        # Last resort: very small coefficients
        self.get_logger().error('  Could not find valid trajectory even with scaling!')
        return ak * 0.01, bk * 0.01

    def _common_metadata(self):
        return {
            'num_harmonics': self.num_harmonics,
            'base_frequency': self.base_frequency,
            'omega_f': self.omega_f,
            'duration': self.duration,
            'sample_rate': self.sample_rate,
            'initial_q': self.initial_q.tolist(),
            'joint_position_min': self.joint_pos_min.tolist(),
            'joint_position_max': self.joint_pos_max.tolist(),
            'joint_velocity_max': self.joint_vel_max.tolist(),
            'joint_acceleration_max': self.joint_acc_max.tolist(),
            'safety_margin': self.safety_margin,
        }

    def _load_existing_summary(self):
        if not self.existing_summary_file:
            raise ValueError(
                'existing_summary_file is required when generation_mode is train or test.')

        if not os.path.exists(self.existing_summary_file):
            raise FileNotFoundError(f'existing_summary_file does not exist: {self.existing_summary_file}')

        with open(self.existing_summary_file, 'r') as f:
            summary = json.load(f)

        self._validate_existing_summary_compatible(summary)
        return summary

    def _validate_existing_summary_compatible(self, summary):
        """Protect against mixing old train/test trajectories with incompatible global parameters."""
        scalar_keys = ['num_harmonics', 'base_frequency', 'duration']
        for key in scalar_keys:
            if key not in summary:
                raise ValueError(f'Existing summary is missing key: {key}')
            old_value = float(summary[key])
            new_value = float(getattr(self, key))
            if not np.isclose(old_value, new_value, rtol=1e-9, atol=1e-12):
                raise ValueError(
                    f'Existing summary {key}={old_value} is incompatible with current config {key}={new_value}. '
                    f'Use generation_mode:=all if you want to change global trajectory parameters.')

        old_initial_q = np.array(summary.get('initial_q', []), dtype=float)
        if old_initial_q.shape != self.initial_q.shape or not np.allclose(old_initial_q, self.initial_q):
            raise ValueError(
                'Existing summary initial_q is incompatible with current config initial_q. '
                'Use generation_mode:=all if you want to change initial_q.')

    def _generate_split(self, split_name, count, ak_range, bk_range, timestamp, metadata_common):
        self.get_logger().info(f'Generating {count} {split_name} trajectories...')
        params = []

        split_dir = os.path.join(self.output_base_dir, split_name)
        os.makedirs(split_dir, exist_ok=True)

        for i in range(count):
            self.get_logger().info(f'  {split_name.capitalize()} trajectory {i + 1}/{count}')
            ak, bk = self.sample_trajectory(ak_range, bk_range)
            param = {
                'index': i + 1,
                'type': split_name,
                'ak': ak.tolist(),
                'bk': bk.tolist(),
            }
            params.append(param)

            # Save individual trajectory parameter file
            filename = f'trajectory_{i + 1:03d}_{timestamp}.json'
            filepath = os.path.join(split_dir, filename)
            with open(filepath, 'w') as f:
                json.dump({**metadata_common, **param}, f, indent=2)

        return params

    def _resolve_summary_path(self, timestamp):
        if self.output_summary_file:
            return self.output_summary_file
        return os.path.join(self.output_base_dir, f'ffs_summary_{timestamp}.json')

    def _backup_if_needed(self, summary_path, timestamp):
        if self.backup_existing_summary and os.path.exists(summary_path):
            backup_path = f'{summary_path}.bak_{timestamp}'
            shutil.copy2(summary_path, backup_path)
            self.get_logger().warn(f'Existing summary will be overwritten. Backup saved to: {backup_path}')

    def generate_trajectories(self):
        """Generate FFS trajectories according to generation_mode."""
        os.makedirs(os.path.join(self.output_base_dir, 'train'), exist_ok=True)
        os.makedirs(os.path.join(self.output_base_dir, 'test'), exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        metadata_common = self._common_metadata()

        existing_summary = None
        train_params = []
        test_params = []

        if self.generation_mode in ('train', 'test'):
            existing_summary = self._load_existing_summary()
            self.get_logger().info(f'Loaded existing summary: {self.existing_summary_file}')

        if self.generation_mode in ('all', 'train'):
            train_params = self._generate_split(
                'train',
                self.num_train,
                self.train_ak_range,
                self.train_bk_range,
                timestamp,
                metadata_common)
        else:
            train_params = existing_summary.get('train_trajectories', [])
            self.num_train = len(train_params)
            self.get_logger().info(
                f'Keeping {self.num_train} train trajectories from existing summary.')

        if self.generation_mode in ('all', 'test'):
            test_params = self._generate_split(
                'test',
                self.num_test,
                self.test_ak_range,
                self.test_bk_range,
                timestamp,
                metadata_common)
        else:
            test_params = existing_summary.get('test_trajectories', [])
            self.num_test = len(test_params)
            self.get_logger().info(
                f'Keeping {self.num_test} test trajectories from existing summary.')

        summary = {
            **metadata_common,
            'timestamp': timestamp,
            'generation_mode': self.generation_mode,
            'source_summary_file': self.existing_summary_file if existing_summary is not None else '',
            'num_train_trajectories': len(train_params),
            'num_test_trajectories': len(test_params),
            'train_ak_range': self.train_ak_range.tolist(),
            'train_bk_range': self.train_bk_range.tolist(),
            'test_ak_range': self.test_ak_range.tolist(),
            'test_bk_range': self.test_bk_range.tolist(),
            'train_trajectories': train_params,
            'test_trajectories': test_params,
        }

        summary_path = self._resolve_summary_path(timestamp)
        os.makedirs(os.path.dirname(summary_path), exist_ok=True)
        self._backup_if_needed(summary_path, timestamp)

        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(f'Trajectories generated. Summary saved to: {summary_path}')
        return summary_path


def main(args=None):
    rclpy.init(args=args)
    node = FFSTrajectoryGenerator()

    try:
        summary_path = node.generate_trajectories()
        node.get_logger().info(f'Done! Summary: {summary_path}')
    except Exception as e:
        node.get_logger().error(f'Error generating trajectories: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
