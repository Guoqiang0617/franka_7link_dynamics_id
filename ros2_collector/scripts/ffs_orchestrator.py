#!/usr/bin/env python3
"""Execute selected FR3 FFS trajectories using already running ROS 2 hardware."""
import hashlib
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class FFSOrchestrator(Node):
    def __init__(self):
        super().__init__('ffs_orchestrator')
        defaults = {
            'summary_file': '',
            'output_base_dir': os.path.join(os.path.expanduser('~'), 'franka_7link_dynamics_id_runs', 'acquisitions'),
            'controller_manager': 'controller_manager',
            'joint_states_topic': 'joint_states',
            'settle_time': 3.0,
            'return_to_start_timeout': 20.0,
            'start_position_tolerance': 0.01,
            'start_velocity_tolerance': 0.03,
            'trajectory_timeout_factor': 1.5,
            'trajectory_split': 'train',
            'trajectory_index': 1,
            'collection_rate_hz': 1000.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        self.output_dir = Path(self.settings['output_base_dir']).expanduser() / datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')
        self.ffs_ctrl = 'ffs_trajectory_controller'
        self.return_ctrl = 'velocity_based_joint_position_controller'
        self.data_ctrl = 'ffs_data_collection_controller'
        manager = str(self.settings['controller_manager']).rstrip('/')
        self.switch_client = self.create_client(SwitchController, f'{manager}/switch_controller')
        self.list_client = self.create_client(ListControllers, f'{manager}/list_controllers')
        self.trajectory_done = False
        self.create_subscription(Bool, f'{self.ffs_ctrl}/trajectory_done', self._done, 10)
        self.joint_names = [f'fr3_joint{j}' for j in range(1, 8)]
        self.joint_state = None
        self.create_subscription(JointState, self.settings['joint_states_topic'], self._state, qos_profile_sensor_data)
        self.executing = False

    def _done(self, msg):
        if msg.data:
            self.trajectory_done = True

    def _state(self, msg):
        indices = {name: index for index, name in enumerate(msg.name)}
        if not all(name in indices for name in self.joint_names):
            return
        selected = [indices[name] for name in self.joint_names]
        if not selected or max(selected) >= min(len(msg.position), len(msg.velocity)):
            return
        q = [msg.position[index] for index in selected]
        dq = [msg.velocity[index] for index in selected]
        if all(math.isfinite(x) for x in q + dq):
            self.joint_state = (time.monotonic(), q, dq)

    def _call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service unavailable: {client.srv_name}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            self.get_logger().error(f'Service timed out: {client.srv_name}')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'Service failed: {client.srv_name}: {exc}')
            return None

    def set_controller_parameter(self, controller, name, value):
        parameter = ParameterMsg()
        parameter.name = name
        if isinstance(value, list):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(x) for x in value]
        elif isinstance(value, str):
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            raise ValueError(f'Unsupported parameter {name}: {type(value)}')
        client = self.create_client(SetParameters, f'{controller}/set_parameters')
        try:
            request = SetParameters.Request()
            request.parameters = [parameter]
            response = self._call(client, request, 5.0)
            if response is None or len(response.results) != 1:
                return False
            result = response.results[0]
            if not result.successful:
                self.get_logger().error(f'{controller}/{name} rejected: {result.reason}')
                return False
            return True
        finally:
            self.destroy_client(client)

    def _return_target(self):
        client = self.create_client(GetParameters, f'{self.return_ctrl}/get_parameters')
        try:
            request = GetParameters.Request()
            request.names = ['target_q']
            response = self._call(client, request, 5.0)
            if response is None or len(response.values) != 1:
                raise RuntimeError('Cannot read return controller target_q.')
            target = list(response.values[0].double_array_value)
            if len(target) != 7 or not all(math.isfinite(x) for x in target):
                raise ValueError('Return controller target_q must contain seven finite positions.')
            return target
        finally:
            self.destroy_client(client)

    def switch_controllers(self, activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        response = self._call(self.switch_client, request)
        if response is None or not response.ok:
            self.get_logger().error(f'Controller switch failed: activate={activate}, deactivate={deactivate}')
            return False
        return True

    def _controllers_ready(self):
        deadline = time.monotonic() + 20.0
        expected = {self.return_ctrl: 'active', self.ffs_ctrl: 'inactive', self.data_ctrl: 'inactive'}
        while rclpy.ok() and time.monotonic() < deadline:
            response = self._call(self.list_client, ListControllers.Request(), 2.0)
            if response is not None:
                states = {controller.name: controller.state for controller in response.controller}
                if all(states.get(name) == state for name, state in expected.items()):
                    return True
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().error('Expected return controller active and FFS/collector inactive.')
        return False

    def wait_for_return_to_start(self, target):
        self.get_logger().info('Waiting for fresh joint states at the configured start pose...')
        deadline = time.monotonic() + float(self.settings['return_to_start_timeout'])
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            ready = False
            if self.joint_state is not None:
                received, q, dq = self.joint_state
                ready = (now-received < 1.0
                         and max(abs(x-y) for x, y in zip(q, target)) <= self.settings['start_position_tolerance']
                         and max(abs(x) for x in dq) <= self.settings['start_velocity_tolerance'])
            if ready:
                if stable_since is None:
                    stable_since = now
                if now-stable_since >= max(0.5, self.settings['settle_time']):
                    return True
            else:
                stable_since = None
        self.get_logger().error('Start pose not confirmed before timeout; no FFS activation.')
        return False

    @staticmethod
    def validate_trajectory(trajectory, target):
        harmonics = int(trajectory['num_harmonics'])
        if harmonics < 2 or float(trajectory['base_frequency']) <= 0 or float(trajectory['duration']) <= 0:
            raise ValueError('Invalid harmonic count, frequency or duration.')
        initial = trajectory['initial_q']
        if (len(initial) != 7 or not all(math.isfinite(float(x)) for x in initial)
                or max(abs(float(x)-y) for x, y in zip(initial, target)) > 1e-6):
            raise ValueError('Trajectory initial_q differs from return controller target_q; update its YAML before launch.')
        for key in ('ak', 'bk'):
            values = trajectory[key]
            if len(values) != 7 or any(len(row) != harmonics for row in values):
                raise ValueError(f'{key} must have shape (7, num_harmonics).')
            if not all(math.isfinite(float(x)) for row in values for x in row):
                raise ValueError(f'Non-finite {key}.')
        for row in trajectory['ak']:
            if abs(sum(row)) > 1e-7:
                raise ValueError('FFS start velocity must be zero.')
        for row in trajectory['bk']:
            if abs(sum((k + 1)*float(value) for k, value in enumerate(row))) > 1e-7:
                raise ValueError('FFS start acceleration must be zero.')
        cycles = float(trajectory['duration'])*float(trajectory['base_frequency'])
        if not math.isfinite(cycles) or abs(cycles-round(cycles)) > 1e-8:
            raise ValueError('FFS duration must contain an integer number of periods.')

    def execute_trajectory(self, trajectory, output):
        options = {key: trajectory[key] for key in ('num_harmonics', 'base_frequency', 'duration', 'initial_q')}
        options['num_harmonics'] = int(options['num_harmonics'])
        options['base_frequency'] = float(options['base_frequency'])
        options['duration'] = float(options['duration'])
        for key in ('ak', 'bk'):
            options[key] = [float(x) for row in trajectory[key] for x in row]
        for key, value in options.items():
            if not self.set_controller_parameter(self.ffs_ctrl, key, value):
                return False
        if not self.set_controller_parameter(self.data_ctrl, 'output_file', str(output)):
            return False
        if not self.set_controller_parameter(self.data_ctrl, 'collection_rate_hz', float(self.settings['collection_rate_hz'])):
            return False
        self.trajectory_done = False
        if not self.switch_controllers([self.ffs_ctrl, self.data_ctrl], [self.return_ctrl]):
            return False
        self.executing = True
        deadline = time.monotonic() + options['duration']*self.settings['trajectory_timeout_factor'] + 5.0
        while rclpy.ok() and not self.trajectory_done and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.switch_controllers([self.return_ctrl], [self.ffs_ctrl, self.data_ctrl]):
            return False
        self.executing = False
        if not self.trajectory_done:
            self.get_logger().error('FFS completion was not received; collection aborted.')
            return False
        self.get_logger().info(f'Completed: {output}')
        return True

    def run(self):
        summary_path = Path(self.settings['summary_file']).expanduser()
        summary_bytes = summary_path.read_bytes()
        summary = json.loads(summary_bytes.decode('utf-8'))
        split = self.settings['trajectory_split']
        index = int(self.settings['trajectory_index'])
        if split not in ('all', 'train', 'test') or index < 0:
            raise ValueError('Use split=train/test/all; index=0 for all or a positive 1-based index.')
        for key in ('return_to_start_timeout', 'start_position_tolerance', 'start_velocity_tolerance', 'trajectory_timeout_factor', 'collection_rate_hz'):
            if not math.isfinite(float(self.settings[key])) or self.settings[key] <= 0:
                raise ValueError(f'{key} must be positive and finite.')
        if not math.isfinite(float(self.settings['settle_time'])) or self.settings['settle_time'] < 0:
            raise ValueError('settle_time must be nonnegative and finite.')
        if not self._controllers_ready():
            return False
        target = self._return_target()
        common = {key: summary[key] for key in ('num_harmonics', 'base_frequency', 'duration', 'initial_q') if key in summary}
        selected = []
        for part in ('train', 'test'):
            if split not in ('all', part):
                continue
            for ordinal, values in enumerate(summary.get(f'{part}_trajectories', []), start=1):
                if index not in (0, ordinal):
                    continue
                trajectory = {**common, **values}
                self.validate_trajectory(trajectory, target)
                selected.append((part, ordinal, trajectory))
        if not selected:
            raise ValueError('No trajectories match the requested split/index.')
        timestamp = str(summary.get('timestamp', datetime.now().strftime('%Y%m%d_%H%M%S')))
        if len(timestamp) != 15 or timestamp[8] != '_' or not timestamp.replace('_', '').isdigit():
            raise ValueError('Summary timestamp must have the format YYYYMMDD_HHMMSS.')
        self.output_dir.mkdir(parents=True, exist_ok=False)
        (self.output_dir/'summary_snapshot.json').write_bytes(summary_bytes)
        report = {'summary_source': str(summary_path.resolve()),
                  'summary_sha256': hashlib.sha256(summary_bytes).hexdigest(),
                  'settings': self.settings, 'trajectories': [], 'complete': False}
        for part, ordinal, trajectory in selected:
            output = self.output_dir/part/f'trajectory_{ordinal:03d}_{timestamp}.csv'
            output.parent.mkdir(exist_ok=True)
            success = (self._controllers_ready() and self.wait_for_return_to_start(target)
                       and self.execute_trajectory(trajectory, output))
            report['trajectories'].append({'split': part, 'index': ordinal, 'file': str(output), 'success': success})
            (self.output_dir/'collection_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            if not success:
                return False
        report['complete'] = True
        (self.output_dir/'collection_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FFSOrchestrator()
    success = False
    try:
        success = node.run()
    except KeyboardInterrupt:
        node.get_logger().error('Collection interrupted.')
    except Exception as exc:
        node.get_logger().error(f'Collection failed: {exc}')
    finally:
        if node.executing and rclpy.ok():
            node.switch_controllers([node.return_ctrl], [node.ffs_ctrl, node.data_ctrl])
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
