#!/usr/bin/env python3
"""离线跨进程验证状态桥、ROS 消息和 MoveIt 状态接收，不连接控制柜。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[2]
JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
OPTIONS = None


class BridgeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('aubo_bridge_offline_validation')
        base = OPTIONS.install_base
        cls.bridge = base / 'aubo_joint_state_bridge/lib/aubo_joint_state_bridge/joint_state_bridge'
        cls.snapshot = base / 'i10_rh56_moveit_config/lib/i10_rh56_moveit_config/snapshot_arm_state'
        cls.config = base / 'i10_rh56_moveit_config/share/i10_rh56_moveit_config/config'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='aubo_bridge_test_')
        self.path = Path(self.directory.name)
        self.processes = []
        self.subscriptions = []
        suffix = hashlib.sha256(self._testMethodName.encode()).hexdigest()[:8]
        self.topic = f'/aubo_bridge_test/case_{suffix}/joint_states'

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
            process.communicate(timeout=5)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.directory.cleanup()

    def start(self, executable, **parameters):
        command = [str(executable), '--ros-args']
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        environment = dict(os.environ, ROS_LOG_DIR=str(self.path / 'ros_logs'))
        process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        self.processes.append(process)
        return process

    def records(self, position=None, velocity=None, count=35):
        position = position if position is not None else [0, -.5, 1.2, 0, 0, 0]
        return [dict(parse_ok=True, host_monotonic_s=10 + index * .02,
                     R1_actual_q=position, R1_actual_qd=velocity if velocity is not None else [0.] * 6)
                for index in range(count)]

    def replay(self, records, **parameters):
        file = self.path / 'samples.jsonl'
        file.write_text(''.join(json.dumps(record) + '\n' for record in records))
        return self.start(self.bridge, source='replay', replay_path=file,
                          joint_states_topic=parameters.pop('joint_states_topic', self.topic),
                          stale_timeout_s=.15, **parameters)

    def listen(self):
        messages = []
        self.subscriptions.append(self.node.create_subscription(JointState, self.topic, messages.append, 10))
        return messages

    def collect_until_exit(self, process, timeout=8):
        deadline = time.monotonic() + timeout
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertIsNotNone(process.poll(), '状态桥未按期退出')
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=.01)
        return process.communicate(timeout=2)[0]

    def test_recorded_capture_reaches_moveit(self):
        """使用已有真机历史帧，核对解析后的关节值进入 MoveIt 六轴状态。"""
        capture = ROOT / 'artifacts/hardware/aubo_sdk_readonly_20261023_002/rtde_samples.jsonl'
        if not capture.is_file():
            self.skipTest('本机没有历史采集文件')
        records = [json.loads(line) for line in capture.read_text().splitlines()[:60]]
        messages = self.listen()
        output = self.path / 'snapshot.json'
        monitor = self.start(self.snapshot, config_dir=self.config, joint_states_topic=self.topic,
                             state_timeout_s=4., output_path=output)
        process = self.replay(records)
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertIn('断流', log)
        monitor_log = monitor.communicate(timeout=6)[0]
        self.assertEqual(monitor.returncode, 0, monitor_log)
        result = json.loads(output.read_text())
        self.assertTrue(result['valid'])
        self.assertFalse(result['hand_measured'])
        self.assertEqual(result['arm_joints'], JOINTS)
        self.assertIn(result['arm_q_rad'], [record['R1_actual_q'] for record in records])
        self.assertGreater(len(messages), 10)
        expected = {(tuple(record['R1_actual_q']), tuple(record['R1_actual_qd'])) for record in records}
        stamps = []
        for message in messages:
            self.assertEqual(list(message.name), JOINTS)
            self.assertIn((tuple(message.position), tuple(message.velocity)), expected)
            self.assertEqual(list(message.effort), [])
            stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
        self.assertEqual(stamps, sorted(set(stamps)))
        self.assertLessEqual(len(messages), len(records))

    def test_preserves_winding_and_missing_velocity(self):
        messages = self.listen()
        # 保留足够长的流，避免把 DDS 首次发现耗时误判成消息丢失。
        records = self.records(position=[0, -1, 2, -3, 4, 7], count=100)
        for record in records:
            del record['R1_actual_qd']
        process = self.replay(records)
        log = self.collect_until_exit(process)
        self.assertGreater(len(messages), 5, log + '\nROS 话题：' + str(self.node.get_topic_names_and_types()))
        self.assertTrue(all(list(message.position) == [0, -1, 2, -3, 4, 7] for message in messages))
        self.assertTrue(all(not message.velocity for message in messages))

    def test_replay_cannot_publish_as_live_state(self):
        process = self.replay(self.records(), joint_states_topic='/joint_states')
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertIn('独立话题', log)

    def test_partial_measurement_is_rejected(self):
        messages = self.listen()
        process = self.replay(self.records(position=[0, 1]))
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertEqual(messages, [])

    def test_nonfinite_measurement_is_rejected(self):
        messages = self.listen()
        process = self.replay(self.records(position=[0, 0, 0, 0, 0, float('nan')]))
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertEqual(messages, [])

    def test_moveit_timeout_invalidates_old_snapshot(self):
        output = self.path / 'snapshot.json'
        output.write_text('{"valid":true}')
        process = self.start(self.snapshot, config_dir=self.config, joint_states_topic=self.topic,
                             state_timeout_s=.3, output_path=output)
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertFalse(json.loads(output.read_text())['valid'])

    def test_moveit_rejects_partial_or_stale_arm(self):
        """逐关节检查新鲜度，不能由单个新关节或旧整帧凑出完整起点。"""
        publisher = self.node.create_publisher(JointState, self.topic, 10)
        try:
            process = self.start(self.snapshot, config_dir=self.config, joint_states_topic=self.topic,
                                 state_timeout_s=.8, state_max_age_s=.1)
            deadline = time.monotonic() + 1.5
            while process.poll() is None and time.monotonic() < deadline:
                message = JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name = JOINTS[:1]
                message.position = [0.]
                publisher.publish(message)
                rclpy.spin_once(self.node, timeout_sec=.02)
            log = self.collect_until_exit(process)
            self.assertEqual(process.returncode, 1, log)
            process = self.start(self.snapshot, config_dir=self.config, joint_states_topic=self.topic,
                                 state_timeout_s=.8, state_max_age_s=.1)
            deadline = time.monotonic() + 1.5
            while process.poll() is None and time.monotonic() < deadline:
                message = JointState()
                message.name = JOINTS
                message.position = [0.] * 6
                message.header.stamp.sec = 1
                publisher.publish(message)
                rclpy.spin_once(self.node, timeout_sec=.02)
            log = self.collect_until_exit(process)
            self.assertEqual(process.returncode, 1, log)
        finally:
            self.node.destroy_publisher(publisher)

    def test_live_connection_requires_explicit_address(self):
        process = self.start(self.bridge)
        log = self.collect_until_exit(process)
        self.assertEqual(process.returncode, 1, log)
        self.assertIn('显式提供', log)

    def test_planner_uses_measured_start(self):
        """请求臂起点与回放状态故意不同，检查 MTC 路径使用后者并保存来源。"""
        request = self.path / 'request.json'
        output = self.path / 'plan.json'
        preparation = subprocess.run([
            str(ROOT / '.venv/bin/python'), str(ROOT / 'tools/prepare_cap_approach.py'),
            '--start', '0', '-0.4', '1.0', '0', '0', '0', '--distance', '.1',
            '--output', str(request)], capture_output=True, text=True, timeout=30)
        self.assertEqual(preparation.returncode, 0, preparation.stdout + preparation.stderr)
        # 使用已有独立几何复核通过的预抓取构型，避免长距离随机路径影响状态接入检查。
        baseline = ROOT / 'artifacts/e2_checks/fixed_grasp_20261020/baseline/trajectory.json'
        measured = json.loads(baseline.read_text())['stages'][0]['points'][-1]
        self.replay(self.records(position=measured, count=1000))
        command = [
            'ros2', 'launch', 'i10_rh56_moveit_config', 'approach.launch.py',
            f'request_path:={request}', f'output_path:={output}',
            'start_source:=joint_states', f'joint_states_topic:={self.topic}',
            'state_timeout_s:=5.0']
        result = subprocess.run(command, capture_output=True, text=True, timeout=50,
                                env=dict(os.environ, ROS_LOG_DIR=str(self.path / 'ros_logs')))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(output.read_text())
        self.assertTrue(report['dense_fcl_check_passed'])
        self.assertEqual(report['stages'][0]['points'][0], measured)
        actual_request = report['request']
        self.assertEqual([actual_request['initial_joint_positions'][name] for name in JOINTS], measured)
        self.assertEqual(actual_request['initial_state_source']['arm'], 'joint_states')
        self.assertEqual(actual_request['initial_state_source']['hand'], 'request_preform_unmeasured')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-base', type=Path, required=True)
    OPTIONS, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__, *remaining])
