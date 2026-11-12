"""Infer Custom Hand policy outputs from calibrated ROS states, without motors."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile
from system_interfaces.msg import CustomPolicyInput, HandPolicyAction

from custom_hand_control.policy_adapter import CustomPolicyAdapter


FINGER_ORDER = ('right', 'front', 'left', 'back')


class CustomPolicyShadowNode(Node):
    """Consume already-calibrated policy coordinates and publish four actions."""

    def __init__(self):
        super().__init__('custom_hand_policy_shadow_node')
        package_share = Path(get_package_share_directory('custom_hand_control'))
        self.declare_parameter(
            'policy_path',
            str(package_share / 'models' / 'finger_robot_ppo_best.zip'))
        # The reference sim2real configuration sets max_age_s to 0.1.
        self.declare_parameter('max_input_age_ms', 100)
        self._max_age_ms = int(self.get_parameter('max_input_age_ms').value)
        if self._max_age_ms <= 0:
            raise ValueError('max_input_age_ms must be positive.')
        self._policy = CustomPolicyAdapter(
            self.get_parameter('policy_path').value)
        self._subscription = self.create_subscription(
            CustomPolicyInput, '/custom_hand/policy_input', self._on_input,
            QoSProfile(depth=1))
        self._publisher = self.create_publisher(
            HandPolicyAction, '/custom_hand/policy_action', QoSProfile(depth=1))
        self.get_logger().info(
            'Shadow inference ready: /custom_hand/policy_input → '
            '/custom_hand/policy_action; four finger outputs only, motor output disabled.')

    def _on_input(self, message):
        try:
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec))
            if stamp_ns <= 0:
                raise ValueError('input timestamp is missing.')
            age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0
            if age_ms < 0 or age_ms > self._max_age_ms:
                raise ValueError(f'input timestamp is future-dated or stale ({age_ms:.1f} ms).')
            if not message.scan_id or not message.execution_id:
                raise ValueError('scan_id and execution_id are required.')

            observation = np.asarray(
                list(message.policy_positions) + list(message.policy_velocities),
                dtype=np.float32)
            action = self._policy.infer(observation)
            output = HandPolicyAction()
            output.header = message.header
            output.hand_id = 'custom_hand'
            output.scan_id = message.scan_id
            output.execution_id = message.execution_id
            output.element_names = list(FINGER_ORDER)
            output.values = [float(value) for value in action]
            output.shadow_only = True
            self._publisher.publish(output)
        except ValueError as exc:
            self.get_logger().warning(f'Custom policy input rejected: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = CustomPolicyShadowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
