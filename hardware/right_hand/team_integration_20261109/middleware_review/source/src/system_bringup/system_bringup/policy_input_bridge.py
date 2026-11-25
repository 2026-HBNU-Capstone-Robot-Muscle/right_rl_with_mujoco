"""Synchronize external pose/state topics into the two policy input contracts."""

from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from system_interfaces.msg import (
    CubeExecutionStatus,
    CubePolicyGoal,
    CustomPolicyInput,
    LeapPolicyInput,
)


LEAP_POLICY_JOINT_NAMES = (
    'if_mcp', 'if_rot', 'if_pip', 'if_dip',
    'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
    'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
    'th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
)
CUSTOM_POLICY_FINGER_NAMES = ('right', 'front', 'left', 'back')
CUSTOM_PRIMITIVES = {
    'grasp_upper_layer', 'rotate_upper_layer', 'release_upper_layer'}


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _ordered_joint_values(message, names, field, expected_size):
    source_names = list(message.name)
    values = list(getattr(message, field))
    if (len(names) != expected_size or len(set(names)) != expected_size
            or len(source_names) != len(values)
            or len(set(source_names)) != len(source_names)
            or set(source_names) != set(names)):
        raise ValueError(f'{field} message names do not match configured joint mapping.')
    by_name = dict(zip(source_names, values))
    ordered = np.asarray([by_name[name] for name in names], dtype=np.float64)
    if not np.all(np.isfinite(ordered)):
        raise ValueError(f'{field} contains non-finite values.')
    return ordered


class PolicyInputBridge(Node):
    """Pair fresh source topics with the active workflow and emit policy inputs."""

    def __init__(self):
        super().__init__('policy_input_bridge')
        self.declare_parameter('world_frame', '')
        self.declare_parameter('max_input_age_ms', 100)
        self.declare_parameter('cube_pose_topic', '/cube/pose')
        self.declare_parameter('palm_pose_topic', '/leap_hand/palm_pose')
        self.declare_parameter('leap_joint_state_topic', '/leap_hand/joint_states')
        self.declare_parameter(
            'leap_commanded_state_topic', '/leap_hand/commanded_joint_states')
        self.declare_parameter('goal_topic', '/cube/policy_goal')
        self.declare_parameter(
            'custom_policy_state_topic', '/custom_hand/policy_joint_states')
        self.declare_parameter(
            'execution_status_topic', '/cube/execution_status')
        self.declare_parameter(
            'leap_source_joint_names', list(LEAP_POLICY_JOINT_NAMES))
        self.declare_parameter(
            'leap_command_joint_names', list(LEAP_POLICY_JOINT_NAMES))
        self.declare_parameter(
            'custom_source_joint_names', list(CUSTOM_POLICY_FINGER_NAMES))

        self._world_frame = str(self.get_parameter('world_frame').value).strip()
        self._max_age_ms = int(self.get_parameter('max_input_age_ms').value)
        if self._max_age_ms <= 0:
            raise ValueError('max_input_age_ms must be positive.')
        self._leap_source_names = list(
            self.get_parameter('leap_source_joint_names').value)
        self._leap_command_names = list(
            self.get_parameter('leap_command_joint_names').value)
        self._custom_source_names = list(
            self.get_parameter('custom_source_joint_names').value)
        if not self._world_frame:
            self.get_logger().warning(
                'world_frame is unset; LEAP policy input will be withheld.')

        self._cube_pose = None
        self._palm_pose = None
        self._leap_joints = None
        self._leap_command = None
        self._goal = None
        self._custom_joints = None
        self._status = None
        self._last_warning = ''
        self._last_leap_context = None
        self._last_leap_sample_stamps = None
        self._last_custom_context = None
        self._last_custom_sample_stamp = None

        self._leap_publisher = self.create_publisher(
            LeapPolicyInput, '/leap_hand/policy_input', QoSProfile(depth=1))
        self._custom_publisher = self.create_publisher(
            CustomPolicyInput, '/custom_hand/policy_input', QoSProfile(depth=1))
        self._subscriptions = [
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('cube_pose_topic').value),
                self._on_cube_pose, qos_profile_sensor_data),
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('palm_pose_topic').value),
                self._on_palm_pose, qos_profile_sensor_data),
            self.create_subscription(
                JointState,
                str(self.get_parameter('leap_joint_state_topic').value),
                self._on_leap_joints, qos_profile_sensor_data),
            self.create_subscription(
                JointState,
                str(self.get_parameter('leap_commanded_state_topic').value),
                self._on_leap_command, qos_profile_sensor_data),
            self.create_subscription(
                CubePolicyGoal,
                str(self.get_parameter('goal_topic').value),
                self._on_goal, 10),
            self.create_subscription(
                JointState,
                str(self.get_parameter('custom_policy_state_topic').value),
                self._on_custom_joints, qos_profile_sensor_data),
            self.create_subscription(
                CubeExecutionStatus,
                str(self.get_parameter('execution_status_topic').value),
                self._on_status, 10),
        ]
        self.get_logger().info(
            'Policy input bridge ready. Source topics are external; '
            'outputs feed shadow inference only.')

    def _fresh(self, message, label, now_ns):
        if message is None:
            raise ValueError(f'{label} has not arrived.')
        stamp = _stamp_ns(message.header.stamp)
        age_ms = (now_ns - stamp) / 1_000_000.0
        if stamp <= 0 or age_ms < 0 or age_ms > self._max_age_ms:
            raise ValueError(f'{label} is missing, future-dated, or stale ({age_ms:.1f} ms).')

    def _warn_once(self, text):
        if text != self._last_warning:
            self.get_logger().warning(text)
            self._last_warning = text

    def _try_publish(self):
        now_ns = self.get_clock().now().nanoseconds
        now_msg = self.get_clock().now().to_msg()
        status = self._status
        if (status is None or status.state != CubeExecutionStatus.STATE_EXECUTING
                or not status.scan_id or not status.execution_id):
            return

        if (status.hand_id == 'leap_hand'
                and status.primitive_id == 'orient_face_up_and_support'):
            try:
                self._publish_leap(status, now_ns, now_msg)
            except (ValueError, KeyError) as exc:
                self._warn_once(f'LEAP policy input withheld: {exc}')
        elif (status.hand_id == 'custom_hand'
              and status.primitive_id in CUSTOM_PRIMITIVES):
            try:
                self._publish_custom(status, now_ns, now_msg)
            except (ValueError, KeyError) as exc:
                self._warn_once(f'Custom policy input withheld: {exc}')

    def _publish_leap(self, status, now_ns, now_msg):
        if not self._world_frame:
            raise ValueError('world_frame must be configured from measured calibration.')
        for message, label in (
                (self._cube_pose, 'cube pose'),
                (self._palm_pose, 'palm pose'),
                (self._leap_joints, 'LEAP joint state'),
                (self._leap_command, 'LEAP commanded state')):
            self._fresh(message, label, now_ns)
        if self._goal is None or _stamp_ns(self._goal.header.stamp) <= 0:
            raise ValueError('calibrated cube goal has not arrived or has no timestamp.')

        for pose, label in ((self._cube_pose, 'cube pose'),
                            (self._palm_pose, 'palm pose'),
                            (self._goal, 'cube goal')):
            if pose.header.frame_id != self._world_frame:
                raise ValueError(f'{label} frame_id must equal world_frame.')
        expected_face = status.current_move[:1]
        if (self._goal.scan_id != status.scan_id
                or self._goal.execution_id != status.execution_id
                or self._goal.target_face != expected_face):
            raise ValueError('cube goal does not match the active scan, execution, and face.')

        context = (
            status.scan_id, status.execution_id, status.move_index,
            status.step_index, status.current_move, status.hand_id,
            status.primitive_id)
        if context != self._last_leap_context:
            self._last_leap_context = context
            self._last_leap_sample_stamps = None
        sample_stamps = tuple(_stamp_ns(message.header.stamp) for message in (
            self._cube_pose, self._palm_pose, self._leap_joints,
            self._leap_command))
        previous_stamps = self._last_leap_sample_stamps
        if previous_stamps is not None:
            if any(current < previous for current, previous
                   in zip(sample_stamps, previous_stamps)):
                self._last_leap_sample_stamps = None
            elif any(current == previous for current, previous
                     in zip(sample_stamps, previous_stamps)):
                return

        joint_positions = _ordered_joint_values(
            self._leap_joints, self._leap_source_names, 'position', 16)
        command_positions = _ordered_joint_values(
            self._leap_command, self._leap_command_names, 'position', 16)

        output = LeapPolicyInput()
        output.header.stamp = now_msg
        output.header.frame_id = self._world_frame
        output.scan_id = status.scan_id
        output.execution_id = status.execution_id
        output.target_face = expected_face
        output.cube_pose = deepcopy(self._cube_pose)
        output.palm_pose = deepcopy(self._palm_pose)
        output.goal_orientation = deepcopy(self._goal.goal_orientation)
        output.leap_joint_state.header = deepcopy(self._leap_joints.header)
        output.leap_joint_state.name = list(LEAP_POLICY_JOINT_NAMES)
        output.leap_joint_state.position = [float(v) for v in joint_positions]
        output.commanded_joint_positions = [float(v) for v in command_positions]
        self._leap_publisher.publish(output)
        self._last_leap_sample_stamps = sample_stamps
        self._last_warning = ''

    def _publish_custom(self, status, now_ns, now_msg):
        self._fresh(self._custom_joints, 'Custom policy joint state', now_ns)
        context = (
            status.scan_id, status.execution_id, status.move_index,
            status.step_index, status.current_move, status.hand_id,
            status.primitive_id)
        sample_stamp = _stamp_ns(self._custom_joints.header.stamp)
        if context != self._last_custom_context:
            self._last_custom_context = context
            self._last_custom_sample_stamp = None
        if self._last_custom_sample_stamp is not None:
            if sample_stamp < self._last_custom_sample_stamp:
                self._last_custom_sample_stamp = None
            elif sample_stamp == self._last_custom_sample_stamp:
                return
        positions = _ordered_joint_values(
            self._custom_joints, self._custom_source_names, 'position', 4)
        velocities = _ordered_joint_values(
            self._custom_joints, self._custom_source_names, 'velocity', 4)

        output = CustomPolicyInput()
        output.header.stamp = now_msg
        output.header.frame_id = self._custom_joints.header.frame_id
        output.scan_id = status.scan_id
        output.execution_id = status.execution_id
        output.policy_positions = [float(v) for v in positions]
        output.policy_velocities = [float(v) for v in velocities]
        self._custom_publisher.publish(output)
        self._last_custom_sample_stamp = sample_stamp
        self._last_warning = ''

    def _on_cube_pose(self, message):
        self._cube_pose = message
        self._try_publish()

    def _on_palm_pose(self, message):
        self._palm_pose = message
        self._try_publish()

    def _on_leap_joints(self, message):
        self._leap_joints = message
        self._try_publish()

    def _on_leap_command(self, message):
        self._leap_command = message
        self._try_publish()

    def _on_goal(self, message):
        self._goal = message
        self._try_publish()

    def _on_custom_joints(self, message):
        self._custom_joints = message
        self._try_publish()

    def _on_status(self, message):
        self._status = message
        self._try_publish()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyInputBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
