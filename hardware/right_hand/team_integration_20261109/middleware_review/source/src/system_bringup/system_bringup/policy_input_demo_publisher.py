"""Publish clearly synthetic inputs to exercise policy shadow topics only."""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from system_interfaces.msg import (
    CubeExecutionStatus,
    CubePolicyGoal,
    HandPolicyAction,
)


LEAP_JOINT_NAMES = [
    'if_mcp', 'if_rot', 'if_pip', 'if_dip',
    'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
    'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
    'th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
]


class PolicyInputDemoPublisher(Node):
    """Publish synthetic, nonphysical policy samples for manual smoke checking."""

    def __init__(self):
        super().__init__('policy_input_demo_publisher')
        self.declare_parameter('world_frame', 'policy_demo_world')
        self._world_frame = str(self.get_parameter('world_frame').value).strip()
        if not self._world_frame:
            raise ValueError('world_frame must be non-empty for the demo publisher.')
        self._cube_pose_pub = self.create_publisher(PoseStamped, '/cube/pose', 10)
        self._palm_pose_pub = self.create_publisher(
            PoseStamped, '/leap_hand/palm_pose', 10)
        self._leap_joint_pub = self.create_publisher(
            JointState, '/leap_hand/joint_states', 10)
        self._leap_command_pub = self.create_publisher(
            JointState, '/leap_hand/commanded_joint_states', 10)
        self._custom_joint_pub = self.create_publisher(
            JointState, '/custom_hand/policy_joint_states', 10)
        self._goal_pub = self.create_publisher(CubePolicyGoal, '/cube/policy_goal', 10)
        self._status_pub = self.create_publisher(
            CubeExecutionStatus, '/cube/execution_status', 10)
        self._last_action_report = {}
        self._action_subscriptions = [
            self.create_subscription(
                HandPolicyAction, '/leap_hand/policy_action', self._on_action, 10),
            self.create_subscription(
                HandPolicyAction, '/custom_hand/policy_action', self._on_action, 10),
        ]
        self._timer = self.create_timer(0.05, self._publish_sample)
        self.get_logger().warning(
            'Publishing synthetic data on project input topics. '
            'Do not run with camera, robot-state, or cube executor nodes.')

    def _pose(self, now, position):
        message = PoseStamped()
        message.header.stamp = now
        message.header.frame_id = self._world_frame
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.w = 1.0
        return message

    def _joint_state(self, now, names):
        message = JointState()
        message.header.stamp = now
        message.header.frame_id = self._world_frame
        message.name = list(names)
        message.position = [0.0] * len(names)
        message.velocity = [0.0] * len(names)
        return message

    def _status(self, now, hand_id, primitive_id, step_index):
        message = CubeExecutionStatus()
        message.header.stamp = now
        message.header.frame_id = self._world_frame
        message.scan_id = 'policy-demo-scan'
        message.execution_id = 'policy-demo-execution'
        message.workflow_id = 'two_hand_layer_turn_v1'
        message.state = CubeExecutionStatus.STATE_EXECUTING
        message.move_index = 1
        message.total_moves = 1
        message.current_move = 'U'
        message.step_index = step_index
        message.total_steps = 4
        message.hand_id = hand_id
        message.primitive_id = primitive_id
        return message

    def _publish_sample(self):
        now = self.get_clock().now().to_msg()
        self._cube_pose_pub.publish(self._pose(now, [0.0, 0.0, 0.0]))
        self._palm_pose_pub.publish(self._pose(now, [0.0, 0.0, 0.0]))
        self._leap_joint_pub.publish(self._joint_state(now, LEAP_JOINT_NAMES))
        self._leap_command_pub.publish(self._joint_state(now, LEAP_JOINT_NAMES))
        self._custom_joint_pub.publish(
            self._joint_state(now, ['right', 'front', 'left', 'back']))

        goal = CubePolicyGoal()
        goal.header.stamp = now
        goal.header.frame_id = self._world_frame
        goal.scan_id = 'policy-demo-scan'
        goal.execution_id = 'policy-demo-execution'
        goal.target_face = 'U'
        goal.goal_orientation.w = 1.0
        self._goal_pub.publish(goal)

        self._status_pub.publish(self._status(
            now, 'leap_hand', 'orient_face_up_and_support', 1))
        self._status_pub.publish(self._status(
            now, 'custom_hand', 'grasp_upper_layer', 2))

    def _on_action(self, message):
        now = time.monotonic()
        previous = self._last_action_report.get(message.hand_id, 0.0)
        if now - previous < 1.0:
            return
        self._last_action_report[message.hand_id] = now
        self.get_logger().info(
            f"{message.hand_id}: shadow_only={message.shadow_only}, "
            f"elements={len(message.values)}, values={message.values}")


def main(args=None):
    rclpy.init(args=args)
    node = PolicyInputDemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
