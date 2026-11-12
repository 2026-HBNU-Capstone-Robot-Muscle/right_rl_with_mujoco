"""Run a selected offline-trained hand policy from an explicit open-loop vector."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from system_interfaces.action import InferHandPolicy

from custom_hand_control.policy_adapter import CustomPolicyAdapter
from leap_hand_control.policy_adapter import LeapPolicyAdapter


POLICY_SPECS = {
    'leap_faceup_v1': {
        'hand_id': 'leap_hand',
        'package': 'leap_hand_control',
        'filename': 'policy_np.npz',
        'observation_size': 108,
        'action_size': 16,
        'adapter': LeapPolicyAdapter,
    },
    'custom_finger_ppo_v1': {
        'hand_id': 'custom_hand',
        'package': 'custom_hand_control',
        'filename': 'finger_robot_ppo_best.zip',
        'observation_size': 8,
        'action_size': 4,
        'adapter': CustomPolicyAdapter,
    },
}


class OpenLoopPolicyBridge(Node):
    """Select a registered fixed model and infer from caller-maintained state."""

    def __init__(self):
        super().__init__('open_loop_policy_bridge')
        self.declare_parameter(
            'leap_policy_path',
            str(Path(get_package_share_directory('leap_hand_control'))
                / 'models' / 'policy_np.npz'))
        self.declare_parameter(
            'custom_policy_path',
            str(Path(get_package_share_directory('custom_hand_control'))
                / 'models' / 'finger_robot_ppo_best.zip'))

        self._policies = {}
        for policy_id, spec in POLICY_SPECS.items():
            parameter = ('leap_policy_path' if spec['hand_id'] == 'leap_hand'
                         else 'custom_policy_path')
            path = str(self.get_parameter(parameter).value).strip()
            if not path:
                raise ValueError(f'{parameter} must point to a model file.')
            adapter = spec['adapter'](path)
            self._policies[policy_id] = (spec, adapter)
            self.get_logger().info(
                f"Registered {policy_id}: hand={spec['hand_id']}, "
                f"observation={spec['observation_size']}, "
                f"action={spec['action_size']}; motor output disabled.")

        self._server = ActionServer(
            self,
            InferHandPolicy,
            '/policy/infer',
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            cancel_callback=self._cancel_request,
        )
        self.get_logger().info(
            'Open-loop policy bridge ready: /policy/infer; '
            'caller supplies and updates the observation list.')

    @staticmethod
    def _goal_request(_goal):
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_request(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = InferHandPolicy.Result()
        result.scan_id = goal.scan_id
        result.execution_id = goal.execution_id
        result.step_index = goal.step_index
        result.hand_id = goal.hand_id
        result.policy_id = goal.policy_id
        result.shadow_only = True

        feedback = InferHandPolicy.Feedback()
        feedback.state = InferHandPolicy.Feedback.STATE_ACCEPTED
        feedback.hand_id = goal.hand_id
        feedback.policy_id = goal.policy_id
        feedback.message = '모델 선택과 요청 식별자를 확인했습니다.'
        goal_handle.publish_feedback(feedback)

        selected = self._policies.get(goal.policy_id)
        if selected is None:
            return self._abort(
                goal_handle, result, feedback, 'UNKNOWN_POLICY',
                f'등록되지 않은 policy_id입니다: {goal.policy_id}')
        spec, adapter = selected
        if goal.hand_id != spec['hand_id']:
            return self._abort(
                goal_handle, result, feedback, 'POLICY_HAND_MISMATCH',
                f"{goal.policy_id}는 {spec['hand_id']} 전용 모델입니다.")
        if (not goal.scan_id or not goal.execution_id or goal.step_index == 0):
            return self._abort(
                goal_handle, result, feedback, 'INVALID_EXECUTION_CONTEXT',
                'scan_id, execution_id, step_index가 필요합니다.')

        observation = np.asarray(goal.observation, dtype=np.float64)
        expected = spec['observation_size']
        if (observation.shape != (expected,)
                or not np.all(np.isfinite(observation))):
            return self._abort(
                goal_handle, result, feedback, 'INVALID_OBSERVATION',
                f'{goal.policy_id}는 유한한 값 {expected}개가 필요합니다. '
                f'입력 차원: {observation.size}')

        if goal_handle.is_cancel_requested:
            result.success = False
            result.error_code = 'CANCELLED'
            result.error_message = '추론 요청이 취소되었습니다.'
            goal_handle.canceled()
            feedback.state = InferHandPolicy.Feedback.STATE_FAILED
            feedback.message = result.error_message
            goal_handle.publish_feedback(feedback)
            return result

        feedback.state = InferHandPolicy.Feedback.STATE_INFERENCING
        feedback.message = '선택된 고정 모델로 deterministic 추론을 실행합니다.'
        goal_handle.publish_feedback(feedback)
        try:
            action = np.asarray(adapter.infer(observation), dtype=np.float64)
        except Exception as exc:  # Return model errors through the ROS action result.
            return self._abort(
                goal_handle, result, feedback, 'INFERENCE_FAILED', str(exc))

        if (action.shape != (spec['action_size'],)
                or not np.all(np.isfinite(action))):
            return self._abort(
                goal_handle, result, feedback, 'INVALID_MODEL_OUTPUT',
                f"모델 출력은 유한한 값 {spec['action_size']}개여야 합니다.")

        result.success = True
        result.action_values = action.tolist()
        result.error_code = ''
        result.error_message = (
            '사전학습 모델 추론이 완료됐습니다. 출력은 shadow 값이며 '
            '모터 명령으로 보내지 않았습니다.')
        goal_handle.succeed()
        feedback.state = InferHandPolicy.Feedback.STATE_COMPLETED
        feedback.message = result.error_message
        goal_handle.publish_feedback(feedback)
        return result

    @staticmethod
    def _abort(goal_handle, result, feedback, code, message):
        result.success = False
        result.error_code = code
        result.error_message = message
        goal_handle.abort()
        feedback.state = InferHandPolicy.Feedback.STATE_FAILED
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPolicyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
