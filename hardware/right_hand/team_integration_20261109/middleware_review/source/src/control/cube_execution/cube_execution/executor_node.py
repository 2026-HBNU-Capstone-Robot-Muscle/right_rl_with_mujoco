"""Validate cube solutions and run their two-hand workflow in dry-run mode."""

import re
import uuid
from collections import OrderedDict
from threading import Lock

import numpy as np
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from cube_execution.calibration import validate_calibration_file
from system_interfaces.action import (
    ExecuteCubeSolution, ExecuteHandPrimitive, InferHandPolicy)
from system_interfaces.msg import (
    CubeExecutionStatus, CubeSolution, OpenLoopPolicyState)
from leap_hand_control.observation_builder import LeapObservationBuilder


_MOVE_PATTERN = re.compile(r"^[URFDLB](?:2|')?$")
_FACES = 'URFDLB'
_MAX_SAVED_SOLUTIONS = 32
_WORKFLOW_ID = 'two_hand_layer_turn_v1'
_POLICY_BY_HAND = {
    'leap_hand': 'leap_faceup_v1',
    'custom_hand': 'custom_finger_ppo_v1',
}
_LEAP_ACTION_SCALE = 0.2
_LEAP_LOW = np.array([
    -0.314, -1.047, -0.2, -0.2, -0.314, -1.047, -0.2, -0.2,
    -0.314, -1.047, -0.2, -0.2, -0.349, -0.47, -1.2, -1.34])
_LEAP_HIGH = np.array([
    2.23, 1.047, 1.885, 2.042, 2.23, 1.047, 1.885, 2.042,
    2.23, 1.047, 1.885, 2.042, 2.094, 2.443, 1.9, 1.88])
_CUSTOM_LOW = np.array([-0.0873, 0.0, -0.0441, 0.0])
_CUSTOM_HIGH = np.array([2.0071, 2.0944, 2.0503, 2.0944])
_FACE_ORDER = 'URFDLB'


def validate_solution(solution):
    """Return an error code/message pair, or None when the solution is valid."""
    if not solution.scan_id.strip():
        return 'INVALID_SOLUTION', 'scan_id가 비어 있습니다.'

    facelets = solution.facelets
    if (len(facelets) != 54 or any(face not in _FACES for face in facelets)
            or any(facelets.count(face) != 9 for face in _FACES)):
        return 'INVALID_SOLUTION', 'facelets는 URFDLB 각 9개, 총 54자여야 합니다.'

    invalid_moves = [move for move in solution.moves if not _MOVE_PATTERN.fullmatch(move)]
    if invalid_moves:
        return 'INVALID_SOLUTION', f'허용되지 않은 이동 토큰: {invalid_moves[0]}'

    return None


def compile_workflow(moves):
    """Expand each Kociemba token into semantic LEAP and Custom Hand steps.

    The target face stays in Kociemba's logical URFDLB frame. A hand adapter
    must combine it with measured cube pose before issuing physical commands.
    """
    plan = []
    for move_index, move in enumerate(moves, start=1):
        target_face = move[0]
        suffix = move[1:]
        turns = 2 if suffix == '2' else -1 if suffix == "'" else 1
        plan.extend([
            {
                'move_index': move_index,
                'cube_move': move,
                'target_face': target_face,
                'clockwise_quarter_turns': 0,
                'hand_id': 'leap_hand',
                'primitive_id': 'orient_face_up_and_support',
            },
            {
                'move_index': move_index,
                'cube_move': move,
                'target_face': target_face,
                'clockwise_quarter_turns': 0,
                'hand_id': 'custom_hand',
                'primitive_id': 'grasp_upper_layer',
            },
            {
                'move_index': move_index,
                'cube_move': move,
                'target_face': target_face,
                'clockwise_quarter_turns': turns,
                'hand_id': 'custom_hand',
                'primitive_id': 'rotate_upper_layer',
            },
            {
                'move_index': move_index,
                'cube_move': move,
                'target_face': target_face,
                'clockwise_quarter_turns': 0,
                'hand_id': 'custom_hand',
                'primitive_id': 'release_upper_layer',
            },
        ])
    return plan


class CubeSequenceExecutor(Node):
    """Run the declared workflow while keeping physical motion disabled."""

    def __init__(self):
        super().__init__('cube_sequence_executor')
        self._callback_group = ReentrantCallbackGroup()
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        solution_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._solutions = OrderedDict()
        self._solutions_lock = Lock()
        self._execution_busy = False
        self._execution_busy_lock = Lock()
        self._active_goal_handle = None
        self.declare_parameter('calibration_file', '')
        calibration_file = self.get_parameter('calibration_file').value
        self._calibration_errors = validate_calibration_file(calibration_file)
        if self._calibration_errors:
            self.get_logger().warning(
                'Physical calibration gate is closed: '
                + '; '.join(self._calibration_errors[:5]))
        self._status_publisher = self.create_publisher(
            CubeExecutionStatus, '/cube/execution_status', status_qos)
        self._solution_subscription = self.create_subscription(
            CubeSolution, '/cube/solution', self._on_solution, solution_qos)
        self._hand_clients = {
            'leap_hand': ActionClient(
                self, ExecuteHandPrimitive, '/leap_hand/execute_primitive',
                callback_group=self._callback_group),
            'custom_hand': ActionClient(
                self, ExecuteHandPrimitive, '/custom_hand/execute_primitive',
                callback_group=self._callback_group),
        }
        self._policy_client = ActionClient(
            self, InferHandPolicy, '/policy/infer',
            callback_group=self._callback_group)
        self._action_server = ActionServer(
            self,
            ExecuteCubeSolution,
            '/cube/execute_solution',
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            cancel_callback=self._cancel_request,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'Ready: /cube/execute_solution; dry-run dispatches semantic steps to both hand endpoints')

    def _goal_request(self, _goal_request):
        with self._execution_busy_lock:
            if self._execution_busy:
                return GoalResponse.REJECT
            self._execution_busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_request(_goal_handle):
        return CancelResponse.ACCEPT

    def _on_solution(self, solution):
        error = validate_solution(solution)
        if error:
            code, message = error
            self._publish_status(
                solution.scan_id, '', '', CubeExecutionStatus.STATE_FAILED,
                error_code=code, message=message)
            self.get_logger().warning(f'Solution rejected: {message}')
            return

        saved = self._copy_solution(solution)
        with self._solutions_lock:
            self._solutions[saved.scan_id] = saved
            self._solutions.move_to_end(saved.scan_id)
            while len(self._solutions) > _MAX_SAVED_SOLUTIONS:
                self._solutions.popitem(last=False)

        self._publish_status(
            saved.scan_id, '', '', CubeExecutionStatus.STATE_RECEIVED,
            total_moves=len(saved.moves), message='풀이를 수신하고 검증했습니다.')
        self._publish_status(
            saved.scan_id, '', '', CubeExecutionStatus.STATE_WAITING_FOR_EXECUTION,
            total_moves=len(saved.moves), message='실행 액션 목표를 기다립니다.')

    async def _execute(self, goal_handle):
        self._active_goal_handle = goal_handle
        try:
            return await self._execute_goal(goal_handle)
        finally:
            self._active_goal_handle = None
            with self._execution_busy_lock:
                self._execution_busy = False

    async def _execute_goal(self, goal_handle):
        goal = goal_handle.request
        result = ExecuteCubeSolution.Result()
        result.scan_id = goal.scan_id
        result.dry_run = goal.dry_run
        result.execution_id = str(uuid.uuid4())
        workflow_id = goal.workflow_id.strip() or _WORKFLOW_ID

        with self._solutions_lock:
            saved = self._solutions.get(goal.scan_id)
            solution = self._copy_solution(saved) if saved is not None else None

        if solution is None:
            return self._fail(
                goal_handle, result, workflow_id, 'SOLUTION_NOT_FOUND',
                '요청한 scan_id의 검증된 풀이를 찾지 못했습니다.')

        self._publish_status(
            goal.scan_id, result.execution_id, workflow_id,
            CubeExecutionStatus.STATE_VALIDATING, total_moves=len(solution.moves),
            message='실행 목표와 저장된 풀이를 확인합니다.')
        error = validate_solution(solution)
        if error:
            return self._fail(goal_handle, result, workflow_id, *error,
                              total_moves=len(solution.moves))

        if workflow_id != _WORKFLOW_ID:
            return self._fail(
                goal_handle, result, workflow_id, 'WORKFLOW_NOT_FOUND',
                f'지원하지 않는 workflow_id입니다. 사용 가능 값: {_WORKFLOW_ID}',
                total_moves=len(solution.moves))

        if not goal.dry_run:
            if self._calibration_errors:
                return self._fail(
                    goal_handle, result, workflow_id, 'CALIBRATION_INCOMPLETE',
                    '실제 구동 calibration 검증이 완료되지 않았습니다: '
                    + '; '.join(self._calibration_errors))
            return self._fail(
                goal_handle, result, workflow_id, 'EXECUTION_DISABLED',
                'calibration 기록은 통과했지만 실제 모터 드라이버와 안전 정지 adapter는 아직 구현되지 않았습니다.',
                total_moves=len(solution.moves))

        if goal.run_pretrained_policies:
            state_error = self._validate_open_loop_state(
                goal.initial_open_loop_state)
            if state_error:
                return self._fail(
                    goal_handle, result, workflow_id, 'INVALID_OPEN_LOOP_STATE',
                    state_error, total_moves=len(solution.moves))
            if not self._policy_client.wait_for_server(timeout_sec=0.5):
                return self._fail(
                    goal_handle, result, workflow_id, 'POLICY_SERVER_NOT_READY',
                    '/policy/infer action server에 연결할 수 없습니다.',
                    total_moves=len(solution.moves))
            open_state = self._copy_open_loop_state(goal.initial_open_loop_state)
            initial_cube_quat_wxyz = self._quat_normalize([
                open_state.cube_orientation.w, open_state.cube_orientation.x,
                open_state.cube_orientation.y, open_state.cube_orientation.z])
            leap_builder = LeapObservationBuilder()
            previous_goal_face = ''
        else:
            open_state = None
            initial_cube_quat_wxyz = None
            leap_builder = None
            previous_goal_face = ''

        plan = compile_workflow(solution.moves)
        result.completed_steps = 0
        result.completed_moves = 0
        total_moves = len(solution.moves)
        total_steps = len(plan)
        previous_hand = ''

        for step_index, step in enumerate(plan, start=1):
            if goal_handle.is_cancel_requested:
                result.error_code = 'CANCELLED'
                result.error_message = '실행이 hand primitive 경계에서 취소되었습니다.'
                goal_handle.canceled()
                self._publish_status(
                    goal.scan_id, result.execution_id, workflow_id,
                    CubeExecutionStatus.STATE_CANCELLED,
                    move_index=step['move_index'], total_moves=total_moves,
                    current_move=step['cube_move'], step_index=step_index - 1,
                    total_steps=total_steps, error_code=result.error_code,
                    message=result.error_message)
                return result

            if previous_hand and previous_hand != step['hand_id']:
                handoff_status = self._publish_status(
                    goal.scan_id, result.execution_id, workflow_id,
                    CubeExecutionStatus.STATE_HANDOFF,
                    move_index=step['move_index'], total_moves=total_moves,
                    current_move=step['cube_move'], step_index=step_index,
                    total_steps=total_steps, hand_id='', primitive_id='',
                    message=f"{previous_hand}에서 {step['hand_id']}로 handoff합니다.")
                self._publish_feedback(goal_handle, handoff_status)

            status = self._publish_status(
                goal.scan_id, result.execution_id, workflow_id,
                CubeExecutionStatus.STATE_EXECUTING,
                move_index=step['move_index'], total_moves=total_moves,
                current_move=step['cube_move'], step_index=step_index,
                total_steps=total_steps, hand_id=step['hand_id'],
                primitive_id=step['primitive_id'],
                message=('고정 사전학습 모델의 dry-run 추론과 hand primitive를 실행합니다.'
                         if goal.run_pretrained_policies else
                         'hand primitive dry-run을 실행합니다. 모터 명령은 비활성화되어 있습니다.'))
            self._publish_feedback(goal_handle, status)

            if goal.run_pretrained_policies:
                policy_id = _POLICY_BY_HAND[step['hand_id']]
                try:
                    direction_order_before = self._face_direction_order(
                        open_state, initial_cube_quat_wxyz)
                    target_direction_before = direction_order_before.index(
                        step['target_face'])
                    observation, goal_quat = self._build_open_loop_observation(
                        open_state, leap_builder, step, previous_goal_face)
                except (ValueError, FloatingPointError) as exc:
                    return self._fail(
                        goal_handle, result, workflow_id,
                        'OPEN_LOOP_OBSERVATION_FAILED', str(exc),
                        move_index=step['move_index'], total_moves=total_moves,
                        current_move=step['cube_move'], step_index=step_index,
                        total_steps=total_steps, hand_id=step['hand_id'],
                        primitive_id=step['primitive_id'])
                policy_result, policy_error = await self._infer_policy(
                    goal_handle, goal, result, step_index, step,
                    policy_id, observation)
                if policy_result is None or not policy_result.success:
                    if goal_handle.is_cancel_requested:
                        result.error_code = 'CANCELLED'
                        result.error_message = '정책 추론 경계에서 실행이 취소되었습니다.'
                        goal_handle.canceled()
                        return result
                    code = (policy_result.error_code if policy_result is not None
                            else 'POLICY_INFERENCE_FAILED')
                    message = (policy_result.error_message if policy_result is not None
                               else policy_error)
                    return self._fail(
                        goal_handle, result, workflow_id, code, message,
                        move_index=step['move_index'], total_moves=total_moves,
                        current_move=step['cube_move'], step_index=step_index,
                        total_steps=total_steps, hand_id=step['hand_id'],
                        primitive_id=step['primitive_id'])
                action_values = list(policy_result.action_values)
                result.policy_ids.append(policy_id)
                result.policy_step_indices.append(step_index)
                result.policy_observation_sizes.append(len(observation))
                result.policy_observations.extend(
                    np.asarray(observation, dtype=np.float64).tolist())
                result.policy_action_sizes.append(len(action_values))
                result.policy_action_values.extend(action_values)
                self._advance_open_loop_state(
                    open_state, leap_builder, step, action_values, goal_quat)
                try:
                    direction_order_after = self._face_direction_order(
                        open_state, initial_cube_quat_wxyz)
                    target_direction_after = direction_order_after.index(
                        step['target_face'])
                except (ValueError, FloatingPointError) as exc:
                    return self._fail(
                        goal_handle, result, workflow_id,
                        'FACE_DIRECTION_UPDATE_FAILED', str(exc),
                        move_index=step['move_index'], total_moves=total_moves,
                        current_move=step['cube_move'], step_index=step_index,
                        total_steps=total_steps, hand_id=step['hand_id'],
                        primitive_id=step['primitive_id'])
                result.face_direction_orders_before.append(
                    direction_order_before)
                result.face_direction_orders_after.append(direction_order_after)
                result.target_face_direction_indices_before.append(
                    target_direction_before)
                result.target_face_direction_indices_after.append(
                    target_direction_after)
                step['face_direction_order'] = direction_order_after
                step['target_face_direction_index'] = target_direction_after
                previous_goal_face = step['target_face']
                policy_status = self._publish_status(
                    goal.scan_id, result.execution_id, workflow_id,
                    CubeExecutionStatus.STATE_EXECUTING,
                    move_index=step['move_index'], total_moves=total_moves,
                    current_move=step['cube_move'], step_index=step_index,
                    total_steps=total_steps, hand_id=step['hand_id'],
                    primitive_id=step['primitive_id'],
                    message=f'{policy_id} deterministic 추론 완료; shadow action을 primitive로 전달합니다.')
                self._publish_feedback(goal_handle, policy_status)
                step['policy_id'] = policy_id
                step['policy_action_values'] = action_values

            hand_result, error_message = await self._dispatch_step(
                goal_handle, goal, result, step_index, step)
            if hand_result is None or not hand_result.success:
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.error_code = 'CANCELLED'
                    result.error_message = '실행이 hand primitive 경계에서 취소되었습니다.'
                    goal_handle.canceled()
                    self._publish_status(
                        goal.scan_id, result.execution_id, workflow_id,
                        CubeExecutionStatus.STATE_CANCELLED,
                        move_index=step['move_index'], total_moves=total_moves,
                        current_move=step['cube_move'], step_index=step_index,
                        total_steps=total_steps, hand_id=step['hand_id'],
                        primitive_id=step['primitive_id'],
                        error_code=result.error_code,
                        message=result.error_message)
                    return result
                error_code = (
                    hand_result.error_code if hand_result is not None
                    else 'HAND_NOT_READY')
                message = (
                    hand_result.error_message if hand_result is not None
                    else error_message)
                return self._fail(
                    goal_handle, result, workflow_id, error_code, message,
                    move_index=step['move_index'], total_moves=total_moves,
                    current_move=step['cube_move'], step_index=step_index,
                    total_steps=total_steps, hand_id=step['hand_id'],
                    primitive_id=step['primitive_id'])

            result.completed_steps = step_index
            if step['primitive_id'] == 'rotate_upper_layer':
                result.completed_moves = step['move_index']
            previous_hand = step['hand_id']

        result.success = True
        result.error_code = ''
        result.error_message = (
            ('풀이를 hand primitive 단계로 전개하고 고정 모델 추론 및 두 adapter의 '
             'dry-run 응답을 확인했습니다. 상태 갱신은 이상적 계획 가정이며 물리 동작은 하지 않았습니다.'
             if goal.run_pretrained_policies else
             '풀이를 hand primitive 단계로 전개하고 두 adapter의 dry-run 응답을 확인했습니다. '
             '물리 동작은 하지 않았습니다.'))
        goal_handle.succeed()
        final_status = self._publish_status(
            goal.scan_id, result.execution_id, workflow_id,
            CubeExecutionStatus.STATE_COMPLETED, move_index=total_moves,
            total_moves=total_moves, step_index=total_steps,
            total_steps=total_steps, message=result.error_message)
        self._publish_feedback(goal_handle, final_status)
        return result

    async def _dispatch_step(
            self, parent_goal_handle, parent_goal, parent_result, step_index, step):
        client = self._hand_clients[step['hand_id']]
        if not client.wait_for_server(timeout_sec=0.5):
            return None, f"{step['hand_id']} action server에 연결할 수 없습니다."

        request = ExecuteHandPrimitive.Goal()
        request.scan_id = parent_goal.scan_id
        request.execution_id = parent_result.execution_id
        request.move_index = step['move_index']
        request.step_index = step_index
        request.cube_move = step['cube_move']
        request.target_face = step['target_face']
        request.face_direction_order = step.get('face_direction_order', '')
        request.target_face_direction_index = step.get(
            'target_face_direction_index', 0)
        request.clockwise_quarter_turns = step['clockwise_quarter_turns']
        request.primitive_id = step['primitive_id']
        request.dry_run = True
        request.policy_id = step.get('policy_id', '')
        request.policy_action_values = step.get('policy_action_values', [])

        try:
            child_goal = await client.send_goal_async(request)
            if not child_goal.accepted:
                return None, f"{step['hand_id']}가 primitive 요청을 거부했습니다."

            result_future = child_goal.get_result_async()
            cancellation = {'sent': False}

            def cancel_child_if_requested():
                if parent_goal_handle.is_cancel_requested and not cancellation['sent']:
                    cancellation['sent'] = True
                    child_goal.cancel_goal_async()

            cancel_timer = self.create_timer(
                0.05, cancel_child_if_requested, callback_group=self._callback_group)
            try:
                wrapped_result = await result_future
            finally:
                cancel_timer.cancel()
                self.destroy_timer(cancel_timer)
            return wrapped_result.result, ''
        except Exception as exc:  # ROS action transport errors become action results.
            return None, f"{step['hand_id']} action 오류: {exc}"

    async def _infer_policy(
            self, parent_goal_handle, parent_goal, parent_result, step_index,
            step, policy_id, observation):
        request = InferHandPolicy.Goal()
        request.scan_id = parent_goal.scan_id
        request.execution_id = parent_result.execution_id
        request.step_index = step_index
        request.hand_id = step['hand_id']
        request.policy_id = policy_id
        request.observation = np.asarray(observation, dtype=np.float64).tolist()
        try:
            goal_handle = await self._policy_client.send_goal_async(request)
            if not goal_handle.accepted:
                return None, '정책 추론 액션 목표가 거부됐습니다.'
            result_future = goal_handle.get_result_async()
            cancellation = {'sent': False}

            def cancel_policy_if_requested():
                if (parent_goal_handle.is_cancel_requested
                        and not cancellation['sent']):
                    cancellation['sent'] = True
                    goal_handle.cancel_goal_async()

            timer = self.create_timer(
                0.05, cancel_policy_if_requested,
                callback_group=self._callback_group)
            try:
                wrapped = await result_future
            finally:
                timer.cancel()
                self.destroy_timer(timer)
            return wrapped.result, ''
        except Exception as exc:
            return None, f'정책 추론 action 오류: {exc}'

    @staticmethod
    def _validate_open_loop_state(state):
        quaternion = np.array([
            state.cube_orientation.x, state.cube_orientation.y,
            state.cube_orientation.z, state.cube_orientation.w])
        face_normals = np.asarray(
            state.face_normals_cube_frame_urfdlb, dtype=np.float64).reshape(6, 3)
        palm_minus_cube = np.asarray(
            state.palm_minus_cube_position_m, dtype=np.float64)
        leap_positions = np.asarray(
            state.leap_command_positions_rad, dtype=np.float64)
        custom_positions = np.asarray(
            state.custom_policy_positions, dtype=np.float64)
        custom_velocities = np.asarray(
            state.custom_policy_velocities, dtype=np.float64)
        values = np.concatenate([
            quaternion, face_normals.ravel(), palm_minus_cube, leap_positions,
            custom_positions, custom_velocities])
        if not np.all(np.isfinite(values)):
            return '초기 open-loop 상태의 모든 값은 유한해야 합니다.'
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-8 or abs(norm - 1.0) > 1e-3:
            return 'cube_orientation은 정규화된 x,y,z,w quaternion이어야 합니다.'
        normal_lengths = np.linalg.norm(face_normals, axis=1)
        if np.any(np.abs(normal_lengths - 1.0) > 1e-3):
            return 'URFDLB 면 법선은 cube frame에서 각각 단위 벡터여야 합니다.'
        if (abs(float(face_normals[0] @ face_normals[3]) + 1.0) > 1e-3
                or abs(float(face_normals[1] @ face_normals[4]) + 1.0) > 1e-3
                or abs(float(face_normals[2] @ face_normals[5]) + 1.0) > 1e-3):
            return 'URFDLB의 마주 보는 면 법선은 서로 반대 방향이어야 합니다.'
        if any(abs(float(face_normals[a] @ face_normals[b])) > 1e-3
               for a, b in ((0, 1), (0, 2), (1, 2))):
            return '서로 다른 큐브 축의 면 법선은 직교해야 합니다.'
        if (np.any(leap_positions < _LEAP_LOW)
                or np.any(leap_positions > _LEAP_HIGH)):
            return 'LEAP 초기 관절 위치가 참고 정책의 안전 범위를 벗어났습니다.'
        if (np.any(custom_positions < _CUSTOM_LOW)
                or np.any(custom_positions > _CUSTOM_HIGH)):
            return 'Custom 초기 정책 위치가 CTRL 범위를 벗어났습니다.'
        return None

    @staticmethod
    def _copy_open_loop_state(source):
        state = OpenLoopPolicyState()
        state.header = source.header
        state.cube_orientation = source.cube_orientation
        state.face_normals_cube_frame_urfdlb = list(
            source.face_normals_cube_frame_urfdlb)
        state.palm_minus_cube_position_m = list(source.palm_minus_cube_position_m)
        state.leap_command_positions_rad = list(source.leap_command_positions_rad)
        state.custom_policy_positions = list(source.custom_policy_positions)
        state.custom_policy_velocities = list(source.custom_policy_velocities)
        return state

    @staticmethod
    def _quat_normalize(q):
        q = np.asarray(q, dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if norm < 1e-12 or not np.isfinite(norm):
            raise ValueError('Quaternion norm is zero or non-finite.')
        return q / norm

    @staticmethod
    def _quat_multiply(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ], dtype=np.float64)

    @staticmethod
    def _rotate_vector(q, vector):
        from leap_hand_control.observation_builder import quat_to_mat
        return quat_to_mat(q) @ np.asarray(vector, dtype=np.float64)

    @classmethod
    def _face_direction_order(cls, state, initial_cube_quat_wxyz):
        """Return initial face labels occupying initial URFDLB direction slots."""
        orientation = state.cube_orientation
        current = cls._quat_normalize([
            orientation.w, orientation.x, orientation.y, orientation.z])
        initial_inverse = np.asarray([
            initial_cube_quat_wxyz[0], -initial_cube_quat_wxyz[1],
            -initial_cube_quat_wxyz[2], -initial_cube_quat_wxyz[3]])
        relative = cls._quat_normalize(
            cls._quat_multiply(initial_inverse, current))
        normals = np.asarray(
            state.face_normals_cube_frame_urfdlb,
            dtype=np.float64).reshape(6, 3)
        direction_slots = [''] * len(_FACE_ORDER)
        for face_index, face in enumerate(_FACE_ORDER):
            current_normal = cls._rotate_vector(relative, normals[face_index])
            scores = normals @ current_normal
            slot_index = int(np.argmax(scores))
            if scores[slot_index] < 0.99:
                raise ValueError(
                    '현재 quaternion이 초기 URFDLB 방향 슬롯에 충분히 정렬되지 않아 '
                    '면 방향 인덱스를 단일 순열로 나타낼 수 없습니다.')
            if direction_slots[slot_index]:
                raise ValueError(
                    '두 큐브 면이 같은 방향 슬롯에 대응되어 면 인덱스가 모호합니다.')
            direction_slots[slot_index] = face
        if any(not face for face in direction_slots):
            raise ValueError('면 방향 인덱스에 빈 슬롯이 남았습니다.')
        return ''.join(direction_slots)

    @classmethod
    def _face_up_goal(cls, cube_quat_xyzw, face_normal_cube):
        cube_wxyz = cls._quat_normalize([
            cube_quat_xyzw[3], cube_quat_xyzw[0],
            cube_quat_xyzw[1], cube_quat_xyzw[2]])
        normal = cls._rotate_vector(cube_wxyz, face_normal_cube)
        normal /= np.linalg.norm(normal)
        up = np.array([0.0, 0.0, 1.0])
        dot = float(np.clip(normal @ up, -1.0, 1.0))
        if dot > 1.0 - 1e-10:
            delta = np.array([1.0, 0.0, 0.0, 0.0])
        elif dot < -1.0 + 1e-10:
            axis = np.cross(normal, np.array([1.0, 0.0, 0.0]))
            if np.linalg.norm(axis) < 1e-8:
                axis = np.cross(normal, np.array([0.0, 1.0, 0.0]))
            axis /= np.linalg.norm(axis)
            delta = np.concatenate([[0.0], axis])
        else:
            cross = np.cross(normal, up)
            delta = cls._quat_normalize(np.concatenate([[1.0 + dot], cross]))
        return cls._quat_normalize(cls._quat_multiply(delta, cube_wxyz))

    @classmethod
    def _build_open_loop_observation(
            cls, state, builder, step, previous_goal_face):
        if step['hand_id'] == 'leap_hand':
            orientation = state.cube_orientation
            cube_xyzw = np.array([
                orientation.x, orientation.y, orientation.z, orientation.w])
            cube_wxyz = cls._quat_normalize(
                [cube_xyzw[3], cube_xyzw[0], cube_xyzw[1], cube_xyzw[2]])
            face_normals = np.asarray(
                state.face_normals_cube_frame_urfdlb,
                dtype=np.float64).reshape(6, 3)
            face_normal = face_normals[_FACE_ORDER.index(step['target_face'])]
            goal = cls._face_up_goal(cube_xyzw, face_normal)
            if previous_goal_face != step['target_face']:
                builder.fill_orientation_history(cube_wxyz, goal)
            q = np.asarray(state.leap_command_positions_rad, dtype=np.float64)
            palm_minus_cube = np.asarray(
                state.palm_minus_cube_position_m, dtype=np.float64)
            obs = builder.build(
                q, q, palm_minus_cube, np.zeros(3), cube_wxyz, goal)
            return obs, goal
        positions = np.asarray(state.custom_policy_positions, dtype=np.float64)
        velocities = np.asarray(state.custom_policy_velocities, dtype=np.float64)
        return np.concatenate([positions, velocities]), None

    @classmethod
    def _advance_open_loop_state(
            cls, state, builder, step, action_values, goal_quat):
        action = np.asarray(action_values, dtype=np.float64)
        if step['hand_id'] == 'leap_hand':
            if action.shape != (16,):
                raise ValueError('LEAP policy action must have 16 values.')
            q = np.asarray(state.leap_command_positions_rad, dtype=np.float64)
            q = np.clip(q + _LEAP_ACTION_SCALE * action, _LEAP_LOW, _LEAP_HIGH)
            state.leap_command_positions_rad = q.tolist()
            builder.set_last_action(action)
            # Idealized open-loop assumption: successful face-up primitive reaches goal pose.
            state.cube_orientation.x = float(goal_quat[1])
            state.cube_orientation.y = float(goal_quat[2])
            state.cube_orientation.z = float(goal_quat[3])
            state.cube_orientation.w = float(goal_quat[0])
        else:
            if action.shape != (4,):
                raise ValueError('Custom policy action must have 4 values.')
            action = np.clip(action, -1.0, 1.0)
            targets = _CUSTOM_LOW + (action + 1.0) * 0.5 * (_CUSTOM_HIGH - _CUSTOM_LOW)
            state.custom_policy_positions = targets.tolist()
            state.custom_policy_velocities = [0.0] * 4

    def _fail(
            self, goal_handle, result, workflow_id, error_code, message,
            move_index=0, total_moves=0, current_move='', step_index=0,
            total_steps=0, hand_id='', primitive_id=''):
        result.success = False
        result.error_code = error_code
        result.error_message = message
        goal_handle.abort()
        status = self._publish_status(
            result.scan_id, result.execution_id, workflow_id,
            CubeExecutionStatus.STATE_FAILED, move_index=move_index,
            total_moves=total_moves, current_move=current_move,
            step_index=step_index, total_steps=total_steps,
            hand_id=hand_id, primitive_id=primitive_id,
            error_code=error_code, message=message)
        self._publish_feedback(goal_handle, status)
        return result

    @staticmethod
    def _copy_solution(source):
        copied = CubeSolution()
        copied.header = source.header
        copied.scan_id = source.scan_id
        copied.facelets = source.facelets
        copied.moves = list(source.moves)
        return copied

    @staticmethod
    def _publish_feedback(goal_handle, status):
        feedback = ExecuteCubeSolution.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    def _publish_status(
            self, scan_id, execution_id, workflow_id, state, move_index=0,
            total_moves=0, current_move='', step_index=0, total_steps=0,
            hand_id='', primitive_id='', error_code='', message=''):
        status = CubeExecutionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.scan_id = scan_id
        status.execution_id = execution_id
        status.workflow_id = workflow_id
        status.state = state
        status.move_index = move_index
        status.total_moves = total_moves
        status.current_move = current_move
        status.step_index = step_index
        status.total_steps = total_steps
        status.hand_id = hand_id
        status.primitive_id = primitive_id
        status.error_code = error_code
        status.message = message
        self._status_publisher.publish(status)
        return status


def main(args=None):
    rclpy.init(args=args)
    node = CubeSequenceExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
