# 큐브 조작 제어

`cube_execution`은 `/cube/solution`을 받아 실행 요청을 기다리고, 명시적인 ROS 2 액션 목표를 받으면 Kociemba 풀이 토큰을 두 손의 추상 동작으로 전개합니다. `leap_hand_control`과 `custom_hand_control`은 각각 손 primitive 액션 서버를 제공합니다.

현재 primitive 동작은 **dry-run 전용**입니다. 선택적으로 `/cube/execute_solution`에서 고정 모델을 primitive마다 한 번 추론하고 그 shadow action을 hand action에 전달할 수 있습니다. 초기 상태는 action goal에 명시적으로 제공하는 계획값이며 센서 입력이나 실시간 pose가 아닙니다. 추론값과 계획 상태 갱신은 모터 driver로 연결되지 않습니다.

## ROS 2 실행 파이프라인의 역할

이 프로젝트에서 ROS 2는 상황을 자율적으로 해석해 계획을 만드는 계층이 아니라, 명시된 `/cube/solution`을 순서가 정해진 hand primitive로 바꾸고 각 손 action endpoint를 차례로 호출하는 실행 파이프라인입니다. 현재 `cube_sequence_executor`가 이 순서를 수행하며, 상태 추정이나 다음 동작 선택을 하지 않습니다.

강화학습은 사전에 학습을 마친 모델을 사용합니다. ROS 2 실행 중에는 새로 학습하지 않고, 명시된 workflow 단계에 맞는 등록 모델을 선택해 추론을 호출하는 구조입니다. 즉 workflow는 순서와 사용할 hand/model을 정하고, 선택된 모델은 자기 입력 규약에 따라 action 값을 계산합니다. 상황을 해석해 새로운 계획이나 정책을 생성하는 온라인 학습기로 동작하지 않습니다.

현재 workflow는 `orient_face_up_and_support`에 `leap_faceup_v1`, `grasp_upper_layer`·`rotate_upper_layer`·`release_upper_layer`에 `custom_finger_ppo_v1`을 고정 지정합니다. 각 손에 등록된 checkpoint는 하나이며 상황에 따른 모델 자동 선택은 하지 않습니다. 각 primitive에서 해당 모델을 한 번 호출하고 policy ID와 action 값을 hand dry-run action으로 전달합니다.

모델 선택과 observation 입력은 별개입니다. Custom Hand 모델은 8개 손가락 위치·속도로 추론하고 큐브 3D pose는 직접 입력으로 받지 않습니다. LEAP 모델은 학습된 108차원 입력에 관절 상태와 손바닥·큐브 상대 pose 및 목표 자세가 포함되므로, **LEAP 모델을 선택한 단계에서만** 그 observation 계약에 맞는 값이 필요합니다. 이는 실행 파이프라인이 상황을 자율 판단한다는 뜻이 아닙니다. 임의의 0 벡터나 합성 demo 값은 추론 경로 점검에만 쓰며 실제 동작 검증으로 보지 않습니다.

LEAP의 108차원 입력은 전달된 planned joint positions, palm-minus-cube 위치, 현재 큐브 quaternion과 목표 면을 위로 둔 quaternion, observation history로 구성합니다. Custom의 8차원 입력은 전달된 4개 policy 좌표와 4개 velocity입니다. `OpenLoopPolicyState.face_normals_cube_frame_urfdlb`에는 실제 큐브 frame에서 측정한 URFDLB 순서의 단위 면 법선 6개가 필요합니다. executor는 면 법선 대응을 내부에서 가정하지 않으며, 제공된 값이 단위·대향·직교 조건을 만족하는지 확인합니다.

추론 뒤 계획 상태는 open-loop 가정으로 갱신합니다. LEAP 명령 위치는 `q_cmd + 0.2 × action`을 참고 joint 한계로 제한하고, face-up primitive가 성공하면 큐브가 계산된 목표 방향에 도달했다고 기록합니다. Custom action은 참고 `CTRL` 범위를 이용해 목표 위치로 변환하며 위치가 즉시 도달하고 velocity가 0이 되었다고 기록합니다. 이후 primitive 입력에 이 값이 재사용됩니다. 이 과정은 관측 기반 상태 추정, 물리 시뮬레이션, 실행 성공의 증거가 아닙니다.

`run_pretrained_policies=false`이면 기존 dry-run action 검증만 수행합니다. true이면 유한하고 범위 내인 초기 계획 상태와 `/policy/infer` 서버가 필요합니다. 결과의 `policy_ids`, `policy_step_indices`, `policy_observation_sizes`, `policy_observations`, `policy_action_sizes`, `policy_action_values`는 primitive 순서입니다. 관측·action 크기 배열로 각각의 평탄화된 값을 나눠, 이전 action으로 갱신된 상태가 다음 observation에 반영됐는지 확인할 수 있습니다. 실제 입력 토픽이나 모터 제어는 연결되지 않습니다.

같은 모드에서 `face_direction_orders_before/after`는 URFDLB 방향 슬롯에 현재 들어 있는 초기 면 라벨의 순열을 primitive 전·후로 기록합니다. `target_face_direction_indices_before/after`는 논리 `target_face`가 해당 순열의 어느 방향 슬롯에 있는지 0부터 나타냅니다. LEAP가 큐브 자세를 갱신하면 순열도 갱신되고, Custom의 레이어 회전 primitive에서는 큐브 전체 방향이 유지된다는 계획 가정에 따라 순열이 유지됩니다. solver의 원래 `target_face` 라벨과 54자 facelets 자체는 재배열하지 않습니다. 실제 조립 좌표계·법선이 검증되지 않았으므로, dry-run 예시의 순열값을 실물 장착 결과로 간주하지 않습니다.

## ROS 인터페이스

| 이름 | 타입 | 방향·역할 |
|---|---|---|
| `/cube/solution` | `system_interfaces/msg/CubeSolution` | 인식·풀이에서 실행 노드로 상태와 `moves`를 전달합니다. 실행 노드는 메시지를 검증·저장하지만 수신만으로 손을 움직이지 않습니다. |
| `/cube/execute_solution` | `system_interfaces/action/ExecuteCubeSolution` | `scan_id`, `workflow_id`, `dry_run`을 받습니다. 선택적으로 `run_pretrained_policies`와 `initial_open_loop_state`를 받아 primitive마다 등록 모델을 추론합니다. |
| `/cube/execution_status` | `system_interfaces/msg/CubeExecutionStatus` | 풀이 수신, 대기, 검증, handoff, primitive 진행, 완료·실패·취소를 발행합니다. |
| `/leap_hand/execute_primitive` | `system_interfaces/action/ExecuteHandPrimitive` | 하단 LEAP Hand의 방향 조정·지지 primitive 요청을 받습니다. |
| `/custom_hand/execute_primitive` | `system_interfaces/action/ExecuteHandPrimitive` | 상단 Custom Hand의 파지·레이어 회전·해제 primitive 요청을 받습니다. |
| `/policy/infer` | `system_interfaces/action/InferHandPolicy` | executor가 hand 역할에 고정된 모델과 open-loop observation을 전달하고 shadow action을 받습니다. |
| `/leap_hand/policy_input` → `/leap_hand/policy_action` | `LeapPolicyInput` → `HandPolicyAction` | LEAP 센서 입력을 조립해 108→16 shadow 추론을 수행합니다. |
| `/custom_hand/policy_input` → `/custom_hand/policy_action` | `CustomPolicyInput` → `HandPolicyAction` | 보정된 Custom 상태에서 8→4 shadow 추론을 수행합니다. |
| `/cube/policy_goal` | `system_interfaces/msg/CubePolicyGoal` | 보정된 face-to-frame 변환기가 현재 move의 목표 quaternion을 제공합니다. |
| `/cube/pose`, `/leap_hand/palm_pose`, `/leap_hand/joint_states`, `/leap_hand/commanded_joint_states`, `/custom_hand/policy_joint_states` | 각각 `PoseStamped` 또는 `JointState` | 외부 pose·driver 입력입니다. `system_bringup/policy_input_bridge`가 workflow 상태와 최신 입력을 정책 입력으로 묶습니다. |

`ExecuteHandPrimitive` 목표에는 `scan_id`, `execution_id`, 이동·단계 인덱스, 원래의 `cube_move`, 논리 좌표계의 `target_face`, 현재 방향 순열 `face_direction_order`, 순열 안에서 해당 면의 슬롯 `target_face_direction_index`, `clockwise_quarter_turns`, `primitive_id`, `dry_run`이 포함됩니다. 방향 순열이 있으면 각 hand 서버는 URFDLB의 유효한 순열인지, 해당 인덱스의 면이 `target_face`와 같은지 확인합니다. 회전 방향은 `target_face` 바깥에서 면을 바라볼 때 시계 방향을 양수로 표현합니다. 회전 primitive의 값은 `1`, `-1`, `2`이며 그 외 primitive는 `0`입니다.

## 풀이 토큰과 손 primitive 순서

현재 workflow ID는 `two_hand_layer_turn_v1`입니다. 각 Kociemba 이동은 다음 네 단계로 전개되고, 앞 단계가 성공한 뒤 다음 액션을 보냅니다.

1. `leap_hand`: `orient_face_up_and_support` — 조작 대상 면을 위로 맞추고 큐브를 아래에서 지지합니다.
2. `custom_hand`: `grasp_upper_layer` — 위쪽 손으로 레이어를 파지합니다.
3. `custom_hand`: `rotate_upper_layer` — 요청된 방향과 1/2 회전량으로 상단 레이어를 회전합니다.
4. `custom_hand`: `release_upper_layer` — 파지를 해제합니다.

예를 들어 `R'`는 `target_face=R`, `clockwise_quarter_turns=-1`로, `F2`는 `target_face=F`, `clockwise_quarter_turns=2`로 전달됩니다. 각 이동의 대상 면은 Kociemba 논리 `URFDLB` 좌표계 값입니다.

실행 노드는 한 번에 하나의 풀이 실행만 받습니다. 각 단계의 진행은 액션 feedback과 `/cube/execution_status`에 기록합니다. 손 어댑터가 실패하거나 연결되지 않으면 이후 단계를 중단합니다. 취소 요청은 현재 손 primitive에 전달하며 primitive 경계에서 상위 액션의 취소 결과를 반환합니다.

## 현재 확인된 범위

- Windows 격리 빌드에서 `system_interfaces`, `cube_execution`, `custom_hand_control`, `leap_hand_control` 네 패키지가 빌드됐습니다.
- `ExecuteHandPrimitive` 생성 인터페이스와 세 패키지 실행 항목을 `ros2 interface show`, `ros2 pkg executables`로 확인했습니다.
- 구현상 두 손 서버는 입력 필드를 검증하고 `dry_run=true` 요청에 성공 응답을 보냅니다. `dry_run=false`는 실행 노드와 손 어댑터가 거부합니다.
- 첫 runtime dry-run에서 발생한 `no running event loop` 문제를 수정하고 네 패키지를 다시 빌드했습니다.
- 사용자가 Windows에서 세 노드와 액션 클라이언트를 실행해 `/cube/solution` 입력부터 두 손 액션 응답까지 확인했습니다. 3개 이동의 결과는 `success: true`, `completed_moves: 3`, `completed_steps: 12`였습니다. 확인 범위는 dry-run이며 실제 모터 구동은 아닙니다.

## LEAP 정책 소프트웨어 어댑터

`leap_hand_control`에는 참조 배포물과 같은 NumPy MLP 추론 형식을 읽는 `LeapPolicyAdapter`가 있습니다. LEAP 정책 `policy_np.npz`는 참조 디렉토리에서 middleware-bridge의 `src/control/leap_hand_control/models/policy_np.npz`로 복사했고, SHA-256은 `DC220D146BC5532C4AE09907C6D6BBC887335389D215AFAAFB69EEAB679560E7`입니다. Windows 빌드 설치 시 패키지 share의 `models` 아래에도 포함됩니다.

어댑터는 108개 관측값을 정규화한 뒤 은닉층 ReLU와 마지막 층의 첫 16개 출력에 `tanh`를 적용합니다. 이 규약은 `right_rl_with_mujoco/deploy/policy_np.py`의 추론 경로를 따릅니다. 동작 중 모터 API나 Dynamixel SDK를 불러오지 않습니다.

### LEAP checkpoint 분석

현재 middleware의 `models/policy_np.npz`는 참고 배포물 `right_rl_with_mujoco/policy/policy_np.npz`와 SHA-256이 모두 `DC220D146BC5532C4AE09907C6D6BBC887335389D215AFAAFB69EEAB679560E7`로 일치합니다. 파일 안에는 정규화 `mean/std` 각 108개, action 크기 16, 그리고 하나의 MLP 파라미터 세트가 있습니다. 층 크기는 `108 → 512 → 256 → 128 → 32`이며 마지막 32개 중 첫 16개를 action 평균으로 사용하고 `tanh`를 적용합니다. 배포 추론은 결정적(deterministic)입니다. 모델 내부에 면별 네트워크, mixture-of-experts, 분류기 또는 입력에 따른 checkpoint router는 보이지 않습니다.

따라서 **배포 artifact는 단일 정책 모델**입니다. 단일 목표만 처리하는 모델이라는 뜻은 아닙니다. 학습 환경은 하나의 정책에 여러 목표 방향을 입력해 학습합니다. 목표 방향은 별도 one-hot으로 주지 않고 관측 안의 큐브-목표 상대 자세 오차로 표현합니다(`face_onehot_obs=False`). 학습 설정과 증류 실행기는 목표면 인덱스 1–5를 사용하며, 연쇄 평가에서는 현재 위를 향한 면을 제외하고 다음 목표면을 고릅니다. 배포 runner도 `--face`로 목표를 정하거나 연쇄 실행에서 목표면을 바꿉니다. 이때 바뀌는 것은 **같은 정책의 목표 입력**이지 선택되는 모델이 아닙니다.

학습 기록에서 확인되는 생성 경로는 다음과 같습니다.

1. PPO 교사 정책과 결합한 residual SAC 학생을 2개 seed로 학습합니다. 실행 스크립트는 seed당 4천만 step을 설정하고 각 seed의 마지막 checkpoint 후보 3개씩을 연쇄 평가해 최고 SAC checkpoint를 선택합니다. `BEST_sac_checkpoint.txt`와 `TEACHER_checkpoint.txt`에는 사용했다고 기록된 외부 Linux checkpoint 경로가 있습니다. 해당 checkpoint 본체와 학습 로그는 현재 workspace에 없어 기록된 출처를 독립 재현·검증할 수는 없습니다.
2. 선택된 SAC 학생과 PPO 교사의 결합 동작을 DAgger로 하나의 MLP에 증류합니다. 스크립트는 4개 토크 조건(`0.09`, `0.11`, `0.16`, `0.21` N·m)에서 데이터를 모으고, 목표면별 환경을 구성합니다.
3. 증류된 단일 MLP를 `policy_np.npz`로 내보냅니다. 관측 정규화 통계도 파일에 포함하며, 참고 export 코드는 JAX와 NumPy 결정적 추론 출력의 일치를 확인하도록 작성돼 있습니다.

참고 가이드의 시뮬레이션 결과에서는 증류 모델이 교사 PPO보다 낮았습니다. 연속 목표 성공 횟수는 토크 `0.11 N·m`에서 `5.98` 대 `7.19`, `0.16 N·m`에서 `10.89` 대 `12.50`, `0.21 N·m`에서 `13.52` 대 `14.87`입니다(각각 증류 모델 대 교사). 그러므로 이 artifact를 “여러 모델 중 성능에 따라 선택된 모델”이라고 부를 근거는 현재 자료에서 확인되지 않습니다. 확인 가능한 것은 SAC 후보 중 하나를 고른 뒤 PPO 교사와 함께 증류해 만든 단일 배포망이라는 점입니다. 수치는 참고 학습 저장소 문서의 시뮬레이션 결과이며 이 workspace에서 재실행한 결과는 아닙니다.

정리하면 workflow가 선택할 대상은 현재 여러 LEAP checkpoint가 아니라 **하나의 LEAP 정책**입니다. 목표면에 따라 정책 입력값이 달라져 action이 달라질 수 있지만 모델 교체는 아닙니다. Kociemba의 `U/R/F/D/L/B`를 모델의 목표 quaternion 순서 `+z/-z/+x/-x/+y/-y`에 매핑하는 보정은 별도이며 아직 확인되지 않았습니다.

기본 LEAP primitive 서버를 실행하면 설치된 정책 파일을 열어 네트워크 모양과 가중치가 유효한지 확인합니다.

```powershell
ros2 run leap_hand_control leap_hand_primitive_server
```

primitive 서버 로그에 `LEAP 정책 파일 형식과 108→16 추론기를 준비했습니다`가 나타나면 모델 로딩까지 된 것입니다. 이것은 모델 추론 결과나 하드웨어 동작 검증은 아닙니다. 추론 shadow node는 `LeapPolicyInput` subscriber를 갖지만 카메라 pose와 LEAP joint state를 해당 계약으로 발행하는 upstream node는 없습니다. primitive 액션 목표에도 관측 데이터가 없습니다. 그래서 `dry_run=false` 요청은 정책이 로드되어도 `POSE_SOURCE_UNAVAILABLE`로 거부됩니다. 상위 `cube_sequence_executor`도 실제 구동을 계속 차단합니다.

예시 calibration JSON의 `policies.leap.model_path`에는 비워 둔 경로에 사용자의 로컬 절대 경로를 기록하고, `sha256`은 파일을 교체할 때만 실제 값에 맞춰 갱신합니다. 모델 파일을 읽을 수 있어도 `verified`와 `observation_schema_verified`는 실측·입출력 검증 전까지 `false`로 둡니다.

## 정책 ROS 입력 계약과 Shadow 추론

두 정책의 센서·상태 입력을 ROS 메시지로 명시하고, 계산 결과는 모터 드라이버가 아닌 `HandPolicyAction` 토픽으로 발행합니다. 두 출력 메시지는 `shadow_only: true`입니다.

| 토픽 | 입력 타입 | 의미 |
|---|---|---|
| `/leap_hand/policy_input` | `system_interfaces/msg/LeapPolicyInput` | timestamp가 있는 큐브 pose, LEAP palm pose, 16개 LEAP 관절 상태, 직전 명령 관절각, 목표 quaternion을 한 샘플로 전달합니다. |
| `/leap_hand/policy_action` | `system_interfaces/msg/HandPolicyAction` | LEAP 시뮬 관절 순서 16개에 대한 정규화 정책 출력입니다. 모터에 전달되지 않습니다. |
| `/custom_hand/policy_input` | `system_interfaces/msg/CustomPolicyInput` | Custom 정책 좌표계의 손가락 4개 위치와 속도를 전달합니다. 순서는 `right, front, left, back`입니다. |
| `/custom_hand/policy_action` | `system_interfaces/msg/HandPolicyAction` | 손가락 순서의 정규화 출력 4개입니다. 손가락 위치 변환이나 손목 Feetech 제어를 수행하지 않습니다. |

LEAP 입력의 `cube_pose`, `palm_pose`와 `leap_joint_state` timestamp는 모두 유효하고 최근 값이어야 합니다. 입력 생산자는 구성요소 측정값을 같은 샘플로 동기화해야 합니다. 두 pose의 `header.frame_id`는 설정한 `world_frame`과 같아야 하며 pose 위치 단위는 m, quaternion 순서는 ROS 표준 x,y,z,w입니다. `joint_state.name`은 참조 배포 코드의 16개 이름을 포함해야 하고 위치는 rad 단위입니다. `commanded_joint_positions`는 같은 순서의 직전 명령각(rad)이며 `goal_orientation`은 목표 cube quaternion입니다. 입력 조립기는 참조 순서로 관절을 재배열하고 108차원 관측의 이력을 구성합니다. 입력 bridge는 기본 100 ms freshness를 적용하고, 원본 LEAP runner의 pose 기준은 150 ms입니다. 목표 quaternion과 Kociemba 면 문자의 대응은 아직 보정·연결되지 않았으므로 이 입력의 생산자가 검증된 변환을 제공해야 합니다.

108개 값의 순서는 관절각 16, 관절오차 이력 48, 손바닥-큐브 위치오차 이력 9, cube 자세오차 이력 18, 직전 행동 16, 자세오차 scalar 1입니다. 이력은 최근 샘플부터 저장합니다.

```powershell
ros2 interface show system_interfaces/msg/LeapPolicyInput
ros2 interface show system_interfaces/msg/CustomPolicyInput
ros2 interface show system_interfaces/msg/HandPolicyAction
ros2 interface show system_interfaces/msg/CubePolicyGoal
ros2 run leap_hand_control leap_policy_shadow --ros-args -p world_frame:=YOUR_MEASURED_FRAME
ros2 run custom_hand_control custom_hand_policy_shadow
```

`YOUR_MEASURED_FRAME`은 카메라/로봇 보정으로 실제 확인한 공통 프레임 이름으로 바꿉니다. `system_bringup/policy_input_bridge`는 외부 source topics를 구독합니다. LEAP 실행 단계의 `CubeExecutionStatus`와 scan/execution ID, `CubePolicyGoal`, pose frame, 각 센서 timestamp를 확인한 다음 `/leap_hand/policy_input`을 발행합니다. 실행 상태와 목표 quaternion은 해당 primitive 동안 유지되는 문맥값으로 다룹니다. 새 pose·joint 샘플 묶음마다 한 번만 발행하고, 정책 입력 구독 큐는 최신 샘플 하나만 보관합니다. Custom 실행 단계에서도 상태와 ID가 유지되는 동안 새 `/custom_hand/policy_joint_states` 샘플당 한 번만 `/custom_hand/policy_input`을 발행합니다. `leap_source_joint_names`, `leap_command_joint_names`, `custom_source_joint_names` 매개변수의 순서는 실제 source name을 정책 관절·손가락 순서에 대응하도록 지정해야 합니다.

현재 middleware에는 실측 `/cube/pose`, palm pose, LEAP joint/command state, Custom policy-coordinate state 또는 `CubePolicyGoal` publisher가 없습니다. YOLO의 2D OBB를 3D pose로 바꾸는 변환도 구현하지 않았습니다. Dynamixel joint feedback은 보정된 관절각과 link geometry가 있을 때 손가락/손의 순기구학에 쓸 수 있지만, 그 값만으로 큐브의 절대 위치·자세를 정하지는 못합니다. 큐브 pose를 joint feedback에서 추정하려면 손·큐브 기준 frame, 접촉/파지 관계, slip 판정과 object-to-hand calibration이 추가로 검증돼야 합니다. 따라서 실제 정책 입력을 내기 위해서는 보정된 pose provider, 손바닥 pose와 motor-state publisher, 목표면 quaternion 변환기가 필요합니다. Custom 입력은 motor tick이 아니라 참고 `core.observation`의 calibration 이후 policy position·velocity 좌표여야 합니다. 미검증 mapping으로 값을 임의 변환하지 않습니다.

Custom 모델은 Stable-Baselines3 PPO actor의 8→64→64→4 구조를 CPU에서 weights-only로 읽습니다. 기본 Tanh 은닉층과 action-space `[-1,1]` clipping을 적용합니다. 정책 후보 파일은 middleware 내부에 복사했지만 참조 설정의 hash와 일치하지 않으므로, 이 shadow 추론 결과는 선택 checkpoint를 확정하거나 실기 동작을 입증하지 않습니다. 네 hand policy 출력 토픽 모두 실제 모터 구동과 분리되어 있습니다.

## Open-loop 사전학습 정책 호출

`system_bringup/open_loop_policy_bridge`는 실시간 카메라·모터 피드백을 구독하지 않고, ROS 2 action 요청의 `observation` 리스트를 그대로 선택된 고정 모델에 전달합니다. 호출자가 `scan_id`, `execution_id`, `step_index`, `hand_id`, `policy_id`, observation을 관리하며, action 결과에는 같은 식별자와 모델 action 리스트가 돌아옵니다. 상태 리스트는 호출자/상위 파이프라인이 보관하고 다음 단계 요청을 만들 때 갱신합니다. 이 API는 상태를 자동 추정하거나 큐브·손 움직임의 성공을 판정하지 않습니다.

등록 모델은 다음과 같으며, 임의 경로나 미등록 ID는 요청에서 선택할 수 없습니다.

| `policy_id` | hand | 입력 리스트 | 반환 리스트 | 현재 의미 |
|---|---|---:|---:|---|
| `leap_faceup_v1` | `leap_hand` | 108개 실수 | 16개 정규화 action | 목표면 정렬 정책의 단일 checkpoint |
| `custom_finger_ppo_v1` | `custom_hand` | 8개 실수 | 4개 정규화 action | Custom Hand의 단일 checkpoint |

서버는 입력 차원, 유한값, `hand_id`와 `policy_id`의 대응을 검사하고 deterministic inference를 한 번 수행합니다. 결과는 `shadow_only: true`인 정책 출력일 뿐 motor target 변환이나 하드웨어 쓰기는 하지 않습니다. 외부 open-loop 상태 관리자는 각 solver move와 primitive에 맞춰 어느 `policy_id`를 호출할지 결정합니다. LEAP 입력 항목과 Custom 입력 항목의 순서는 기존 정책 계약과 같아야 합니다. 의미가 확인되지 않은 리스트 값으로 얻은 출력은 통신 smoke test에만 해당합니다.

빌드 후 별도 셸에서 bridge를 시작합니다.

```powershell
ros2 run system_bringup open_loop_policy_bridge
```

Custom 모델의 8개 observation 리스트를 보내는 smoke test 예입니다. 0값은 action 통신과 모델 호출 확인용일 뿐, 유효한 실물 observation을 뜻하지 않습니다.

```powershell
$obs = @(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
$obsYaml = $obs -join ', '
$goal = "{scan_id: open-loop-demo, execution_id: run-001, step_index: 1, hand_id: custom_hand, policy_id: custom_finger_ppo_v1, observation: [$obsYaml]}"
ros2 action send_goal /policy/infer system_interfaces/action/InferHandPolicy $goal --feedback
```

성공 응답에서 `success: true`, `shadow_only: true`, `action_values` 4개를 확인합니다. LEAP 요청은 동일한 action에 `hand_id: leap_hand`, `policy_id: leap_faceup_v1`, observation 108개를 보내며 출력은 16개입니다. 아래의 통합 dry-run에서는 executor가 이 추론을 primitive별로 호출하고 계획 상태를 갱신합니다.

## 빠른 Shadow 경로 확인

카메라와 모터 없이 bridge와 양쪽 정책 추론 경로를 확인할 수 있습니다. 아래 합성 publisher는 프로젝트 입력 토픽에 데이터를 발행하므로 실제 카메라·하드웨어 driver·`cube_sequence_executor`와 동시에 실행하지 않습니다.

```powershell
cd \middleware-bridge
pixi run build-windows
```

PowerShell 창 네 개를 열고, 각 창에서 프로젝트 환경과 overlay를 불러옵니다.

```powershell
cd \middleware-bridge
pixi shell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
. .\.colcon-win\install\local_setup.ps1
```

각 창에서 하나씩 실행합니다.

1. 입력 bridge

   ```powershell
   ros2 run system_bringup policy_input_bridge --ros-args -p world_frame:=policy_demo_world
   ```

2. LEAP shadow 추론

   ```powershell
   ros2 run leap_hand_control leap_policy_shadow --ros-args -p world_frame:=policy_demo_world
   ```

3. Custom shadow 추론

   ```powershell
   ros2 run custom_hand_control custom_hand_policy_shadow
   ```

4. 합성 입력 publisher

   ```powershell
   ros2 run system_bringup policy_input_demo_publisher --ros-args -p world_frame:=policy_demo_world
   ```

4번 창에 `leap_hand: shadow_only=True, elements=16`과 `custom_hand: shadow_only=True, elements=4`가 출력되면 양쪽 정책 출력이 발행된 것입니다. 출력 숫자는 모터로 전달되지 않습니다. 입력 속도와 중복 발행 여부는 별도 창에서 `ros2 topic hz /leap_hand/policy_input` 및 `ros2 topic hz /custom_hand/policy_input`으로 확인할 수 있습니다. 데모는 각 입력을 약 20 Hz로 발행합니다. 토픽 내용은 `ros2 topic echo /leap_hand/policy_action`, `ros2 topic echo /custom_hand/policy_action`으로 확인합니다. 합성 publisher는 `Ctrl+C`로 종료합니다.

빌드 중 `system_interfaces`에서 CMake deprecation 및 policy 경고가 출력될 수 있습니다. 이는 경고이며 빌드가 `Finished`로 끝나면 그 경고만으로 패키지 실패를 의미하지 않습니다. 빌드 결과의 `Summary`에서 실패 패키지가 없는지도 확인합니다.

## Windows 빌드

프로젝트 루트에서 전체 워크스페이스를 빌드합니다.

```powershell
cd \middleware-bridge
pixi run build-windows
```

빌드가 끝난 뒤 각 PowerShell 창에서 ROS overlay를 불러옵니다.

```powershell
cd \middleware-bridge
pixi shell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
. .\.colcon-win\install\local_setup.ps1
```

## 기존 Primitive Dry-run 확인

PowerShell 창 네 개를 사용합니다. 먼저 각 창에서 위의 Pixi 환경과 ROS overlay를 불러온 뒤 다음 노드를 각각 실행합니다.

1. 창 1 — 풀이 실행 노드

   ```powershell
   ros2 run cube_execution cube_sequence_executor
   ```

2. 창 2 — LEAP Hand primitive 서버

   ```powershell
   ros2 run leap_hand_control leap_hand_primitive_server
   ```

3. 창 3 — Custom Hand primitive 서버

   ```powershell
   ros2 run custom_hand_control custom_hand_primitive_server
   ```

4. 창 4 — 실행 요청. 아래 예시는 색상 수가 맞는 solved facelets와 세 개의 형식상 유효한 Kociemba 토큰을 보내 dry-run 단계 전달을 확인합니다. 예시 입력은 실제 큐브 풀이를 검증하는 용도가 아닙니다.

   ```powershell
   ros2 topic pub --once /cube/solution system_interfaces/msg/CubeSolution "{scan_id: dryrun-001, facelets: UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB, moves: [R, `"U'`", F2]}"
   ```

   메시지를 발행한 다음 같은 창에서 실행 액션을 보냅니다.

   ```powershell
   ros2 action send_goal /cube/execute_solution system_interfaces/action/ExecuteCubeSolution "{scan_id: dryrun-001, workflow_id: two_hand_layer_turn_v1, dry_run: true}" --feedback
   ```

세 이동이 모두 전달되면 액션 결과에서 `success: true`, `completed_moves: 3`, `completed_steps: 12`를 확인합니다. feedback에는 각 이동의 네 primitive와 handoff 상태가 표시됩니다. 결과는 ROS 액션 요청과 dry-run 응답을 확인한 것이며 실제 큐브가 움직였다는 뜻은 아닙니다.

상태 토픽은 별도 창에서 확인할 수 있습니다.

```powershell
ros2 topic echo /cube/execution_status
```

액션 인터페이스와 실행 항목은 다음 명령으로 확인합니다.

```powershell
ros2 interface show system_interfaces/action/ExecuteCubeSolution
ros2 interface show system_interfaces/action/ExecuteHandPrimitive
ros2 pkg executables cube_execution
ros2 pkg executables leap_hand_control
ros2 pkg executables custom_hand_control
```

## 고정 모델 포함 통합 Dry-run 확인

기존 세 서버에 더해 별도 PowerShell 창에서 다음 노드를 실행합니다.

```powershell
ros2 run system_bringup open_loop_policy_bridge
```

`/cube/solution`에 한 이동을 발행한 뒤 `/cube/execute_solution`에 다음 goal을 보냅니다. 초기 값은 통신 smoke test용 임의 계획값입니다. 실제 입력이나 실물 calibration을 의미하지 않습니다.

```powershell
ros2 topic pub --once /cube/solution system_interfaces/msg/CubeSolution "{scan_id: policy-dryrun-001, facelets: UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB, moves: [R]}"
$goal = @'
{scan_id: policy-dryrun-001, workflow_id: two_hand_layer_turn_v1, dry_run: true, run_pretrained_policies: true, initial_open_loop_state: {cube_orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, face_normals_cube_frame_urfdlb: [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0], palm_minus_cube_position_m: [0.0, 0.0, 0.0], leap_command_positions_rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], custom_policy_positions: [0.0, 0.0, 0.0, 0.0], custom_policy_velocities: [0.0, 0.0, 0.0, 0.0]}}
'@
ros2 action send_goal /cube/execute_solution system_interfaces/action/ExecuteCubeSolution $goal --feedback
```

정상 결과는 `success: true`, `completed_steps: 4`, `policy_ids: [leap_faceup_v1, custom_finger_ppo_v1, custom_finger_ppo_v1, custom_finger_ppo_v1]`이며 `policy_action_sizes`는 `[16, 4, 4, 4]`입니다. 위 면 법선은 CLI 통신 smoke test용 예시 축 대응일 뿐이며 실제 조립 장치의 보정값으로 쓰면 안 됩니다. `policy_action_values`는 크기 배열 순서대로 이어 붙인 shadow 값입니다. 이 확인은 정책 호출·workflow 전달·이상적 계획 상태 갱신만 검증하며 센서, 물리 동작, 모터 제어를 확인하지 않습니다.

## 실제 구동 연결 전 미확정 항목

LEAP Hand는 큐브 아래에서 지지·방향 조정을 하고, Custom Hand는 위에서 학습된 손가락 파지와 손목 회전을 담당하는 구조입니다. 사용자가 제공한 모터 정보는 Custom 손가락 Dynamixel XL330-M288-T, Custom 손목 Feetech STS3215, LEAP Hand Dynamixel XC330-M288-T입니다.

아래는 참고 디렉토리와 현재 코드에서 파일·설정으로 확인한 내용입니다. 참고 설정에서 가져온 후보 값은 현재 조립 장치의 calibration으로 간주하지 않으며 검증 전까지 `verified: false`로 둡니다.

| 대상 | 확인된 자료 | 현재 판정 |
|---|---|---|
| Windows 카메라 | 이전 실험에서 `camera_index=0`, `dshow`, 1280×720, 30 FPS로 영상 취득과 여섯 면 풀이를 확인했습니다. 카메라 제조사·모델·장치 식별자는 기록되지 않았습니다. | 동작 설정은 확인됐지만 장치 식별과 6-DoF pose 출력은 미확인입니다. 현재 detector는 `/camera/image_raw`에서 OBB 검출 후 `/cube/face_detections`를 발행하며 pose topic은 발행하지 않습니다. |
| 큐브 YOLO | `src/perception/cube_detection/model/cube-detector.pt`가 존재합니다. SHA-256은 `C1403D43D654A37D21351A5F5124AABC07C1BDB921E54E5FBBC899304BB409FB`입니다. | OBB 검출용 모델입니다. 모델 존재와 검출 기능은 큐브 pose 보정 완료를 뜻하지 않습니다. |
| Custom Hand 정책 | `sim2real/finger_robot_ppo_best.zip`을 `src/control/custom_hand_control/models/finger_robot_ppo_best.zip`로 복사했습니다. 실제 파일 SHA-256은 `5B9A3F36B348FA7FCC16370432E9F5CE476091CBEE38431686166FFABABD426B`입니다. | ZIP 메타데이터는 observation(8), action(4), actor 층 크기 8→64→64→4를 확인합니다. 참고 설정에 기록된 해시 `be90522e32676f1e815267e8ce962eeb9e404ebef55d97d3fca37505a84478f9`와 달라 같은 checkpoint로 확정할 수 없습니다. 복사본은 후보이며 calibration은 미검증입니다. |
| Custom Hand 모터 설정 | `left-rl-with-mujoco/sim2real/config.provisional.json`에는 `COM8`, 1,000,000 baud, XL330 ID 1–4, 손가락 순서 right/front/left/back → ID 4/1/2/3이 적혀 있습니다. 사용자는 해당 코드로 실작동을 확인했다고 보고했습니다. | 예시 JSON에 포트·baud·ID를 참고 후보로 반영했습니다. 단, 참고 설정의 mapping, 모터별 calibration·limits, `hardware_validated` 표시는 여전히 미검증입니다. 손목 Feetech 설정은 이 디렉토리에 없습니다. |
| LEAP 실작동 코드 | `right_rl_with_mujoco/deploy/policy_runner.py`는 기본 실행에서 `LeapHW`를 만들고 제어 루프에서 `write_sim(q_cmd)`를 호출합니다. `leap_hw.py`는 Dynamixel 연결, 토크 활성화·해제, 상태 읽기와 위치 쓰기를 구현합니다. | **실제 모터를 쓰는 배포 코드가 있습니다.** 따라서 임의 motor ID를 만들지 않고 소스의 ID 1–16을 예시 JSON에 반영했습니다. 현재 middleware LEAP adapter 자체는 dry-run입니다. |
| LEAP 정책·모터·pose 설정 | 참고 정책은 `right_rl_with_mujoco/policy/policy_np.npz` (SHA-256 `DC220D146BC5532C4AE09907C6D6BBC887335389D215AFAAFB69EEAB679560E7`)이며 108차원 관측 조립기를 사용합니다. `leap_hw.py`의 기본 포트는 Linux `/dev/ttyUSB0`, baud는 3,000,000입니다. 회전 관절 ID 1, 5, 9의 부호는 미검증입니다. 정책 실행기는 `--cams`에 카메라 보정 파일을 요구합니다. | motor ID와 Linux 기준값을 참조로 기록했습니다. `/dev/ttyUSB0`는 Windows COM 포트로 사용할 수 없으므로 실제 Windows port·baud는 비워 두었습니다. 해당 camera calibration 파일도 middleware에 연결되어 있지 않습니다. |

따라서 현재 확정 가능한 실물 모터 포트/ID와 큐브 pose 입력은 없습니다. 추가 수집 항목은 카메라 제조사·모델 또는 pose provider, pose topic/type/frame, 실제 양손의 모터 응답 ID·model number·COM 포트·baud rate, 선택한 policy artifact 경로와 SHA-256, 조립 frame 및 정지 방식입니다. 현재 `target_face`와 회전량은 논리적 계약 값이며 실제 모터 명령으로 변환되지 않습니다.

### 실제 입력 연결 현황

현재 소스에서 확인한 ROS publisher와 정책 입력 bridge의 연결 상태입니다. 여기서 “publisher 없음”은 소스 코드 검색 결과이며, 별도 외부 노드가 실행 중일 가능성까지 배제하는 런타임 검사는 아닙니다.

| 입력 | bridge가 기대하는 형식 | 현재 프로젝트 내 생산자 | 연결 판정 |
|---|---|---|---|
| `/cube/pose` | `geometry_msgs/msg/PoseStamped`, 공통 `world_frame` | 없음. YOLO는 `/cube/face_detections`에 `FaceDetectionArray`(2D OBB, class, confidence)를 발행 | 미연결. 2D 검출만으로 3D pose를 만들 수 없으며 카메라 내·외부 보정과 큐브 방향 추정이 필요 |
| `/leap_hand/palm_pose` | `PoseStamped`, 공통 `world_frame` | middleware 내 없음 | 미연결. 실제 pose provider 또는 보정된 순기구학 publisher 필요 |
| `/leap_hand/joint_states` | `sensor_msgs/msg/JointState`, 정책 관절 16개와 일치하는 이름·position(rad) | middleware 내 없음 | 미연결. LEAP driver의 실제 상태 publisher와 관절 이름 대응 필요 |
| `/leap_hand/commanded_joint_states` | `JointState`, 같은 16개 관절 순서의 명령 위치(rad) | middleware 내 없음 | 미연결. 명령 publisher를 driver 출력과 연결해야 함 |
| `/custom_hand/policy_joint_states` | `JointState`, 보정된 손가락 policy 좌표 4개와 velocity | middleware 내 없음 | 미연결. 원시 Dynamixel tick을 직접 넣지 말고 학습 observation 좌표로 변환해야 함 |
| `/cube/policy_goal` | `system_interfaces/msg/CubePolicyGoal`, 활성 scan/execution 및 target face에 대응하는 quaternion | middleware 내 없음 | 미연결. facelet 표기와 측정 큐브 frame 사이의 보정 변환기가 필요 |
| `/cube/execution_status` | `system_interfaces/msg/CubeExecutionStatus` | `cube_sequence_executor` | 생산 코드 있음. 실제 hand adapter가 실행 단계 status를 내는지 별도로 확인 필요 |

따라서 실제 입력 연결 단계는 현재 코드만으로 완료할 수 없습니다. 먼저 아래 명령으로 각 토픽의 런타임 publisher와 형식을 확인합니다. `topic list`만으로 토픽이 준비됐다고 판단하지 말고, `topic info -v`의 publisher 개수와 `echo --once` 메시지 내용을 확인합니다.

```powershell
ros2 topic list -t
ros2 topic info -v /cube/pose
ros2 topic info -v /leap_hand/palm_pose
ros2 topic info -v /leap_hand/joint_states
ros2 topic info -v /leap_hand/commanded_joint_states
ros2 topic info -v /custom_hand/policy_joint_states
ros2 topic info -v /cube/policy_goal
ros2 topic info -v /cube/execution_status
```

실제 provider가 연결된 토픽은 다음처럼 한 번 수신해 timestamp, frame, 관절 이름·단위가 위 계약과 맞는지 확인합니다.

```powershell
ros2 topic echo --once /cube/pose
ros2 topic echo --once /leap_hand/palm_pose
ros2 topic echo --once /leap_hand/joint_states
ros2 topic echo --once /leap_hand/commanded_joint_states
ros2 topic echo --once /custom_hand/policy_joint_states
ros2 topic echo --once /cube/policy_goal
```

현재 `policy_input_demo_publisher`는 이 입력 토픽들에 합성값을 발행하므로 실제 provider를 검사할 때는 반드시 종료해야 합니다. 실측 provider가 없는 토픽은 실행 명령이나 합성값으로 대체하지 않고 미연결로 기록합니다. 모든 입력 provider와 `/cube/policy_goal` 변환이 실측으로 확인된 뒤 bridge/shadow를 실행해 정책 입력·출력과 timestamp freshness를 확인합니다. 이 검증도 모터 출력은 연결하지 않습니다.

## Calibration 설정 초안 및 구동 차단

`cube_execution/config/hardware_calibration.example.json`은 실제 calibration 데이터를 채우기 위한 JSON 형식 초안입니다. Custom 손가락과 LEAP 모터 ID, Custom COM8/baud 값은 참고 설정에서 가져온 후보로 미리 적고 `verified`는 `false`로 유지했습니다. LEAP의 Linux port는 `reference_port`에만 두고 실제 `port`는 비워 두었습니다. Custom 손목·frame·pose·모델 경로와 검증 결과 등 미확정 값은 빈 문자열, `null`, 빈 목록 또는 `false`입니다. 두 정책 후보 파일의 SHA-256을 기록했지만 `model_path`는 각 로컬 설정에 맞춰 채워야 합니다. Custom 파일은 참고 설정의 기록 해시와 다르므로 선택 checkpoint가 맞는지도 확인이 필요합니다. 이 파일은 현재 장치와 모델의 완료된 calibration이 아닙니다.

`cube_execution.calibration` 검증기는 다음 항목을 확인합니다.

- schema 버전과 문서 전체, pose, frame/transform, 손별 motor 그룹, 정책, 안전 항목의 명시적 `verified: true` 표시
- pose topic·type·frame·단위·quaternion 형식과 양수 pose age 한도
- 각 프레임 이름, motor model, port·양수 baud rate·중복 없는 motor ID 목록
- 각 motor ID와 1:1로 대응하는 joint mapping: 이름, ID, 방향(`-1` 또는 `1`), zero offset, 단위, 유효 범위
- LEAP/Custom 정책 파일 경로와 기록한 SHA-256의 실제 파일 일치, observation schema 확인 표시
- 비상정지, 정지 방식, joint/workspace 한계, 완료 feedback, handoff 안전 검증 표시

검증기는 JSON 기록의 형식과 필수 표시를 검사합니다. 장치에 실제 연결하거나 pose 측정, 모델 입출력, 안전 동작을 실험하는 기능은 아닙니다. `verified` 표시와 근거를 채우는 책임은 실제 측정·시험을 수행한 사용자에게 있습니다.

검증하려면 예시를 사용자 로컬 설정 위치에 복사한 뒤 checker를 실행합니다. 미완성 예시에서 오류 목록이 출력되는 것이 정상입니다.

```powershell
$calibrationPath = Join-Path $env:LOCALAPPDATA 'middleware-bridge\hardware_calibration.json'
New-Item -ItemType Directory -Force (Split-Path $calibrationPath) | Out-Null
Copy-Item .\src\control\cube_execution\config\hardware_calibration.example.json $calibrationPath
ros2 run cube_execution cube_calibration_check $calibrationPath
```

executor는 시작 시 `calibration_file` ROS 매개변수를 읽습니다. 기본값은 비어 있으므로 실제 실행 목표는 `CALIBRATION_INCOMPLETE`로 거부됩니다. 파일을 지정해 calibration 검증이 통과하더라도 motor driver와 실제 adapter 동작이 아직 구현되지 않았으므로 `dry_run=false`는 `EXECUTION_DISABLED`로 계속 거부됩니다. calibration 파일만으로 실제 구동이 활성화되지는 않습니다.

```powershell
ros2 run cube_execution cube_sequence_executor --ros-args -p "calibration_file:=$calibrationPath"
```

실제 구동은 카메라·pose provider와 calibration을 측정·검증하고, 장치별 driver, pose 기반 목표 변환, 완료 feedback, 검증된 정지·복구 동작을 adapter에 구현한 뒤에만 별도 작업으로 활성화할 수 있습니다.

## Pose·좌표계·안전 계약

이 절은 실제 센서나 하드웨어의 값이 아니라, 각 입력과 동작을 실제 구동에 연결하기 전에 충족해야 할 계약입니다. 미정인 프레임 이름, 오차 한도, 시간 한도, 모터 ID 및 목표값은 임의로 채우지 않습니다.

### 큐브 pose 입력 계약

현재 detector는 큐브 면의 OBB·클래스·신뢰도를 발행하지만, 이 정보만으로 로봇 기준 큐브의 6자유도 pose를 제공한다고 간주하지 않습니다. 현재 `/cube/pose` publisher와 pose 좌표 보정은 구현·검증되지 않았습니다.

실제 구동에 연결할 pose provider는 다음을 제공해야 합니다.

- timestamp가 포함된 큐브 중심 위치와 방향. ROS 표준형을 사용할 경우 `geometry_msgs/msg/PoseStamped`가 후보이며, 실제 topic과 type은 provider 선정 시 확정합니다.
- pose의 `header.frame_id`, 단위(m), 오른손 좌표계 여부, quaternion 성분 순서와 정규화 규칙.
- 로봇 기준 좌표계까지의 정적·동적 transform과 transform의 timestamp. 각 손의 base frame 및 큐브 grasp 기준 frame을 함께 정의합니다.
- pose 신뢰도 또는 유효/무효 상태, 갱신 주기와 허용 가능한 최대 데이터 age. 이 기준은 센서 성능과 실제 동작 속도를 측정한 뒤 정합니다.
- 큐브를 다시 잡거나 손 간 인계 후 pose를 재확인하는 방법. 검증되지 않은 pose 또는 transform 누락·시간 초과는 실행 거부 조건입니다.

Kociemba 논리 면은 `U, R, F, D, L, B`로 식별합니다. calibration은 각 논리 면의 outward normal을 pose provider의 큐브 frame에 대응시키고, 큐브 frame을 LEAP·Custom Hand 기준 frame에 연결해야 합니다. LEAP의 face-up 성공 판정에는 어느 face normal이 위를 향하는지 정의하며, 면 법선 주위의 회전 자유도를 허용할지 별도 명시합니다. OBB 검출만으로 면의 의미 라벨이나 방향을 추정하는 규칙은 추가 검증 전까지 두지 않습니다.

### 회전과 관절 매핑 계약

`clockwise_quarter_turns`의 논리 회전량은 대상 면 바깥에서 볼 때 `+1 = +90°`, `-1 = -90°`, `2 = 180°`입니다. 이는 solver 기준의 의미만 고정합니다. 실제 Feetech 목표각·부호·영점·회전축은 Custom Hand wrist frame과 큐브 frame 사이의 보정으로 결정해야 하며 현재 값은 없습니다.

하드웨어 calibration record는 적어도 다음 항목을 갖추고 각 항목의 검증 근거를 남겨야 합니다.

| 항목 | 기록할 값 | 구동 허가 조건 |
|---|---|---|
| 장치 연결 | Dynamixel/Feetech 모델, 통신 포트·속도, motor ID | 실제 조립된 장치에서 ID 중복·통신 오류가 없음을 읽기 전용 확인 |
| 관절 대응 | ROS joint 이름 ↔ motor ID, 관절 순서, 부호, 영점, 단위, 허용 범위 | 관절별 방향·영점·범위를 저속 검증하고 기록 |
| 손 frame | LEAP base, Custom base, 손목 회전축, 큐브 grasp frame | 측정값·방향 정의 및 재현 가능한 calibration 절차 보유 |
| 학습 정책 | 모델 경로·식별값, observation 이름·순서·단위, action 범위·정규화 | 현재 학습 모델의 입출력이 실기 adapter 변환과 일치함을 검증 |
| 완료 판정 | 목표 대비 joint 상태, 큐브 pose 변화, grasp/support 상태의 기준 | 센서 feedback으로 성공·정지 조건을 구별할 수 있음 |

값마다 `verified`와 검증 기록을 남깁니다. 어떤 필수 항목이 미검증·누락·만료 상태이면 adapter는 실제 모드 목표를 거부해야 합니다. 참고용 학습 디렉토리에 있는 임시 motor mapping과 sign 값은 현재 조립에 대해 검증되지 않았으므로 calibration 값으로 복사하지 않습니다.

### Primitive 순서와 handoff 조건

현재 네 primitive 순서는 dry-run orchestration 계약입니다. 실제 동작에서 각 primitive의 성공 응답은 명령 전송만으로 반환하면 안 되며, 해당 동작의 완료 sensor feedback을 확인해야 합니다.

| 단계 | 성공으로 볼 조건 | 다음 단계 허용 조건 |
|---|---|---|
| LEAP `orient_face_up_and_support` | 목표 면 방향 오차가 검증된 허용범위에 들어오고 큐브 지지가 확인됨 | pose·지지 상태가 유효하고 Custom 파지 영역이 안전함 |
| Custom `grasp_upper_layer` | 손가락 상태와 grasp 확인 기준이 충족됨 | LEAP support 유지가 확인되고 wrist 회전 경로가 비어 있음 |
| Custom `rotate_upper_layer` | 목표 레이어 각도와 큐브 pose 변화가 검증된 범위에 도달함 | 회전 완료가 확인되며 LEAP가 큐브를 계속 지지함 |
| Custom `release_upper_layer` | 파지가 해제됐고 큐브가 LEAP에 안정적으로 남아 있음 | 다음 solver 이동 전에 pose 재측정·논리 상태 갱신 완료 |

목표 오차, 힘·전류, 속도, 정착 시간, timeout은 장치·센서 선택 뒤 실험으로 정합니다. 해당 값이 없으면 dry-run 이외의 동작을 허가하지 않습니다.

### 정지·실패·취소 조건

- pose 또는 transform 누락·오래된 값·유효성 실패, motor 통신/상태 이상, grasp/support 상실, 한계 초과, 완료 feedback 미수신 시 즉시 현재 실행을 중단하고 남은 primitive를 보내지 않습니다.
- adapter는 검증된 장치별 stop 동작을 수행하고, 실행 노드는 실패·취소 상태와 마지막으로 완료가 확인된 move/step을 발행합니다. 자동으로 손을 풀거나 큐브를 재배치하는 복구 동작은 현재 정의하지 않습니다.
- 현재 hardware API의 stop/hold 동작은 정해지지 않았습니다. 정지 명령이 실제로 어떤 자세를 유지하는지 검증하기 전까지 실패 시의 기계적 안전 상태를 보장한다고 간주하지 않습니다.
- 실패 뒤에는 operator 확인, 큐브 pose 재측정, 필요 시 새 `scan_id`로 재인식·재풀이를 거친 다음 별도 실행 목표를 받아야 합니다. 기존 action을 자동 재시도하거나 중단 지점부터 이어서 실행하지 않습니다.
- 비상정지 수단과 접근 위치, 토크·속도 제한, 작업 공간과 사람 접근 제한, 수동 회수 절차가 확인되지 않으면 실제 모터 구동을 시작하지 않습니다.

위 계약을 실제 값으로 채우기 위한 자료는 카메라/pose provider의 선택, 양 손과 손목의 실물 조립 frame·motor ID·통신값, 정책별 실기 observation/action 보정, 하드웨어 정지 방식입니다. shadow input의 freshness·frame·배열 형식 검사는 구현했지만 실제 calibration 기록, upstream pose/joint publisher, pose 기반 목표 변환, motor driver 및 adapter 완료 판정은 아직 없습니다.

## 카메라에서 얻은 풀이로 방향 순열 trace 확인

이전 카메라 실험에서 발행된 facelet과 Kociemba 풀이를 재사용해 20개 이동 전체를 정책 포함 dry-run으로 실행했습니다. 새로 카메라를 촬영한 시험은 아닙니다. 검증한 입력은 다음과 같습니다.

- `facelets`: `BFLDUBFFUFLDFRRLBRRULUFRULBLUDDDRBBURBDLLRRFBFLUDBDFUD`
- `moves`: `D2 L' F' D' L B2 D' R F2 B2 D R B2 L2 U L2 U B2 U2 L2`
- 실행 결과: `success: true`, `completed_moves: 20`, `completed_steps: 80`
- 정책 추론·방향 trace: 각 primitive 80개 항목. LEAP가 정렬한 뒤 각 이동의 대상 면 방향 슬롯은 0이었고, 이어지는 Custom 파지·회전·해제 단계에서는 전체 큐브 방향 순열이 유지됐습니다.

각 이동 직전 순열에서 이동 대상 면이 있던 슬롯과 LEAP 정렬 뒤의 순열은 다음과 같습니다. 순열 문자열은 현재 URFDLB 방향 슬롯에 놓인 초기 면 라벨 순서입니다.

| 이동 | 이동 전 순열 | 대상 슬롯 | LEAP 정렬 뒤 순열 |
|---|---|---:|---|
| `D2` | `URFDLB` | 3 | `DLFURB` |
| `L'` | `DLFURB` | 1 | `LUFRDB` |
| `F'` | `LUFRDB` | 2 | `FURBDL` |
| `D'` | `FURBDL` | 4 | `DFRUBL` |
| `L` | `DFRUBL` | 5 | `LFDRBU` |
| `B2` | `LFDRBU` | 4 | `BLDFRU` |
| `D'` | `BLDFRU` | 2 | `DLFURB` |
| `R` | `DLFURB` | 4 | `RDFLUB` |
| `F2` | `RDFLUB` | 2 | `FDLBUR` |
| `B2` | `FDLBUR` | 3 | `BULFDR` |
| `D` | `BULFDR` | 4 | `DBLUFR` |
| `R` | `DBLUFR` | 5 | `RBDLFU` |
| `B2` | `RBDLFU` | 1 | `BLDFRU` |
| `L2` | `BLDFRU` | 1 | `LFDRBU` |
| `U` | `LFDRBU` | 5 | `UFLDBR` |
| `L2` | `UFLDBR` | 2 | `LFDRBU` |
| `U` | `LFDRBU` | 5 | `UFLDBR` |
| `B2` | `UFLDBR` | 4 | `BULFDR` |
| `U2` | `BULFDR` | 1 | `UFLDBR` |
| `L2` | `UFLDBR` | 2 | `LFDRBU` |

이 시험의 quaternion은 단위 회전, URFDLB 법선은 축 정렬된 예시값, 관절·위치 계획값은 0으로 넣었습니다. 따라서 결과는 풀이 전체를 action으로 전달하고 정책 추론·계획 갱신·방향 순열 trace가 완주하는지 확인한 것입니다. 카메라 pose와 실제 장착 방향, 물리적 LEAP 회전 또는 Custom 레이어 회전을 검증하지 않았으며 모터 명령은 발행되지 않았습니다. solver facelet 문자열도 실행 중 바뀌지 않습니다.

## 저장된 카메라 풀이 재생 스크립트 입력 검사

`scripts/ros2_dryrun_check.py`는 기본으로 가장 최근 `results/cube_solutions/*.json`을 선택합니다. 실행 전에 JSON object 형식, `format`, 비어 있지 않은 `scan_id`, facelet 54자와 URFDLB 각 9개, Kociemba move 토큰, `move_count`와 실제 `moves` 길이의 일치를 검사합니다. 검증 실패 시 오류를 출력하고 ROS 초기화·발행 전에 종료합니다.

ROS action을 실행하지 않고 파일만 확인하려면 overlay를 불러온 환경에서 실행합니다.

```powershell
python scripts\ros2_dryrun_check.py --validate-only
python scripts\ros2_dryrun_check.py --validate-only --solution .\results\cube_solutions\cube_solution_20261103T115403_251838Z_404a3217d7b04219a8dc186fe873bf1b.json
```

`--validate-only`는 facelet 문자·개수와 move 표기·개수 일치 등 파일 구조만 검사합니다. Action 모드(`success`, `missing`, `cancel`)에서는 구조 검사를 통과한 뒤 `/cube/solve` 서비스로 facelet의 물리적 유효성과 풀이 가능 여부를 확인합니다. solver 응답이 실패하거나 서비스가 준비되지 않으면 `/cube/solution`에 발행하지 않습니다. 유효한 입력은 선택된 파일, 원본 `scan_id`, 이동 수를 먼저 출력합니다. JSON의 `moves`가 해당 facelet을 실제로 푸는지는 아직 대조하지 않습니다.

Action 모드를 쓰려면 `cube_solver`, `cube_sequence_executor`, `leap_hand_primitive_server`, `custom_hand_primitive_server`가 실행 중이어야 합니다. `--validate-only`에는 ROS 노드가 필요하지 않습니다.

solver 연동 뒤 저장 JSON은 `solver_validation: PASS`를 거쳐 20 moves·80 steps까지 완료했습니다. 면별 개수는 각각 9개지만 UF edge 스티커 한 쌍만 뒤집은 임시 상태는 `/cube/solve`가 `Cube state is invalid or unsolvable`로 거부했습니다. action 모드에서는 `/cube/solution` 발행 전 검증에 실패해 종료했습니다. `missing`은 `SOLUTION_NOT_FOUND`, `cancel`은 `CANCELED`/`CANCELLED`로 재확인했습니다. 모든 실행은 dry-run입니다.
