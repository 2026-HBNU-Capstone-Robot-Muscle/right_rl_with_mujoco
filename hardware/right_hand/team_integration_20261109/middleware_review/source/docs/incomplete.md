# 실물 구동 전 미완료 항목

## 현재 상태

실기 코드를 확인해 모터 모델 후보, 참조 포트·속도, motor ID 매핑과 정책 주기를 `hardware-setting.md`에 정리했습니다. 하지만 **middleware의 두 hand action server는 아직 dry-run 전용**이고 실제 motor write path가 없습니다. `dry_run=false` 요청은 계속 거부됩니다. 이번 변경은 하드웨어를 움직이지 않았으며, 실물 구동 준비 정보와 차단 사유를 정리한 단계입니다.

## 미완료 항목과 구동 차단 사유

| 우선순위 | 항목 | 확인된 참고 정보 | 완료에 필요한 자료·작업 |
|---|---|---|---|
| P0 | LEAP Windows 연결 정보 | 참조 기본값 `/dev/ttyUSB0`, 3 Mbps, Dynamixel Protocol 2.0 | Windows에서 실제 LEAP 포트(예: COM 번호), USB 변환기, 3 Mbps 통신 가능 여부와 전체 ID 1–16 inventory를 torque-off 상태로 확인 |
| P0 | LEAP 모터/control-table 일치 | 요청된 하드웨어는 XC330-M288-T. 참조 `dynamixel_client.py`의 register address 설명은 XH 모터 기준이며 inventory/model-number gate가 없는 경로 | 실제 각 ID의 model number와 firmware/control-table 호환성을 읽기 전용으로 확인. XC330에서 주소·단위·전류·속도 scale이 맞는지 공식 사양과 실제 telemetry로 대조 |
| P0 | LEAP 관절 매핑·부호 | `SIM2ARR` 기반 ID 순서는 `hardware-setting.md`에 기록. 참조 코드가 `if_rot`, `mf_rot`, `rf_rot` 부호를 미확인으로 표시 | ID별 실물 관절 일치, zero/방향, 실제 충돌 없는 soft/hard limits를 단일 모터 저위험 확인으로 기록. 기존 시뮬 범위를 실물 한계로 간주하지 않음 |
| P0 | Custom 손가락 calibration | 사용자는 참조 실기 코드의 실제 구동을 확인했다고 보고함. 코드에는 `COM8`/1 Mbps와 right/front/left/back → ID 4/1/2/3 후보가 있음. 참조 config의 mapping·limits 검증은 false이고 모델 번호는 null | middleware PC/현재 조립에서 실제 포트·ID·model number를 확인하고, 해당 rig에서 각 모터 영점, open/close tick, 방향, 기계 한계, 전류·온도·추종오차 한계와 검증 근거를 기록 |
| P0 | Custom 손목 구동 | 사용자 제공 모델은 Feetech STS3215. 참조 `sim2real`은 손목을 제외 | 실제 COM 포트, baud, protocol/SDK, motor ID, 축·부호·영점·범위, 속도/가속도·완료 feedback과 safe-stop 동작 확인 후 별도 adapter 구현 |
| P0 | 큐브와 손의 pose/frame | middleware에 실측 `/cube/pose`, `/leap_hand/palm_pose`, transform publisher 없음. 카메라 모델도 미정 | pose provider, 공통 `frame_id`, 카메라/robot extrinsics, 초기 URFDLB 면 대응, 갱신·stale 기준 및 LEAP palm-to-cube pose 구현·측정 |
| P0 | LEAP 실제 정책 입력 | 참조 `policy_runner.py`는 `--cams`를 요구하고 손 관절 측정, cube quaternion·palm 위치로 108차원 관측을 만듦. 무카메라 상태는 명령에 쓰지 말라고 코드에 명시 | 카메라/pose provider를 고르고 middleware 입력 토픽과 관측 이름·순서·단위·timestamp를 연결한 뒤 실측 shadow 비교 |
| P0 | Custom 정책 실측 observation | 참조 실기 코드는 모터 tick/velocity를 4개 policy position/velocity로 변환하지만 mapping과 endpoints는 provisional | 실측 tick↔policy 좌표·부호·영점·속도 변환 확인. middleware 입력 topic을 연결하고 참조 모델과 동일한 8차원 observation 및 4차원 action을 shadow 비교 |
| P0 | 정책 실행주기와 primitive 의미 | 참조 LEAP loop는 50 ms, Custom 모델 기본 실기 주기는 30 ms. middleware workflow는 primitive마다 한 번 추론하는 open-loop 구조 | 각 primitive에서 policy를 한 번 호출할지, 완료까지 주기적으로 호출할지 확정. 목표값 지속시간, action 적분 기준, 완료/timeout 판정과 두 reference runner의 의미 차이를 검증 |
| P0 | Middleware motor driver | 현재 action servers는 validation 후 dry-run 결과를 반환. `pixi.toml` 및 ROS package manifests는 Dynamixel SDK, pyserial, Feetech SDK를 직접 의존성으로 선언하지 않음 | Windows ROS 환경에서 선택한 driver/SDK를 고정·설치하고 포트 권한·baud·실패 동작을 확인. port별 독립 I/O node, 상태 publisher, command adapter를 구현하고 calibration gate와 연결 |
| P0 | 완료 판정과 handoff | 현재 dry-run 성공은 adapter 응답이며 실물 관절 도달·grasp/support·큐브 회전 확인이 아님 | 관절 추종 오차·정착 시간, 큐브 방향/레이어 회전, LEAP 지지와 Custom grasp/release를 확인할 계측과 기준을 정의하고 실패 시 다음 primitive 차단 |
| P0 | 안전 정지·기계 한계 | `hardware_calibration.example.json`의 emergency stop, stop behavior, joint/workspace limits, completion feedback, handoff 검증이 모두 false | 독립 비상정지, torque-off/hold 동작, 작업공간·속도·전류 한계, 통신 단절·stale 상태 동작, 큐브 낙하·끼임 대응과 회수 절차를 실제 장치에서 검증·기록 |
| P1 | Custom 실기 코드와 middleware 모델 실행 호환성 | 현재 middleware Custom checkpoint hash는 참조 `.zip`과 일치. 참조 실기 앱은 Stable-Baselines3와 별도 bus/position-compensation 경로 사용 | Windows 환경의 SB3/모델 로딩과 실제 input/output 좌표를 비교하고, 보정 기능을 사용할지·어떤 제한을 둘지 결정. 참조 설정을 검증 없이 복사하지 않음 |
| P1 | LEAP bus/platform 차이 | 참조 `dynamixel_client.py`는 Linux deploy 경로와 3 Mbps 기본값 사용, `leap_hw.py`는 sysfs FTDI latency timer 확인 시도 | Windows native에서 Dynamixel SDK port handler, latency 설정, serial close 및 torque-off가 기대대로 작동하는지 검사. Linux 경로를 그대로 사용하지 않음 |

## 단계별 해제 조건

1. **Read-only 식별:** 각 독립 bus에서 포트, baud, model number, ID, mode, torque state와 hardware error를 읽고 설정표에 실제값을 기록합니다. 어떤 write도 하지 않습니다.
2. **관절별 무부하 확인:** 실제 비상정지 준비 후 torque-off에서 시작하고, 승인된 제한으로 한 motor/joint씩 방향과 zero를 측정합니다. 결과를 calibration 파일에 기록합니다.
3. **Policy shadow 비교:** 실측 관절·pose 입력을 ROS 토픽에 발행해 middleware와 reference observation/action을 같은 시점에서 비교합니다. motor output은 계속 끕니다.
4. **단일 actuator 구동:** 안전 검증이 끝난 뒤 한 손가락 또는 한 축만 낮은 제한으로 시험하고 command, feedback, stop을 확인합니다. 전 단계 통과 전에는 양손·손목을 함께 활성화하지 않습니다.
5. **Primitive·handoff 검증:** LEAP support, Custom grasp/rotate/release를 큐브 없는 조건부터 단일 큐브 조작까지 순서대로 검증합니다. 완료 feedback과 안전 정지가 확인된 뒤에만 전체 action을 고려합니다.

모든 `verified` 필드와 증거가 실제 측정·시험으로 채워지고 SDK·pose·safety 조건이 충족되기 전까지 middleware는 `dry_run=false`를 받아들이지 않아야 합니다. 저장된 참고 설정만 복사하거나 calibration 표시만 true로 바꾸는 것은 구동 허가 근거가 되지 않습니다.
