# 미들웨어 TODO와 현재 오른손 작업 비교

확인일: 2026-11-09. 저장소 문서와 소스만 읽었으며 장치를 열거나 외부 코드를 실행하지 않았다.

- 대상: `2026-HBNU-Capstone-Robot-Muscle/middleware-bridge`
- 확인한 `main`: `9f6b7d014c9a2b30a7fab13f12a93d4fee37a97b`
- 원격 브랜치 목록에는 현재 `main`만 있었다. 하드웨어 구현 시 사용자가 전달한 팀 지침대로 `feature/hardware`에서 작업한다.
- `apps/monitoring_ui`는 읽기·수정 대상에서 제외했다. 원격 파일 변경과 브랜치 생성은 하지 않았다.
- 읽은 파일과 SHA256은 [manifest.json](manifest.json)에 기록했다.

## 핵심 판단

현재 미들웨어의 작업 분담은 우리가 진행한 목표와 맞는다. 한 풀이 수마다 다음 네 단계를 요청한다.

1. LEAP 오른손 `orient_face_up_and_support`: 목표 면을 위로 보내고 큐브 아래쪽 지지.
2. Custom 왼손 `grasp_upper_layer`: 윗단 파지.
3. Custom 왼손 `rotate_upper_layer`: 지정된 한 층 회전.
4. Custom 왼손 `release_upper_layer`: 윗단 해제.

다만 실제 모터 출력은 아직 없다. 두 손 primitive server는 dry-run이며, 상위 executor도 `dry_run=false`를 차단한다. calibration을 채우더라도 현재 코드는 실제 구동을 시작하지 않는다.

현재 로컬 오른손 작업은 사용자가 확인한 대로 `right_rl_with_mujoco` 원본의 후속 수정·확장이다. 별도 독립 구현으로 취급하거나 원본 제어 설정으로 되돌리지 않는다.

## TODO 중 재사용할 근거와 추가 작업

| 항목 | 현재 로컬에 있는 근거 | 미들웨어에 남은 일 |
|---|---|---|
| 장치 연결·관절 대응 | XC330-M288 16개, 원래 PC COM7/1 Mbps 기록, 명시적 모터 대응 모듈과 실행 로그 | 현재 조립·PC에 맞는 inventory와 출처를 문서·calibration에 전달. 포트 번호를 다른 PC에 고정하지 않음 |
| 관절 변환·제한 | `hand_motor_mapping.py`, ID4 하한 -2도 승인, 기존 목표 간격·추종·전류·전압 검사 | 기존 정책 순서를 유지하는 hardware adapter와 ROS JointState 변환. 실물 재생 제어 주기와 정책 학습 주기를 구분 |
| 모터 통신·중단 | 실제 프로그램에서 읽기 복구, 목표 확인, 토크 OFF 및 설정 복구 기록 | 한 프로세스가 포트를 소유하는 ROS 제어 연결부, 상태 발행, 취소·실패·통신 단절 전달 |
| 방향 변경·파지 | 일부 실제 회전 및 파지 관찰, 원본 시퀀스와 후보 검증 | 지원하는 시작 상태·방향별 경로 선택, 실제 도달·큐브 유지 판정. 모든 면과 배치가 검증됐다고 표시하지 않음 |
| 오른손 지지 유지 | 현재 실험은 유한 시간 관찰 후 열기/제거/OFF | 왼손 파지→회전→해제 동안 오른손의 유지와 건강 감시를 계속하고 다음 방향 변경으로 연결 |
| 큐브 pose·좌표 | 저장 마커 사진과 오프라인 분석 | 실시간 pose provider, 양손 공통 frame, 색/면 ID↔URFDLB 대응, timestamp와 유효성. YOLO 2D 검출을 3D pose로 쓰지 않음 |
| 정책 실행 | 원본 SAC/증류 정책과 middleware 108→16 추론기, 현재 로컬의 사전 경로 재생 | 경로 재생 backend와 주기적 정책 backend의 실행 계약을 구분. 저장 경로를 SAC 실시간 제어로 표시하지 않음 |

기존 로그와 검증은 출처와 해당 조립 조건을 붙여 재사용할 수 있다. 모든 검사를 처음부터 사용자에게 반복시키지 않는다. 바뀐 조립·배선·좌표와 새 ROS 연결 경계는 별도로 확인한다.

## 원본 설정과 현재 설정의 구체적인 차이

같은 이름의 관절 순서 `if_mcp, if_rot, if_pip, if_dip, ...`에서 모터 ID 순서는 다음과 다르다.

```text
원본 deploy/leap_hw.py SIM2ARR+1:
15,16,14,13,11,12,10,9,7,8,6,5,4,3,2,1

현재 hand_motor_mapping.py SIM_TO_MOTOR_IDS:
8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1
```

미들웨어 `hardware-setting.md`에는 아직 위 원본 후보 대응이 들어 있다. 통신 기본값도 Linux `/dev/ttyUSB0`, 3 Mbps이며 현재 실물 기록인 Windows COM7, 1 Mbps와 다르다. 모터 대응 모듈만으로 부호·형상·정책 전체가 검증됐다고 추론하지 않는다. 문서의 `verified`를 일괄 true로 바꾸지 않고 확인된 항목의 근거를 각각 옮긴다.

원본 정책 주기는 50 ms이며 현재 승인된 실물 경로 재생은 최근 75 ms다. 이를 한 상수로 강제 통일하면 학습 관측 이력과 모터 목표 의미가 달라질 수 있으므로 실행 backend별로 명시해야 한다.

## 연결 전에 합의할 중요한 실행 계약

현재 `ExecuteHandPrimitive.action`에는 성공/오류 결과와 실행 상태 feedback이 있다. 그러나 **오른손 action 성공 이후에도 지지 제어가 살아 있어야 하는지**를 구현이 보장하지는 않는다. 현재 네 단계 계획에는 별도 오른손 유지/해제 primitive가 없다.

권장 설계는 오른손이 목표 면과 하부 파지에 도달한 뒤 그 상태를 유지하는 장기 실행 제어부가 있고, ROS 요청은 그 제어부의 상태 전환을 요청하는 방식이다. 성공 응답 이후에도 왼손 작업 중의 오른손 상태 감시가 지속되어야 한다. 오른손 오류가 나면 상위 실행과 왼손도 중단시키는 전파가 필요하다. 구체적인 유지 시간, 종료·취소 시 동작과 양손 해제 순서는 팀과 결정한다.

현재 실험 `.cmd`를 ROS에서 그대로 실행하면 SPACE/PLACE/REMOVE 대기와 자동 OPEN/OFF가 섞인다. 기존의 검사와 통신 로직을 재사용하되 사용자 실험 절차와 통합 제어 수명을 분리해야 한다. 단순히 종료 OFF를 삭제하거나 세션 전체의 중단 검사를 해제하는 수정은 하지 않는다.

정책 경로도 현재는 한 primitive당 추론하고 큐브 방향을 이상적 목표값으로 갱신하는 dry-run이다. 이 값을 실제 도달 측정으로 사용하면 안 된다. 물리 실행은 관측 갱신과 완료 판정이 있는 주기적 제어 또는 명시적으로 선택된 경로 재생이 필요하다.

## 작업 위치와 순서 제안

1. `right_rl_with_mujoco`에 현재 실물 수정의 출처·모터 대응·경로/검증 상태를 정리한다. 이전 원본이 현재의 실물 설정으로 오인되지 않도록 한다.
2. `middleware-bridge`의 `feature/hardware`에서 오른손 adapter, ROS 상태 발행, 완료·오류·취소 전달을 연결한다. UI는 수정하지 않는다.
3. 경로 재생을 쓸지 원본 정책 폐루프를 쓸지 실행 backend를 명시하고, 한 방향→지지 유지→왼손 해제 확인까지의 단일 연결부터 검증한다.
4. 실제 큐브/양손 좌표, 왼손 손가락과 STS3215 손목의 포트·범위·완료 feedback을 담당 팀과 맞춘 뒤 풀이 전체로 넓힌다.

현재 실물의 공통 하부 파지와 윗단 회전 공간은 연구 중이다. 미들웨어 연결 작업이 남아 있는 것과 오른손 동작 자체의 과제 미달을 별도로 기록한다.

## 확인한 소스

- [최신 TODO](https://github.com/2026-HBNU-Capstone-Robot-Muscle/middleware-bridge/blob/9f6b7d014c9a2b30a7fab13f12a93d4fee37a97b/docs/TODO.md)
- [미완료 항목](https://github.com/2026-HBNU-Capstone-Robot-Muscle/middleware-bridge/blob/9f6b7d014c9a2b30a7fab13f12a93d4fee37a97b/docs/incomplete.md)
- [상위 실행기](source/src/control/cube_execution/cube_execution/executor_node.py): 64–109 작업 분담, 243–252 실물 차단, 473 dry-run 전달, 704–708 이상적 방향 갱신.
- [오른손 서버](source/src/control/leap_hand_control/leap_hand_control/primitive_server.py): 131–146 비실행 경로.
- [입력 bridge](source/src/system_bringup/system_bringup/policy_input_bridge.py): 외부 pose/관절 입력이 있어야 shadow 관측 발행.
- [primitive action](source/src/system_interfaces/action/ExecuteHandPrimitive.action), [미검증 calibration 예시](source/src/control/cube_execution/config/hardware_calibration.example.json).
- 로컬 `hand_motor_mapping.py`, `leap_middle_clearance_probe_20261106.py`, `right_hand_rotation_20261109/middle_probe_actual_review/review.json`.
