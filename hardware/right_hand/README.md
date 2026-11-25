# 오른손 실물 후속 개발 작업 — 2026-11-25

최신 가로·세로와 A/B 선택은 아래 실행파일을 사용한다. 두 축 마지막 파지는 검지 완료→중지 이동 순서다.203ms 읽기 지연을 포함한 최대10초 READ 복구를 적용했다. [현재 실행/설치 안내](../../docs/current-rotations-20261125.md)를 먼저 읽는다.

| 현재 동작 | 실행파일 |
|---|---|
| 가로 → 같은 G 유지 | `run_current_horizontal.cmd` |
| 세로 → 같은 G 유지 | `run_current_vertical.cmd` |
| A 가로 / B 세로 선택 반복 | `run_current_keyboard.cmd` |

새cmd는 `.runtime` 외에 `.venv`와 PATH의 Python도 사용할 수 있다. 미들웨어에도 단독과 같은 선택 속도(허용 구간 최대6틱)를 연결했다. 준비/가로 최종 받침2틱은 유지한다. 배속·READ 지연 복구·순차 파지의 실제 ROS/양손 검증은 남아 있다. 원본267개는 보존했다.

## 아래는 보존한11/24 버전 기록

팀 `right_rl_with_mujoco`를 출발점으로 수정·확장한 코드, 정책·학습 결과, 모델, 실험 기록과 인계 자료다. 현재 실물 제어는 저장된 관절 경로 재생이다. 과거 SAC 추가 학습 자료는 보존하지만 현재 실행을 실시간 SAC 제어로 설명하지 않는다.

| 용도 | 실행파일 | 안내 |
|---|---|---|
| 확정 파지 G로 이동·유지 | `leap_measured_grip_hold.cmd` | `right_hand_rotation_20261117/user_measured_grip/` |
| G → 가로 회전 → G | `leap_grip_rotate_grip.cmd` | `right_hand_rotation_20261118/grip_rotation_return/` |
| G → 세로 회전 → G 계속 유지 | `leap_vertical_direct_grip.cmd` | `right_hand_rotation_20261118/vertical_direct_thumb_return/RUN.md` |
| G → 가로 → G 약10초 → 세로 → G 계속 유지 | `leap_two_axis_cycle.cmd` | `right_hand_rotation_20261124/two_axis_cycle/RUN.md` |

가로·세로를 각각 실물에서 수행해 G로 돌아오는 관찰을 확보했다. 새 가로·세로 연속 파일은 두 원본 목표열과 공통 G를 연결했으며 소프트웨어10검사를 통과했다. 두 회전을 한 번에 연결한 실물 성공은 아직 확인하지 않았다. 최신 세로 실행 `vertical_direct_grip_trial_20261124_081445_872143`은 G 유지에 도달한 뒤 상태 읽기 오류로 종료됐고 OFF·설정복구 확인도 실패했다. 앞선 정상 종료 기록과 이 결과를 구분한다.

최신 세로 경로는 큐브를 약지 쪽으로 미는 구간과 불필요한 엄지 후퇴를 빼고, 낮은 검지·중지 받침 → 엄지 먼저 내림 → 기록한1번·2번 → G로 이어진다. 회전 파일은 1배75ms·최대2틱·450mA·기존PID와 기존 건강·추종·통신 감시를 사용한다. 연속 파일은 시작 SPACE 한 번, 이동 중 SPACE/Q/Esc 중단, 마지막 G에서 SPACE 종료다. 무한 회전 반복 파일은 아니다.

Windows 실행파일은 이 폴더를 작업 루트로 사용하며 `.runtime/python/python.exe`를 찾는다. 설치 환경은 업로드하지 않았으므로 다른 PC에서는 `requirements-leap.txt`와 해당 안내를 참고해 Python 환경을 별도로 준비하고 실제 포트·장치 설정을 확인한다. 이 업로드 작업에서는 장치를 열거나 구동하지 않았다.

- 최신 상세 기록: `cloud_handoff_20261106/CLOUD_START_PROMPT.md`, `professor_progress_20261015.md`의 맨 위 항목.
- 이번 원본 경로·해시: 저장소 루트 `RIGHT_HAND_UPDATE_20261124_MANIFEST.json`. 이전 전체 목록은 `RIGHT_HAND_FULL_MANIFEST.json`, 고유 스냅샷 추가분은 `RIGHT_HAND_SUPPLEMENT_MANIFEST.json`.
- 후보의 `operator_trial_manifest.json`은 실행에 필요한 원본 해시를 검사한다. 일부 파일만 옮기거나 원본을 수정하면 검증을 다시 해야 한다.
- 설치 환경·인증정보·대량 원시 영상/`camera` 프레임·중복 체크아웃은 제외한다. 기존 공유 파일과 원래 PC의 원본 자료를 삭제하지 않는다.

남은 일은 새 연속 파일 실물 확인, 반복 시 큐브 위치·방향·유지 관찰, 미들웨어 실제 모터 backend/ROS·양손 하중 통합 검증이다. 미들웨어 코드는 별도 `middleware-bridge`의 `feature/hardware`에서 관리한다.
