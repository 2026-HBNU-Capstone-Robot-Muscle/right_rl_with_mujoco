# 현재 연결 작업 상태

사용자 목표는 초기 배치 → 큐브 전체 회전 → 윗단 공간을 비운 유지 → 다음90도 → 유지다. 아래는 실제 결과 파일에 근거한 상태이며 과거 세션 번호를 실행 중으로 재사용하지 않는다.

첫3015목표의 대체 작업 위치는 옛 canonical 기준에서5.604mm 떨어져 있다. 옛2mm 기준 실패는 보존하고, 새 다음 회전의 위치 목표는 그 실행 전에 실제 첫 유지 위치로 고정했다.

원래170개 실행 결합·미들웨어57개·450mA/PID/관절 범위/2틱75ms와 native 중단 기준은 유지한다. 모델0.15N·m는450mA 환산값이 아니다. ID4의 승인 설정은 lower_degrees_by=2.0이며 옛 모델 하한에서2도 연장한 값이다. 절대각−2도로 바꾸지 않는다.

이번 새SAC 학습은0스텝이며 과거90000 요청 학습 분량과 구분한다. 모델 교사 계산과 저장 경로 재생을 실제 정책 학습·증류로 설명하지 않는다. 사용자 재연결·촬영·반복 각도 읽기는 지금 요구하지 않는다.

## 완료 및 진행 스냅샷

- horizon_fast_entry_cp1700: 결과 파일 완료 / 400목표 / 방향 오차 13.7455도.
- horizon_transport_first_cp1700: 결과 파일 완료 / 800목표 / 방향 오차 0.4998도.
- horizon_fast_next5_from_clear_grip: 결과 파일 완료 / 134목표 / 방향 오차 0.5899도.
- next5_canonical_station_recovery: 결과 파일 완료 / 48목표 / 방향 오차 0.2585도.
- next5_canonical_priority: 결과 파일 완료 / 58목표 / 방향 오차 1.0304도.
- next5_canonical_settled_finish: 결과 파일 완료 / 172목표 / 방향 오차 0.9791도.
- horizon_next90_after_next5: 다른 분기로 이어간 기록 / 630목표 / 방향 오차 1.5285도.
- hybrid_entry_after400: 결과 파일 완료 / 600목표 / 방향 오차 13.389도.
- hybrid_next90_recenter: 결과 파일 완료 / 600목표 / 방향 오차 6.1045도.
- opposing_workspace_feedback_entry: 결과 파일 완료 / 600목표 / 방향 오차 4.3002도.
- opposing_workspace_next90: 결과 파일 완료 / 360목표 / 방향 오차 11.6453도.
- opposing_workspace_push_transport: 결과 파일 완료 / 360목표 / 방향 오차 4.3433도.
- reached_workstation_next90: 결과 파일 완료 / 234목표 / 방향 오차 86.1007도.

초기 배치부터의 첫 회전 재생: initial_to_reached_workstation90/review.json.3015목표·네 구간 전체 상태 오차0·마지막4050 지지/공간 통과. 다음90도와 전체 실기는 미완료다.

현재 우선 남은 문제는 실제 첫 파지에서 회전력을 전달하는 접촉 전환이다. 상세 위치·판정 범위와 남은 조건은 ../opposing_grip_review_20261110/MODEL_TASK_SPEC.md를 읽는다.

publish_native_whole_hand_results.py → publish_opposing_grip_review.py → publish_clear_bridge_progress.py 순서로 문서를 갱신한 뒤 verify_preflight_artifacts.py를 실행한다.
