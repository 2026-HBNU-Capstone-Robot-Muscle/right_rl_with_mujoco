# 오른손 실물 후속 개발 작업 전체

이 폴더는 원본 `right_rl_with_mujoco`에서 시작해 실물 문제를 수정·확장해 온 개발 작업 전체다. 코드·원본 정책과 SAC 추가 학습 자료·모델·경로 후보·실물/시뮬레이션 검증·분석 사진·진행 및 인계 문서를 포함한다. 설치 환경·인증정보·대량 원시 촬영 폴더와 영상, 중복 체크아웃은 제외했다.

- 최신 상태: `cloud_handoff_20261106/CLOUD_START_PROMPT.md`와 `professor_progress_20261015.md`의 맨 위 항목
- 현재 관찰 시험: `leap_cycle_tip_first_transfer.cmd`
- 시험 안내: `right_hand_rotation_20261112/tip_first_trial_v2/operator_guide.md`
- 전체 원본 경로/해시: 저장소 루트의 `RIGHT_HAND_FULL_MANIFEST.json`
- 과거 스냅샷의 고유 환경 안내: `cloud_handoff_20261106/snapshot_support/`

현재 시험 범위는 주황 옆면 뒤쪽 접촉을 목표로 끝마디 → 다음 마디 순서로 미는 관찰이다. 최종 세 손가락 파지와 다음 회전은 아직 미완료다. 실제 실물은 저장 경로 재생이며, 새 SAC 학습·증류 또는 실시간 정책 구동이 완료됐다고 표현하지 않는다. 모델 검사 통과와 실제 성공을 구분한다.

현재 원래 PC의 Python 설치 환경 `.runtime`은 배포하지 않았다. 다른 PC의 포트/USB/모터 설정을 그대로 적용하지 않는다. 물리 실행은 사용자가 시작하며 기존450mA·PID·2틱/75ms와 모든 중단 기준을 유지한다.

GitHub에 올릴 때는 원본 저장소의 `feature/hardware` 브랜치에 이 `hardware` 폴더와 전체 목록 문서를 넣는다. `apps/monitoring_ui`는 수정하지 않는다. 이 파일 전달 과정에서는 GitHub에 업로드하지 않았다.
