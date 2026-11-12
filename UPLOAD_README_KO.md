# 오른손 전체 개발 자료 — GitHub 수동 업로드용

이 ZIP은 기존의 작은 경로 모듈 ZIP이나23MB 시험용 선별 ZIP을 대신하는 전체 개발 자료 묶음이다. 현재 작업 폴더에서 접근 가능한 코드·모델·SAC 학습/체크포인트·실물 및 오프라인 검증 결과·분석 사진·인계 자료를 포함한다. 22,000파일, 원본 약5.98GB다.

## GitHub에 넣는 위치

압축을 풀면 `hardware/right_hand/` 안에 기존 `outputs`의 상대 경로가 보존돼 있다. 원본 팀 저장소 `2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco`의 로컬 체크아웃 루트에 `hardware` 폴더를 넣는다. 기존 `deploy`, `train`, `policy`, `vision`과 함께 있는 구조다. `feature/hardware` 브랜치에서 커밋·푸시한다. `apps/monitoring_ui`는 수정하지 않는다.

모델 파일 일부는 약65MB라 GitHub 웹 업로드의25MiB 제한을 초과한다. 파일 수도 웹 업로드의 한 번100개 제한보다 많다. GitHub Desktop 또는 Git을 사용한다. ZIP 파일 자체를 저장소에 올리는 것이 아니라 압축을 푼 소스 폴더를 올린다. 큰 초기 업로드는 코드/모델과 과거 결과를 나눠 각각 커밋 후 푸시하면 관리하기 쉽다.

공식 안내: https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository
파일 크기 안내: https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github

## 포함 범위와 보존

- 코드와 현재 실물 실행기 및 안전 감시 코드
- 원본 배포 정책과 접근 가능한 SAC 추가 학습·체크포인트·NumPy 내보내기
- 날짜별 경로 후보, 모델, 초기/종료 상태, 실패를 포함한 검증 결과
- 실물 로그와 분석 사진, 최신 인계·진행 기록
- 원본170개 결합 검증 소스의 해시 일치

설치된 Python/가상환경, 인증·계정 정보, 원시 영상, `camera` 원시 프레임 폴더, 중복 Git 체크아웃과 전달용 압축 파일은 제외했다. 정확한 목록과 SHA256은 `RIGHT_HAND_FULL_MANIFEST.json`에 있다. 전체 PC 백업이나 원시 기록 전체 백업은 아니다. 원래 로컬 파일은 변경하거나 삭제하지 않았다.

## 현재 동작 상태

현재 새 시험은 주황 옆면 뒤쪽 접촉을 목표로 끝마디 → 다음 마디 순서로 미는 관찰 시험이다. 최종 세 손가락 파지와 다음 회전은 미완료다. 모델 검사 통과를 실제 성공으로 표시하지 않았다. 새 SAC 학습이나 증류가 완료됐다는 주장도 하지 않는다.

Windows 원래 실행 환경의 `.runtime`은 포함하지 않았다. 다른 PC의 포트·USB·장치 설정을 그대로 적용하지 않는다. 실물 실행은 사용자가 명령과 SPACE로 시작하며 보조자가 직접 구동한 것은 없다.

이 ZIP을 만드는 동안 GitHub에는 업로드하지 않았다. 사용자가 다시 명시적으로 지시하기 전에는 자동 업로드하지 않는다.
