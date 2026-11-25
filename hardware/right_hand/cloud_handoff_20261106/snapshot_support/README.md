# LEAP 오른손 큐브 조작: 클라우드 인계

VS Code에서 진행하던 로봇 프로젝트의 코드와 선별한 모델 및 근거 자료다. 개인 계정의 비공개 저장소로 먼저 관리하고 추후 팀 쪽으로 이전한다. 원래 로컬 대화 전체나 38.7GB 작업 폴더 전체를 내보낸 것은 아니다.

먼저 [CURRENT_STATE.md](CURRENT_STATE.md)와 [인계문](cloud_handoff_20261106/CLOUD_START_PROMPT.md)을 읽는다. 현재 목표의 모든 방향 회전, 공통 하단 파지 및 양손 통합은 아직 완료되지 않았다.

GitHub 업로드는 확인했다. Codex 웹에 연결하는 순서는 [클라우드 설정](cloud/SETUP.md), 환경 게시 뒤 사용할 첫 요청은 [시작 요청문](cloud/START_PROMPT.txt)에 있다. 실제 클라우드 환경 생성과 계산은 별도 확인해야 한다.

## 장치 없이 시작

Linux 또는 Codex Cloud에서 Python 가상환경을 사용한다.

```sh
python -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-cloud.txt
python -B cloud/smoke.py
```

설치 전에 원본 파일 검증만 하려면 `python -B cloud/smoke.py --verify-only`를 실행한다. 이 검사는 파일 출처와 저장된 모델을 여는지 확인한다. 목표 파지 성공이나 전체 경로 재검증은 아니다.

기존 모델 영상은 `right_hand_rotation_20261106/middle_clearance_probe/evidence/index.html`에서 확인한다. 최신 실물 사진은 `middle_clearance_probe_20261106_230902_998326/`에 있다. 이미지가 가리거나 정합되지 않은 부위를 성공으로 단정하지 않는다.

## 범위와 환경

- `cloud/upload_manifest.json`: 복사한 파일의 SHA256과 포함 범위.
- `cloud/environment.json`: 실제 사용 중이던 시뮬레이션 버전과 공개 소스 커밋.
- `requirements-cloud.txt`: 저장된 MJB/NPZ를 읽는 최소 환경. 그래픽 렌더링은 별도 EGL/OSMesa 또는 디스플레이 구성이 필요하다.
- 전체 과거 모델 재생성/학습에는 JAX 및 MuJoCo Playground와 원래 CAD 자산 등 추가 의존성이 필요하다. 최소 검사 통과를 그 환경 전체의 이식 완료로 해석하지 않는다.

원본 Windows의 USB와 웹캠은 클라우드에서 사용할 수 있다고 가정하지 않는다. `.cmd`, 장치 실행 또는 `--execute` 대신 저장 자료 분석과 오프라인 모델을 사용한다. 현재 실물은 사전 계산 경로를 재생하며 실시간 카메라 SAC 학습을 하는 상태가 아니다.
