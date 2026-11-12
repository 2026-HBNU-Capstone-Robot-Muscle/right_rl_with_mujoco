# Latest hardware development update

<!-- hardware_update_20261112 -->
Read `cloud_handoff_20261106/CLOUD_START_PROMPT.md` first. Latest actual lower-grip run stopped on ID15 tracking. Distal-first thumb/middle repair is under offline review; the full rotation/hold/next-rotation cycle is not complete. `cloud/hardware_update_manifest_20261112.json` records the selected additions.

# 현재 상태: 2026-11-07

`star1209042/xlerobot` 비공개 저장소로 코드와 모델 및 인계 자료를 올리고 두 원격 브랜치의 첫 커밋과 파일 목록을 확인했다. GitHub 인증은 완료됐다. 기본 작업 브랜치는 `main`, 인계 체크포인트는 `codex/cloud-handoff-20261106`이다.

이제 Codex 웹에서 저장소를 연결하고 환경을 만들면 된다. `cloud/SETUP.md`를 따르고 첫 요청은 `cloud/START_PROMPT.txt`를 사용한다. 실제 클라우드 환경 생성/게시와 계산은 아직 확인하지 않았다.

로봇 목표는 원하는 방향 약 90도 회전, 하부 파지와 윗단 공간 확보, 다음 회전 연결이다. 양손 통합과 전체 풀이가 끝난 상태는 아니다. 가장 최근 `middle_clearance_probe_20261106_230902_998326/result.json`은 중지 관찰 완료, 토크 OFF, 설정 복구와 큐브 제거를 확인했다. 해당 폴더의 `cube_grasp_hold.jpg`와 `middle_probe_hold.jpg` 효과는 아직 검토하지 않았다.

소스와 결과 원본 3,519개 약 496MB를 선별했다. 원래 38.7GB 전체의 백업이나 로컬 대화 원문 전체의 동기화는 아니다. 원본 Windows 자료는 보존했다. 현재 동작 결합 원본 170개는 원래 SHA256을 유지한다. 장치 명령 대신 오프라인 작업을 이어간다.

상세 문맥은 `cloud_handoff_20261106/CLOUD_START_PROMPT.md`와 `professor_progress_20261015.md`에 있다. `cloud/transfer_status.json`은 첫 업로드의 실제 확인 기록이다. 이후 변경의 원격 HEAD와 구분해서 읽는다.
