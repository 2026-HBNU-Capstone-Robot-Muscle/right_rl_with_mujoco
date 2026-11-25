# 큐브 접촉 관측 — Full 프로필

먼저 [빈손 준비와 정지](LEAP_next_run.md)를 확인한다.
이 실행은 시작 자세를 고정하고 큐브를 올린 상태를5초 기록한다. 정책 출력은 전송하지 않는다.

```powershell
& "C:\Users\김준서\Documents\Codex\2026-09-23\files-mentioned-by-the-user-leap\outputs\leap_contact.cmd"
```

1. 손을 고정하고 비운다. 시작 전 큐브가 카메라에 보일 필요는 없다. 낙하 위치에 부드러운 받침을 둔다.
2. 카메라 창에서 SPACE → 3초 카운트다운 → 빈손 준비. 이 시작 단계에는 마커 조건이 없다.
3. **PLACE CUBE NOW**가 뜬 후 큐브를 가볍게 올린다. 사람 손을 뺀 뒤 **카메라 또는 실행 터미널에서 SPACE**.
4. 고정 자세로5초 관측한다. ID40과 큐브 마커2개가 보이도록 한다.
5. **REMOVE CUBE NOW**가 뜨면 큐브를 치우고 **카메라 또는 실행 터미널에서 SPACE** → 토크 OFF/설정 복원.

배치·제거는 시간 초과 없이 SPACE를 기다린다. 전체60/90초 제한은 없다.
Q/Esc/Ctrl+C, 통신·온도·전류·추종 오류는 대기 중에도 OFF로 이어진다.
큐브를 억지로 끼우지 않는다. 이상 시 OFF되면 큐브가 떨어질 수 있다.

공통 프로필은16개550mA, 측정전류650mA 초과 중단, PID는 관절별 기존 장치 값 유지다.
직전 실측은 P2100/I0/D1900이며 예제PID로 낮추거나 옆방향 게인을 새로 곱하지 않는다.
속도·정체 검사·근거는 [현재 설정](LEAP_next_run.md)에 정리했다.

결과는 `leap_contact_result.json` 및 시각별JSON에 저장한다.
`contact_fixed_hold_complete`와 OFF/원복true가 정상 완료다. `policy_actions_sent=0`은 정상이다.
`contact_samples`에 전류·추종·큐브 pose, `contact_inferences`에 미전송 정책 출력,
`contact_summary`에 요약, `contact_key_events`에 SPACE 단계/입력창이 남는다.
배치·제거 확인은 사용자 입력이며 접촉 센서 판정은 아니다.
좌표 변환은 정성적으로 확인한 후보값이므로 기록을 정밀 계측으로 간주하지 않는다.
