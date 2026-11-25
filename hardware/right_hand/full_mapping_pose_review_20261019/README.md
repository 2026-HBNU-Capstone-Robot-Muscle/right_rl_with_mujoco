## 2026-10-19: 새 ID 순서로 사진 재구성·동역학 재검증 완료

네 손가락 번호는 10월18일 사용자 확인 순서를 유지했다. 실물 ID·영점·게인을
변경하지 않았으며 실제 모터/카메라 접근도 없었다. 저장 사진과 동시 엔코더
4쌍으로 전체 손을 다시 재구성했다. 손끝/뿌리 임의 ±90° 보정은 하지 않았다.

실제 중지가 엄지 하우징에 닿던 10월17일01:51 자세:
- 기존 매핑의 모델 상자 간격61.42mm → 새 매핑9.44mm.
- 렌더링 메시의 볼록 껍질로 계산하면 최소 간격 하한 약1.615mm.
  이는 단순 충돌 상자의 누락을 보여주지만 실물 간격 측정/영점 보정 인증은 아니다.
- 손바닥 마커 등록은 아직 잠정값이다. ID40가 안 잡힌 접촉 사진에는 자동
  투영을 만들지 않았다. 나머지3쌍은 기존 해부학 기준 변환으로 조건부 투영했다.
  ID9을 뿌리라고 붙인 옛 마커 좌표를 새 ID9 실측 좌표로 재해석하지 않는다.

이전 ‘119/120 목표 자세 겹침,54/120 관절 범위 초과’의 해석을 보완한다.
목표각을 qpos에 강제로 넣은 정적 검사와 물리 엔진의 실제 움직임은 다르다.
해당 경로의 범위 초과는3관절에서 틱 반올림으로1틱 미만 경계를 넘은 경우였다.
경계 안쪽 정수 틱으로 고쳤고, 새 후보의 모든 목표가 관절/제어 범위 안에 있다.
큰 자세 오류나 실물54회 충돌이라는 의미가 아니다.

새 순서로 재인코딩한 경로를 native MuJoCo에서 재생했다. 잘못된 ID16 해석에서
유래한 가상의 rf_dip 기계 한계는 모델에서 제거했다. 중지/엄지 외형 메시의
볼록 충돌 형상도 별도로 추가해 민감도 검사를 했다. 질량·관성·관절 수는
유지했고, 추가 형상끼리 충돌하며 큐브/받침에 중복 적용되지 않는 것을 검증했다.

최종 후보: 빈손5개/큐브55mm·100g5개 저장 시작 조건을 각각 재생.
큐브5개 모두 약89.87~89.95° 회전, 목표 자세 오차0.15~4.02°,
목표 유지2.65~2.85초. 모델 받침 관통0mm, 손 내부 최대 soft-contact 겹침
약2.19mm. 겹침 없는 강체 경로 또는 실물 성공을 뜻하지 않는다.
**SAC에서 얻어 보정한 고정 경로이며 live SAC 재학습/실물 큐브 맞추기 완료가 아니다.**
모델 액추에이터와 실제 PID/마찰, 실제 배치 및 시작 동작 검증은 남아 있다.

결과: `full_mapping_pose_review_20261019/summary.json` 및 `README.md`.
영상: `full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/simulated_path.gif`.
경로는 분석 전용 형식이고 hardware_execution_allowed=false. 기존 잘못된
매핑 기반 시작/경로/loaded policy 차단은 유지한다. raw-ID 원본 시퀀스는 재배열하지 않는다.

다음 사용자 작업은 **현재 자세 PING/READ 한 번**이다. 단독 읽기 파일은10월16일,
짝사진 자료는10월18일이고 그 뒤 사용자가 번호를 확인했다. 새 준비 경로의
출발점은 현재 엔코더로 받아야 한다. 큐브 제거, 다른 모터 프로그램 종료,
USB/모터 전원 연결, 토크는 이미OFF 상태에서 `hand_readonly_check.cmd` 실행.
사진/수동 관절 조작/큐브 올리기 요청 없음. 이 명령은 목표·게인·토크를 쓰지 않는다.
그 결과로 새 번호에 맞는 빈손 준비 동작부터 계산·검증한다.

---

검증 파일:
- `analysis.json`: 저장 사진/엔코더의 기존·수정 해석과 모델 간격.
- `*_comparison.jpg`: 저장 사진 + 두 시점의 모델 비교(카메라 정합 그림 아님).
- `*_overlay_*.png`, `projection.json`: 잠정 마커 등록을 사용한 조건부 투영.
- `contact_mesh_clearance.json`: OBB와 독립 비교한 볼록 메시 거리;3개 수치 검사 통과.
- `housing_proxy_verification.json`: 합성 겹침으로 충돌 필터 활성 확인.
- `inward_quantized/housing_sensitivity/dynamic_replay.json`: 최종10회 동역학 기록.
- `inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json`: 구동 금지 분석용 후보.

재현 스크립트는 outputs의 `audit_full_mapping_poses.py`,
`project_full_mapping_poses.py`, `audit_contact_mesh_clearance.py`,
`audit_remapped_path_dynamics.py --inward-quantization --housing-proxies`.
실제 모터 접근을 구현하지 않는다. 수치 결과는 시뮬레이터 버전에 따라 달라질 수 있다.
