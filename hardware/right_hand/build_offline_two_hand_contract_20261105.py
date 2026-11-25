"""Shareable interface draft and exact logical routes; no transport or devices."""
import json,hashlib
from pathlib import Path
from cube_orientation_routes_20261028 import validate
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';routes=validate()
report=dict(status='idealized_orientation_routes_and_future_interface_draft',logical_orientation_states=len(routes),all24_reachable_under_ideal90_primitives=True,motor_paths_concatenated=False,physical90_primitives_verified=False,common_grip_repeat_entry_verified=False,routes=[dict(up=a,wrist=b,ideal_moves=moves) for (a,b),(_,moves) in sorted(routes.items())],orientation_planner_source_sha256=hashlib.sha256((R/'cube_orientation_routes_20261028.py').read_bytes()).hexdigest(),pose_contract=dict(frame_id='requires measured common fixture frame',origin='cube geometric center',position_unit='meter',rotation='right-handed3x3 matrix from cube axes to fixture axes',cube_axes_from_existing_configuration=dict(white=[0,0,1],yellow=[0,0,-1],red=[1,0,0],orange=[-1,0,0],blue=[0,1,0],green=[0,-1,0]),unknown_real_pose_values_are_null=True),actual_left_hand_torque_nm=None,hardware_execution_allowed=False,devices_opened=False)
(O/'integration_orientation_routes.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
text='''# 양손 연결 인터페이스 초안

현재는 연결 순서와 데이터 형식만 준비했다. 실제 전송 어댑터와 실행 명령은 없다. 목표는 오른손이 큐브의 방향을 바꾼 뒤 하단을 고정하고, 다른 손이 윗단을 돌리고 퇴각하면 다음 방향 변경으로 이어지는 것이다.

1. 오른손이 회전 후 공통 파지에 도착했다고 알린다. 큐브가 남아 있고 관절이 안정된 상태여야 하며, 목표 방향에 실제로 도달했는지도 구분한다.
2. 다른 손은 확인된 공통 좌표로 접근하고 준비 완료를 알린다. 오른손은 계속 같은 목표를 유지한다.
3. 다른 손이 윗단을 +90도 또는 −90도 돌리고 완료를 알린다. 오른손은 큐브 몸통을 계속 잡는다.
4. 다른 손이 큐브에서 완전히 빠진 뒤 퇴각 완료를 알린다.
5. 오른손이 다음 방향 변경을 시작한다. 현재는 이 연결 경로가 미완성이므로, 첫 배치용 경로를 그대로 다시 호출하지 않는다.

각 요청과 완료 응답은 같은 `token`을 사용한다. 중복 응답은 새 동작을 만들지 않는다. 실물에서는 각 손의 기존 상태 읽기와 중단 처리가 계속 작동해야 한다. 예제의 시간 값은 모의 실행용이다.

## 좌표와 기구 결합

큐브를 처음 놓는 곳과 다른 손이 작업할 최종 파지 곳은 달라도 된다. 통합 때 고정할 값은 최종 큐브 중심, 윗단 높이, 모서리 방향과 기울기다. 위치는 공통 프레임에서 미터 단위3차원 값으로, 자세는 오른손 좌표계의3×3 회전 행렬로 공유한다. 실제로 측정되지 않은 위치나 변환은 `null`로 남긴다.

팀에서 말한 바닥 기준 약160mm의 파지 높이는 왼손 측 설계값이다. 새 오른손 파지의 실제 중심 높이는 아직 확정하지 않았다. 이160mm를 시뮬레이션 좌표에 바로 넣지 않는다. 결합 때 오른손의 받침 높이를 조정하고, 현재 손가락의 이동 공간도 함께 확보한다.

실제 왼손이 큐브를 돌릴 때의 힘도 아직 측정되지 않았다. 모델의 ±0.01N·m는 가정한 비교 하중이다. 이를 왼손 모터의 허용 토크나 실물 검증 결과로 전달하지 않는다.

## 방향 처리

기준 표기는 카메라 앞면과 혼용하지 않고 ‘위’와 ‘손목 쪽 면’을 쓴다. 현재 이상적인 기준은 초록 위, 주황 손목 쪽이다. 기존 방향 정의에서 수평 동작은 초록 위를 유지하며 흰색이 손목 쪽으로 오는 방향이고, 수직 동작은 빨강이 위, 초록이 손목 쪽으로 오는 방향이다. 실물의 현재 정확한 회전각이90도라고 가정하지 않는다.

`integration_orientation_routes.json`에는 이상적인90도 동작일 때 가능한24개 방향 조합과 논리 경로가 있다. 실제 모터 경로의 연결 성공을 뜻하지 않는다. 큐브가 섞여도 면 중앙의 정체성은 유지되므로, 조각 색 전체의 변화와 큐브 몸통 방향을 구분한다. 중앙 마커 자체의 회전은 기존 자세 추정기에서 별도로 다룬다.

## 현재 넘길 수 있는 자료

- 현재 결과와 영상: `offline_review.html`
- 후보와 원본 해시: `offline_review.json`
- 양손 순서 모의 실행: `integration_dry_run_v3.json`
- 모델에서 통과한 새 파지: `tip_squeeze/id1/plus12/candidate.npz`

실물 새 파지와 목표 방향, 반복 연결이 확인되기 전에는 새 후보를 장치 제어 어댑터에 연결하지 않는다. 지금 사용자에게 새 촬영이나 장치 연결을 요청하는 문서는 아니다.
'''
(O/'two_hand_interface_draft.md').write_text(text,encoding='utf-8')
print(json.dumps(dict(logical_states=len(routes),maximum_ideal_steps=max(len(m) for _,m in routes.values()),hardware_execution_allowed=False,devices_opened=False)),flush=True)
