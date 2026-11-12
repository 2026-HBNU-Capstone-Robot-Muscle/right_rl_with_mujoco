import json
from pathlib import Path
R=Path(__file__).resolve().parent;rel='right_hand_rotation_20261030/held_contact_release/research_summary.json';r=json.loads((R/rel).read_bytes());assert r['loaded_model_cases']==27 and r['hand_model_cases']==9 and not r['physical_rotation_hold_trial_ready']
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261030/four_forward_review/handoff.json','right_hand_rotation_20261030/placement_alignment_overlay/handoff.json'):
 p=R/name;d=json.loads(p.read_bytes());d.update(latest_rotation_hold_model_research=rel,rotation_hold_model_cases=27,rotation_hold_hand_cases=9,physical_rotation_hold_trial_ready=False);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''\n\n2026-10-30 회전 후 하부 고정과 현재 접촉 방향 해제 연구\n\n- held_contact_release/candidate.npz: 1701목표, 기존 하부 고정 경로 첫1597목표 동일. 실제로 진화한 모델 고정 상태의 손끝 접촉 법선 방향으로 천천히 분리하며, 기존 전체 OPEN으로 되돌리는 동작은 포함하지 않는다. 모터 경로/전류/PID/각도 한계를 늘리지 않았다.\n- 전체 자유 큐브 27조건과 손 동역학9조건 통과. 8초 하부 고정의 최대 회전 흔들림1.057도, 중심 이동2.049mm. 손·받침 겹침0. 새 해제의 기본 상태 별도 검사에서는 양쪽 손끝 정상력0N, 해제 중 회전 변화 최대1.506도/중심1.496mm. 이 힘은 모델 값이고 실물 측정이 아니다. 27조건의 마지막 손끝 해제 여부는 별도로 기록하지 않아 모두 풀렸다고 주장하지 않는다.\n- 전체 모델의 회전량60.535~63.744도. 실물90도 성공과 상단 레이어 작업 및 양손 연계는 아직 검증하지 않았다. 물리 실행 준비로 승격하지 않았으며 다음 사용자 행동은 노란 윤곽 배치 사진이다.\n- 자세한 출처 및 해시: right_hand_rotation_20261030/held_contact_release/research_summary.json. 모터 쓰기0.\n'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
 p=R/name;s=p.read_text(encoding='utf-8');assert '2026-10-30 회전 후 하부 고정과 현재 접촉 방향 해제 연구' not in s;p.write_text(s+note,encoding='utf-8')
print('Saved model research and physical-action handoff; only the placement-photo command is ready.')
