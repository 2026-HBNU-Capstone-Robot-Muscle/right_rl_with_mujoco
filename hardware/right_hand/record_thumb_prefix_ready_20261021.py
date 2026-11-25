"""Persist readiness of a bounded user-run diagnostic, not SAC completion."""
import hashlib
import json
from pathlib import Path
import thumb_prefix_plan as p
from leap_thumb_prefix_empty import validate_offline_review
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_preparation_20261021'
validate_offline_review()
raw=p.source_position();end=p.plan(raw)[-1][1]
files=['thumb_prefix_plan.py','leap_thumb_prefix_empty.py','empty_trial_camera.py','leap_thumb_prefix_empty.cmd',
       'test_leap_thumb_prefix_empty.py','test_empty_trial_camera.py']
review=dict(status='bounded_empty_observation_ready_for_user',source_sha256=p.SOURCE_SHA,
    files_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},
    source_ticks=raw.tolist(),final_goal_ticks=end.tolist(),thumb_delta_ticks=(end[:4]-raw[:4]).tolist(),
    tests=dict(new_trial_and_camera_tests=16,existing_root_tests=20,hardware_executed_by_assistant=False),
    static_cases=67,original_model_dynamics_valid=False,
    diagnostic_model_variant='ID3 upper joint/control bound temporarily admits the measured start plus 36 ticks; isolated in-memory sensitivity only. No persisted model or hardware limit changes.',
    full_preparation_validated=False,live_sac=False,physical_cube_flip_verified=False,
    operator_task='Empty hand; keep current posture/camera. Run leap_thumb_prefix_empty.cmd, SPACE in terminal. Camera records automatically; Q stops. No cube or phone recording required.')
(OUT/'thumb_prefix_ready.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21: 다음 빈손 준비 시험 준비됨 — 뿌리 + 작은 엄지 이동

- 사용자 작업: `leap_thumb_prefix_empty.cmd`. 현재 자세 그대로, 큐브 없이, 카메라에 엄지·중지를 보여준 뒤 터미널 SPACE. 한 명령으로 카메라 프레임과 모터 기록을 저장하며 끝나면 자동 토크 OFF. Q/Esc/Ctrl+C 정지. 기존 뿌리 시험/사진 촬영 반복이나 폰 촬영은 필요 없음.
- 새 명령은 뿌리 ID8/12/16을 2472로 올린 뒤 엄지 4개 관절을 준비 경로의 첫 구간만 이동. 원본 촬영 기준 엄지 변화 [83, 89, -96, 67]틱; 실제 시작각에 맞춰 최대 96틱(8.4375°) 이내로 계산. 나머지 굽힘·회전 관절 목표 고정. 전체 준비·큐브 회전·SAC 실행이 아니다.
- 현재 사진·각도와 해시를 묶고 시작 오차 ±32틱 안에서만 진행. OFF/ON 변화는 별도로 ≤4틱과 두 번의 안정된 읽기를 확인. 실제 정수 경로를 큐에 넣어 순서가 다른 목표/자동 재전송을 차단한다. EEPROM/모드/홈 오프셋 변경 없음.
- 67개 시작 조건 정적 경로: 자기 겹침 없음, 초기 받침 겹침 증가 없음, 엄지 구간 받침 겹침 없음. 일부 시작 표본의 근사 받침 겹침은 뿌리 상승 중 줄어듦. 전 범위/실물 형상 보증은 아니다.
- 원래 시뮬레이터는 ID3 시작각이 모델 범위 밖이어서 유지 명령만 줘도 약 0.659rad 이동시킴. 원래 모델 동역학을 통과했다고 주장하지 않는다. 측정 시작각을 수용한 격리된 진단 모델에서만 gain 0.5/1/1.5 조건의 최대 오차 0.065/0.033/0.022rad, 겹침 없음. 이 진단은 실제 PID나 관절 범위 보정의 증명이 아니며 배포 모델 한계를 수정하지 않았다.
- 새 시험/카메라 테스트 16개, 기존 뿌리 테스트 20개 통과. 통신/전압/추종/고정 관절 이동/카메라/Q/목표 쓰기 오류 및 엄지 진행 정지 시 OFF·설정 복원 검증. 모터 실물 구동은 하지 않음.
- 근거: `right_hand_preparation_20261021/thumb_prefix_screen.json`, `thumb_prefix_ready.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    f=ROOT/name;text=f.read_text(encoding='utf-8')
    if note.splitlines()[0] not in text:f.write_text(note+text,encoding='utf-8')
    else:f.write_text(text.replace('새 시험/카메라 테스트 15개','새 시험/카메라 테스트 16개'),encoding='utf-8')
f=ROOT/'right_hand_start_20261019/objective_and_schedule.json';x=json.loads(f.read_text(encoding='utf-8'))
x['current_user_task']=review['operator_task']
x['current_blocker']='Bounded empty root+thumb-prefix observation awaits user run. Full preparation and SAC still unvalidated; native thumb model range mismatch documented.'
x['latest_offline_review']='right_hand_preparation_20261021/thumb_prefix_ready.json'
f.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Ready record saved. No camera or motor device opened.')
