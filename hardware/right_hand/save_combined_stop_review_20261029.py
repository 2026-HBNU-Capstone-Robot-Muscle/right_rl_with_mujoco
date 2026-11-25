"""Persist observations, preserve ready history, and mark no rerun yet."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'pitch_slow_yaw_20261029_004532_520638';OUT=RUN/'offline_review'
OUT.mkdir(exist_ok=True)
def read(p):return json.loads(p.read_bytes())
r=read(RUN/'result.json');frames=read(RUN/'camera/frames.json')
if isinstance(frames,dict):print('FRAME_KEYS',list(frames));frames=frames.get('frames',[])
print('FIRST_FRAME',json.dumps(frames[0],ensure_ascii=False))
print('SAMPLE_KEYS',list(r['samples'][-1]));print('RESULT_KEYS',list(r))
review=dict(status='stopped_during_regrasp_before_yaw',run=RUN.relative_to(ROOT).as_posix(),
    result_sha256=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest(),
    physical_pitch_qualitatively_observed=True,physical_transfer_retention_observed=True,
    physical_yaw_executed=False,exact_robot_frame90_verified=False,
    visual_scope='Green-up to red-up visible in saved images; no metric robot-frame registration claimed',
    photographs=['cube_before_pitch.jpg','cube_pitch_hold.jpg','cube_transfer_hold.jpg',
        'camera/frame_02216.jpg','camera/frame_02356.jpg','camera/frame_02361.jpg'],
    stop_phase=r['failed_phase'],error=r['error'],all_torque_off_verified=r['all_torque_off_verified'],
    settings_restored=r['settings_restored'],read_failures=r['read_failures'],
    unnecessary_retry_rejection=dict(error_elapsed_ms=r['read_failures'][0]['elapsed_ms'],
        remaining_of200ms=200-r['read_failures'][0]['elapsed_ms'],old_upfront_reserve_ms=120,
        correction='bounded actual quiet/drain, then unchanged100ms reserve;200ms total and two full healthy reads preserved'),
    next_action='Offline regression checks and a scoped slower-yaw-only experiment; wait for cube removal before fresh empty-hand readings')
(OUT/'physical_observation_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for relative in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
    'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/relative;original=p.read_bytes();backup=OUT/(p.name+'.before_stop_review.json')
    if not backup.exists():backup.write_bytes(original)
    current=json.loads(original)
    current.update(current_status='combined_stopped_before_yaw_20261029',latest_combined_run=review,
        next_physical_command_ready=False,next_step='Repair bounded READ resynchronization and prepare isolated retained-red-up slow yaw; cube-removal confirmation pending')
    p.write_text(json.dumps(current,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 최신 기록: 통합 실행은 뒤집기와 큐브를 유지한 자세 이동 후 다시 잡는 구간에서 통신 오류로 중단됐다. '
    '수평 회전은 아직 시작하지 않았다. 저장 사진에서 초록 윗면에서 빨강 윗면으로의 변경과 자세 이동 뒤 큐브 유지가 보인다. '
    '토크 OFF와 설정 복구를 확인했다. 전압 최저4.8V. 통신 오류 뒤 약119.3ms가 남았지만 기존 코드의 선예약120ms 때문에 재읽기를 시도하지 못했다. '
    '전체200ms와 정상 전체 상태2회 확인을 유지하면서 실제 조용해지는 시간을 반영하도록 수정 검증 중이다. '
    '수평 구간만 재검증하는 실행을 준비하며 아직 이전 명령의 반복 실행을 요청하지 않는다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8');backup=OUT/(name+'.before_stop_review')
    if not backup.exists():backup.write_text(old,encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
print('saved',OUT/'physical_observation_review.json')
print('AGENTS')
for parent in (ROOT,*ROOT.parents):
    p=parent/'AGENTS.md'
    if p.is_file():print(str(p),p.read_text(encoding='utf-8')[:12000])
print('RENDER HELPERS',[p.name for p in ROOT.glob('render*.py')][-30:])
