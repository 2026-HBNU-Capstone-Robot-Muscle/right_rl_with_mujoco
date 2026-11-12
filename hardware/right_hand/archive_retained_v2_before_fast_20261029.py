from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'retained_slow_yaw_20261029_014356_533789';OUT=RUN/'offline_review/execution_source_v2';OUT.mkdir(parents=True,exist_ok=True)
r=json.loads((RUN/'current_start_screen.json').read_bytes())
for name in ('combined_quiet_reserve_20261029.py','retained_slow_yaw_plan_20261029.py','retained_slow_yaw_gate_20261029.py',
    'leap_retained_slow_yaw_20261029.py','test_combined_quiet_reserve_20261029.py','test_retained_slow_yaw_operator_20261029.py'):
    data=(ROOT/name).read_bytes()
    if name in r['code_hashes']:assert hashlib.sha256(data).hexdigest()==r['code_hashes'][name],name
    target=OUT/name
    if not target.exists():target.write_bytes(data)
for name in ('request.json','review.json','readiness.json','verification.json'):
    target=OUT/name
    if not target.exists():target.write_bytes((ROOT/'right_hand_rotation_20261029/retained_slow_yaw_v1'/name).read_bytes())
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;d=json.loads(p.read_bytes());d.update(status='stopped_before_yaw_fast_read_work_in_progress',
        current_status='stopped_before_yaw_fast_read_work_in_progress',next_physical_command_ready=False,
        latest_retained_run=RUN.relative_to(ROOT).as_posix(),next_step='Integrate verified Fast READ; await cube removal before fresh empty recovery review')
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Archived actual v2 source; next motion not yet ready.')
