"""Summarize saved grasp attempts without opening devices or dumping traces."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261116/final_grip_attempt_review'
OUT.mkdir(parents=True,exist_ok=True)
rows=[]
for folder in sorted(ROOT.glob('final_grip_pose_20*')):
    path=folder/'result.json'
    if not path.is_file():
        rows.append(dict(run=folder.name,status='no_result_file'));continue
    r=json.loads(path.read_bytes())
    row=dict(run=folder.name,**{k:r.get(k) for k in (
        'timestamp','status','error','failed_phase','primary_failed_phase','last_issued_phase',
        'all_torque_off_verified','settings_restored','current_start_screen_accepted',
        'final_pose_reached','hold_end_reason')})
    row['writes']=len(r.get('write_events',[]));samples=r.get('samples',[])
    row['samples']=len(samples)
    for k in ('progress_fault','fixed_hold_fault','post_off_restore_error','final_check_error'):
        if k in r:row[k]=r[k]
    row['initial_ticks']=r.get('preflight_state',{}).get('position')
    row['final_command']=r.get('last_issued_command',r.get('final_command'))
    if samples:
        s=samples[-1];row['last_sample']={k:s.get(k) for k in (
            'phase','t','position','goal','current_ma','tracking_rad','voltage_v','error') if k in s}
    failures=r.get('read_failures',[])
    row['communication_failures']=len(failures)
    if failures:row['last_read_failure']={k:failures[-1].get(k) for k in (
        'address','length','comm','error','attempt','received_packet_bytes','expected_packet_bytes')}
    proof_path=folder/'current_start_screen.json'
    if proof_path.exists():
        p=json.loads(proof_path.read_bytes());row['review_status']=p.get('status')
    models={}
    for sub in ('empty_recovery_screen','full_pose_screen'):
        path=folder/sub/'review.json'
        if path.is_file():
            p=json.loads(path.read_bytes());models[sub]=dict(all_passed=p.get('all_passed'),
                static_failed=sum(not x.get('passed') for x in p.get('static_cases',[])),
                dynamic_failed=sum(not x.get('passed') for x in p.get('dynamic_cases',[])))
    if models:row['model_checks']=models
    rows.append(row)
(OUT/'attempts.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for row in rows:
    compact={k:v for k,v in row.items() if k not in ('initial_ticks','final_command','last_sample')}
    if row.get('last_sample'):
        s=row['last_sample'];compact['last_sample']=dict(phase=s.get('phase'),
            current_max_ma=max(map(abs,s.get('current_ma') or [0])),
            tracking_max_rad=max(s.get('tracking_rad') or [0]))
    print(json.dumps(compact,ensure_ascii=False))
