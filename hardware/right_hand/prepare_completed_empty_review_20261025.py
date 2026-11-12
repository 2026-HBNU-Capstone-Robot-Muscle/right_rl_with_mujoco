"""Summarize completed empty motion and select saved frames; no device access."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'reverse_quarter_resume_empty_20261025_090335_740374'
OUT=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'
OUT.mkdir(exist_ok=True)
r=json.loads((SOURCE/'result.json').read_bytes());frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
summary={k:r.get(k) for k in ('status','all_torque_off_verified','settings_restored','ring_release_measured_verified','last_issued_command','planned_final_command')}
summary.update(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256((SOURCE/'result.json').read_bytes()).hexdigest(),
    max_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
    peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in r['samples']),
    min_voltage_v=min(min(s['voltage_v']) for s in r['samples']),max_temperature_c=max(max(s['temperature_c']) for s in r['samples']),
    final_on_error_ticks=(np.array(r['samples'][-1]['goal'])-r['samples'][-1]['actual']).tolist(),
    motor_on_elapsed_seconds=r['samples'][-1]['t'],samples=len(r['samples']),camera_frames=len(frames),
    camera_usable_frames=sum(f.get('image_signal_usable',False) for f in frames),
    cube_rotation_physically_tested=False,whole_path_continuously_completed=False,
    empty_path_completed_across_separate_runs=True,proof_scope='Empty recovery and previously unexecuted 288-goal suffix only; original 472 goals ran before an intervening OFF.')
selected=[]
for phase,position in (('stage_5_empty_quarter_start_hold',-1),('stage_6_empty_quarter_reference_path',0),('stage_6_empty_quarter_reference_path',.5),('stage_6_empty_quarter_reference_path',-1),('stage_2_fixed_hold',-1),('post_off_reference',-1)):
    a=[f for f in frames if f['phase']==phase]
    if not a:continue
    f=a[int(len(a)*position) if isinstance(position,float) else position];selected.append(dict(**f,path=(SOURCE/'camera'/f['image']).as_posix()))
summary['selected_frames']=selected
(OUT/'completion_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('selected_frames','last_issued_command','planned_final_command')},ensure_ascii=False))
for f in selected:print('FRAME',f['phase'],f['path'])
print('RESULT_KEYS',list(r))
