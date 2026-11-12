"""Inspect the measured fixed-goal placement plateau; no devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'horizontal_quarter_trial_20261027_221539_291945'
r=json.loads((folder/'result.json').read_text(encoding='utf-8'))
rows=[s for s in r['samples'] if s['phase']=='placement_checkpoint']
positions=np.array([s['actual'] for s in rows]);goals=np.array([s['goal'] for s in rows]);error=goals-positions
frames=json.loads((folder/'camera/frames.json').read_text(encoding='utf-8'))['frames']
selected=min(frames,key=lambda f:abs(f['camera_read_monotonic']-(r['enabled_at']+rows[-1]['t'])))
report=dict(status='observed_settled_open_offset',checkpoint_samples=len(rows),
    checkpoint_duration_s=rows[-1]['t']-rows[0]['t'],position_span_ticks=np.ptp(positions,axis=0).tolist(),
    maximum_span_ticks=int(np.ptp(positions,axis=0).max()),goal_span_ticks=np.ptp(goals,axis=0).tolist(),
    error_min_ticks=error.min(axis=0).tolist(),error_max_ticks=error.max(axis=0).tolist(),
    median_error_ticks=np.median(error,axis=0).tolist(),median_actual_ticks=np.median(positions,axis=0).tolist(),
    last_actual=rows[-1]['actual'],goal=rows[-1]['goal'],
    original_limit_ticks=64,largest_error_id=int(np.unravel_index(np.abs(error).argmax(),error.shape)[1])+1,
    maximum_error_ticks=int(abs(error).max()),
    minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),
    maximum_current_ma=max(max(abs(c) for c in s['current_ma']) for s in r['samples']),
    torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    nearest_checkpoint_image=str((folder/'camera'/selected['image']).relative_to(ROOT)),
    final_off_position=r['final_off_snapshots'][-1]['position'],motor_writes=0)
out=ROOT/'right_hand_rotation_20261027/open_settle_221539';out.mkdir(exist_ok=True)
(out/'measured_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
for name in ('horizontal_quarter_motion_runner_20261027.py','camera_gap_motion_runner_20261026.py','camera_gap_current_runtime_20261026.py'):
    text=(ROOT/name).read_text(encoding='utf-8').splitlines()
    for i,line in enumerate(text):
        if 'Open preparation has not settled' in line:
            print(name,'line',i+1)
            for j in range(max(0,i-12),min(len(text),i+8)):print(f'{j+1}: {text[j]}')
