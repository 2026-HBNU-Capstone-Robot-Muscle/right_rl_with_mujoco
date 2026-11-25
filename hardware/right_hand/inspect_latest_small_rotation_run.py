"""Compact, read-only inspection; does not print bulk sample arrays."""
import json
from pathlib import Path
from collections import Counter
root=Path(__file__).resolve().parent
path=root/'leap_cube_small_rotation_trial_result.json'
r=json.loads(path.read_bytes())
keys=('timestamp','status','error','failed_phase','all_torque_off_verified','settings_restored',
      'rotation_command_path_completed','camera_recording','current_start_screen','final_off_reference_valid')
print(json.dumps({k:r.get(k) for k in keys},ensure_ascii=False,indent=2))
samples=r.get('samples',[])
print('PHASES',dict(Counter(s['phase'] for s in samples)))
for phase in ('stage_3_loaded_approach','stage_4_small_rotation','stage_2_fixed_hold'):
    rows=[s for s in samples if s['phase']==phase]
    if rows:
        print(phase,'first/last',json.dumps([{k:row.get(k) for k in ('t','goal','actual','current_ma')} for row in (rows[0],rows[-1])]))
