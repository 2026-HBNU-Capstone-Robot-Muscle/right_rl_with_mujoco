"""Save a shorter, already executed grasp prefix; no hardware interface."""
import hashlib
import json
from pathlib import Path
import numpy as np
from grasp_place_plan import endpoints,LOADED_PHASE
from grasp_recovery_plan import interpolate
ROOT=Path(__file__).resolve().parent
source=ROOT/'book_cube_grasp_trial_20261024_172726_286561/result.json'
data=source.read_bytes();r=json.loads(data)
if not (r['all_torque_off_verified'] and r['settings_restored'] and r['cube_placement_confirmed']):raise ValueError('Unverified trial source')
loaded=[s for s in r['samples'] if s['phase']==LOADED_PHASE]
sample=loaded[279]  # sample 280 observes the already sent goal 279
opened,closed=endpoints();full=[];interpolate(full,opened,closed,LOADED_PHASE)
count=279
if not np.array_equal(full[count-1][1],sample['goal']):raise ValueError('Observed command is not the exact old path prefix')
if (max(sample['tracking_rad'])>.09 or max(abs(v) for v in sample['current_ma'])>150):raise ValueError('Selected prefix is not the bounded observed point')
out=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(data).hexdigest(),
    phase=LOADED_PHASE,observed_sample_number=280,loaded_target_count=count,
    shortened_goal=sample['goal'],observed_actual_ticks=sample['actual'],observed_current_ma=sample['current_ma'],
    observed_max_tracking_rad=max(sample['tracking_rad']),
    original_loaded_target_count=len(full),old_path_target_sha256=hashlib.sha256(json.dumps([q.tolist() for _,q in full],separators=(',',':')).encode()).hexdigest(),
    physical_grasp_verified=False,
    note='Stop before the later ID12 plateau accumulated 64 ticks of error. No further closing, gain or current increase.')
path=ROOT/'book_grasp_reached_prefix_20261024.json'
path.write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out))
