"""Select original saved frames at observed transfer stages, without image editing."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TRIAL=ROOT/'lower_grip_trial_20261112_162806_610535'
r=json.loads((TRIAL/'result.json').read_bytes());f=json.loads((TRIAL/'camera/frames.json').read_bytes())
samples=r['samples'];grasp=[s for s in samples if s['phase']=='stage_5_loaded_grasp'][-1]
offset=r['grasp_observation_photo']['camera_read_monotonic']-grasp['t']
chosen=[]
for phase in ('stage_18_receiving_hold','stage_19_thumb_middleward','stage_22_transfer_hold','stage_23_ring_support'):
    s=[s for s in samples if s['phase']==phase][-1];t=s['t']+offset
    times=[row['camera_read_monotonic'] for row in f['frames']]
    matching=[i for i,row in enumerate(f['frames']) if row.get('phase')==phase]
    index=matching[-1] if matching else min(range(len(times)),key=lambda i:abs(times[i]-t))
    frame=f['frames'][index]
    chosen.append(dict(phase=phase,frame=frame,index=index,matching_recorded_phase=bool(matching),exact_phase_boundary_registration=False))
print(json.dumps(dict(chosen=chosen,offset=offset),ensure_ascii=False))
(ROOT/'right_hand_rotation_20261112/lower_grip_actual_review/frame_selection.json').write_text(json.dumps(chosen,indent=2,ensure_ascii=False),encoding='utf-8')
