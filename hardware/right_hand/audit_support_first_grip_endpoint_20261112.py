"""Compare the prepared endpoint and latest saved trial without any devices."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer/endpoint_audit';OUT.mkdir(exist_ok=True)
z=np.load(ROOT/'right_hand_rotation_20261112/support_first_trial/candidate.npz',allow_pickle=False)
run=ROOT/'support_first_trial_20261112_075306_271459';r=json.loads((run/'result.json').read_bytes())
ids=[13,15,14,16];indices=np.array(ids)-1
report=dict(current_trial_scope='receiving_index_middle_pads_then_thumb_transfer_not_archived_TH_MF_RF_grip',
    RF_tip_to_palm_IDs=ids,current_start_RF_ticks=z['start'][indices].tolist(),current_end_RF_ticks=z['goals'][-1,indices].tolist(),
    RF_changed_commands=int(np.count_nonzero(np.any(z['goals'][:,indices]!=z['start'][indices],axis=1))),
    actual_run={k:r.get(k) for k in ('status','plan_version','primary_motion_error','error','all_torque_off_verified','settings_restored',
        'cube_support_confirmed_before_off','physical_transfer_success','physical_upper_clearance_verified','support_first_final_photo')},
    seen_phases=dict(Counter(s['phase'] for s in r.get('samples',[]))),
    photos=[p.relative_to(ROOT).as_posix() for p in run.glob('*.jpg')],
    matching_archived_three_finger_grip_not_implemented=True,devices_opened=0)
if r.get('samples'):
    s=r['samples'][-1];report['last_on_sample']=dict(phase=s['phase'],t=s['t'],actual=s['actual'],goal=s['goal'])
(OUT/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
