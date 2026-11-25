"""Match the visually located fall to recorded joint motion and queued goals."""
import bisect
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'nominal_two_turn_trial_20261112_060746_992877'
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/cube_departure'
r=json.loads((RUN/'result.json').read_bytes());ss=r['samples'];times=[s['t'] for s in ss]
z=np.load(ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/rf16_2177_v2/candidate.npz',allow_pickle=False)
groups={'TH':[1,2,3,4],'IF':[5,7,6,8],'MF':[9,11,10,12],'RF':[13,15,14,16]}
points=[]
for t in (243.,246.375,247.438,247.954,248.469,249.422):
    i=min(range(len(times)),key=lambda k:abs(times[k]-t));s=ss[i]
    matches=np.flatnonzero(np.all(z['goals']==np.asarray(s['goal']),axis=1))
    points.append({'t':s['t'],'phase':s['phase'],'matching_loaded_goal_indices_1based':(matches+1).tolist(),
                   'torque_all_ON':all(v==1 for v in s['torque']),'hardware_error_zero':all(v==0 for v in s['error']),
                   'min_voltage_v':min(s['voltage_v']),
                   'joints':{name:{'IDs_tip_to_palm':ids,'actual':[s['actual'][j-1] for j in ids],
                                   'goal':[s['goal'][j-1] for j in ids],'current_ma':[s['current_ma'][j-1] for j in ids]}
                             for name,ids in groups.items()}})
first,last=points[1],points[4]
changes={name:{'IDs':g['IDs_tip_to_palm'],
               'actual_change_ticks':(np.asarray(last['joints'][name]['actual'])-g['actual']).tolist(),
               'goal_change_ticks':(np.asarray(last['joints'][name]['goal'])-g['goal']).tolist()}
          for name,g in first['joints'].items()}
result={'status':'visual_fall_joint_correlation_not_contact_force_measurement','points':points,
        'changes_246p375_to248p469':changes,'stop_t':ss[-1]['t'],'devices_opened':0}
(OUT/'joint_correlation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'changes':changes,'before':points[1],'fall':points[4],'stop_t':ss[-1]['t']}))
