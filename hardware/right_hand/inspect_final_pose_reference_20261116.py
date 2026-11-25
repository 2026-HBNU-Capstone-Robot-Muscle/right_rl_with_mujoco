import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
source=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz'
z=dict(np.load(source,allow_pickle=False));print('KEYS',list(z))
lo,hi=limits()
for key in ['qpos','ctrl']:
    q=z[key][:16];ticks=np.empty(16,dtype=int)
    ticks[np.array(SIM_TO_MOTOR_IDS)-1]=np.rint(q*4096/(2*np.pi)+2048).astype(int)
    print(key,'TICKS',ticks.tolist(),'OUTSIDE',[(i+1,int(ticks[i]),float(lo[i]),float(hi[i])) for i in range(16) if not lo[i]<=ticks[i]<=hi[i]])
print('OPEN',np.load(ROOT/'right_hand_rotation_20261027/new_pair_grasp/open_goal.npz').files if (ROOT/'right_hand_rotation_20261027/new_pair_grasp/open_goal.npz').exists() else 'use OPEN_PATH')
