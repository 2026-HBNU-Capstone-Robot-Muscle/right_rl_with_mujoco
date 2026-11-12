import json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import LOADED_PATH
ROOT=Path(__file__).resolve().parent;O=ROOT/'right_hand_rotation_20261029/reverse_cube_scout'
z=np.load(LOADED_PATH);mapping={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold','release':'stage_9_loaded_open','remove_hold':'stage_10_remove_hold'}
np.savez_compressed(O/'forward_phase_cycle.npz',start=z['start'],goals=z['goals'],phases=np.asarray([mapping[str(p)] for p in z['phases']]))
from loaded_quarter_reentry_20261025 import limits
lo,hi=limits();g=z['goals'][z['phases']=='grasp'][-1];y=z['goals'][z['phases']=='yaw']
print(json.dumps(dict(motor3=dict(base=int(g[2]),lo=int(lo[2]),hi=int(hi[2]),yaw_range=[int(y[:,2].min()),int(y[:,2].max())]),motor10=dict(base=int(g[9]),lo=int(lo[9]),hi=int(hi[9]),yaw_range=[int(y[:,9].min()),int(y[:,9].max())]))))
