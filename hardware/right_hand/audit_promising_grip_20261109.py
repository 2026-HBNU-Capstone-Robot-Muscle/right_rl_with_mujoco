from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore
from research_taught_grasp_20261106 import details

source=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
model=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':source.read_bytes()})
folder=ROOT/'right_hand_rotation_20261109/index_thumb_transfer/cp1042_together'
z=np.load(folder/'end.npz',allow_pickle=False)
report=details(model,z)
(folder/'endpoint_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'worst_upper_gaps':report['upper_gaps_mm'][:12],'contacts':report['contacts'],
                  'goal':report['goal_ticks'],'actual':report['actual_ticks']}))
dynamic=json.loads((folder/'review.json').read_bytes())['dynamic']
print(json.dumps({key:dynamic[key] for key in ('final_up_tilt_deg','regrasp_yaw_change_deg','regrasp_translation_mm','final_contact_N','settled_tail')}))
