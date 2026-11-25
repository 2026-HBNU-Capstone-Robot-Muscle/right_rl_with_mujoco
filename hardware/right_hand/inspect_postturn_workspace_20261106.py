"""Identify the actual saved shapes that obstruct the upper-layer workspace."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def main():
    r=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());m=mujoco.MjModel.from_binary_path(str(R/r['source_model']));hand,_,_=inspect_for(m);cache=precompute(m,hand)
    cases=[('near_quarter_full_replay',ROOT/'near_quarter_full_replay/end_state.npz'),('postturn_middle_clearance',ROOT/'postturn_middle_clearance/end_state.npz')]
    for name,p in cases:
        z=np.load(p);d=restore(m,z);cube=m.geom('cube').id;cl=gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),cache,True);result=dict(source=str(p.relative_to(R)),closest_geoms=sorted(cl,key=lambda x:x[1])[:8],qpos=d.qpos[:16].tolist(),ctrl=d.ctrl.tolist(),devices_opened=False,hardware_execution_allowed=False);(p.parent/'workspace_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(name=name,closest=result['closest_geoms'][:5])),flush=True)
if __name__=='__main__':main()
