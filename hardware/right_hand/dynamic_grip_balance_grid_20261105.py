"""Two-joint free-cube equilibrium search, with existing dynamic guards."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,hashlib
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import R,ROOT,CP,SEED,OUT,assess
from research_rotation_body_hold_20261104 import restore,motor,inspect_for
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--thumb',type=int,required=True);ap.add_argument('--middle',type=int,nargs='+',default=[0,32,64,96,128]);ap.add_argument('--tip',type=int,default=0);a=ap.parse_args()
    if not all(-192<=v<=192 for v in [a.thumb,a.tip,*a.middle]):raise ValueError('bounded search grid')
    out=OUT/f'balance_t{a.thumb}_tip{a.tip}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(SEED);rz=np.load(CP/'end_state.npz');rd=restore(m,rz);cube=m.geom('cube').id;reference={'C':rd.geom_xpos[cube].copy(),'B':rd.geom_xmat[cube].reshape(3,3).copy()};hand,_,_=inspect_for(m);apply_material(m,.3,hand,cube);cache=precompute(m,hand);start=motor(z['ctrl']);cases=[]
    for middle in a.middle:
        goal=start.copy();goal[1]+=a.thumb;goal[9]+=middle;goal[8]+=a.tip;save=out/f'middle{middle:+d}.npz';row=assess(m,z,goal,reference,cache,save)
        row.update(thumb_mcp_delta_ticks=a.thumb,middle_rot_delta_ticks=middle,middle_tip_delta_ticks=a.tip,final_state=str(save.relative_to(R)));cases.append(row)
        report=dict(status='free_cube_local_balance_grid_only',seed_source=str(SEED.relative_to(R)),seed_sha256=hashlib.sha256(SEED.read_bytes()).hexdigest(),cases=cases,checkpoint_is_nominal_only=True,full_prefix_not_replayed=True,original_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('dynamic','goal','closest_geoms')}),flush=True)
if __name__=='__main__':main()
