"""Small free-cube MCP adjustments around the near-clear aligned grip."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import CP,ROOT,R,assess
from research_rotation_body_hold_20261104 import restore,motor,inspect_for
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--middle-root',type=int,required=True);ap.add_argument('--thumb-root',type=int,nargs='+',default=[-8,-16,-24]);a=ap.parse_args()
    if not all(-64<=v<=64 for v in [a.middle_root,*a.thumb_root]):raise ValueError('small grid bounds')
    source=ROOT/'contact_preserving_free/nominal/end_g1_f0.3.npz';out=ROOT/f'near_clear_grid/middle{a.middle_root:+d}';out.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(source);ref=restore(m,np.load(CP/'end_state.npz'));cube=m.geom('cube').id;reference={'C':ref.geom_xpos[cube].copy(),'B':ref.geom_xmat[cube].reshape(3,3).copy()};hand,_,_=inspect_for(m);apply_material(m,.3,hand,cube);cache=precompute(m,hand);start=motor(z['ctrl']);cases=[]
    for thumb in a.thumb_root:
        goal=start.copy();goal[11]+=a.middle_root;goal[3]+=thumb;save=out/f'thumb{thumb:+d}.npz';row=assess(m,z,goal,reference,cache,save,134);row.update(middle_root_delta_ticks=a.middle_root,thumb_root_delta_ticks=thumb,final_state=str(save.relative_to(R)));cases.append(row)
        (out/'review.json').write_text(json.dumps(dict(status='near_clear_free_cube_refinement_only',source=str(source.relative_to(R)),cases=cases,full_prefix_not_yet_replayed=True,hardware_execution_allowed=False,devices_opened=False,original_dynamic_guards_unchanged=True),indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('dynamic','goal','closest_geoms')}),flush=True)
if __name__=='__main__':main()
