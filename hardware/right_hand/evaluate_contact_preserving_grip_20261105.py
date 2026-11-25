"""Replay a contact-preserving proposal with a fully free cube."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import CP,ROOT,R,assess,ramp
from research_rotation_body_hold_20261104 import restore,motor,inspect_for
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--matrix',action='store_true');a=ap.parse_args()
    source=ROOT/'coordinated_refinement/turn18_normal10';endpoint=ROOT/'contact_preserving_endpoint/n3_m1/endpoint.npz';out=ROOT/'contact_preserving_free'/('matrix' if a.matrix else 'nominal');out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(source/'end_state.npz');ep=np.load(endpoint);ref=restore(m,np.load(CP/'end_state.npz'));cube=m.geom('cube').id;reference={'C':ref.geom_xpos[cube].copy(),'B':ref.geom_xmat[cube].reshape(3,3).copy()};hand,_,_=inspect_for(m);cache=precompute(m,hand);gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();cases=[];goal=ep['goal'].astype(int);rows=ramp(motor(z['ctrl']),goal,134)
    np.savez_compressed(out/'suffix.npz',start=motor(z['ctrl']),goals=rows)
    old=np.load(source/'candidate.npz');assert np.array_equal(old['goals'][-1],motor(z['ctrl']));np.savez_compressed(out/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['postturn_preserve_clear']*(len(rows)-134),['postturn_preserve_hold']*134])
    for scale in ((.5,1.,1.5) if a.matrix else (1.,)):
      for friction in ((.2,.3,.5) if a.matrix else (.3,)):
        m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;apply_material(m,friction,hand,cube);row=assess(m,z,goal,reference,cache,out/f'end_g{scale:g}_f{friction:g}.npz',134);row.update(gain_scale=scale,friction=friction);cases.append(row)
        (out/'review.json').write_text(json.dumps(dict(status='free_cube_contact_preserving_suffix_research',source=str(source.relative_to(R)),endpoint=str(endpoint.relative_to(R)),cases=cases,all_suffix_filters_passed=all(c.get('filter_passed',False) for c in cases),full_prefix_not_yet_replayed=True,geometry_native_and_force_bounds_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('dynamic','goal','closest_geoms')}),flush=True)
if __name__=='__main__':main()
