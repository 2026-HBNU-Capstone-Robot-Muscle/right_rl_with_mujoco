"""Test small root adjustments, then continuous simplified regrasp paths."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import CP,ROOT,R,assess,ramp
from research_rotation_body_hold_20261104 import restore,motor,inspect_for
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--index',type=int,required=True);ap.add_argument('--middle',type=int,default=32);ap.add_argument('--thumb',type=int,default=-24);ap.add_argument('--from-reference',action='store_true');ap.add_argument('--matrix',action='store_true');a=ap.parse_args()
    if not (-96<=a.index<=32 and -64<=a.middle<=64 and -64<=a.thumb<=32):raise ValueError('bounded local regrasp')
    seed=ROOT/'contact_preserving_free/nominal/end_g1_f0.3.npz';tag=f'i{a.index:+d}_m{a.middle:+d}_t{a.thumb:+d}';out=ROOT/'recentered_free'/tag/('from_reference_matrix' if a.matrix and a.from_reference else 'from_reference' if a.from_reference else 'matrix' if a.matrix else 'nominal');out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));sz=np.load(seed);goal=motor(sz['ctrl']);goal[7]+=a.index;goal[11]+=a.middle;goal[3]+=a.thumb;z=np.load(CP/'end_state.npz') if a.from_reference else sz;ref=restore(m,np.load(CP/'end_state.npz'));cube=m.geom('cube').id;reference={'C':ref.geom_xpos[cube].copy(),'B':ref.geom_xmat[cube].reshape(3,3).copy()};hand,_,_=inspect_for(m);cache=precompute(m,hand);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();cases=[];rows=ramp(motor(z['ctrl']),goal,134)
    np.savez_compressed(out/'suffix.npz',start=motor(z['ctrl']),goals=rows,phases=np.r_[['postturn_body_regrasp']*(len(rows)-134),['postturn_body_hold']*134])
    if a.from_reference:
        old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');assert np.array_equal(old['goals'][-1],motor(z['ctrl']));np.savez_compressed(out/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['postturn_body_regrasp']*(len(rows)-134),['postturn_body_hold']*134])
    for scale in ((.5,1.,1.5) if a.matrix else (1.,)):
      for friction in ((.2,.3,.5) if a.matrix else (.3,)):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;apply_material(m,friction,hand,cube);row=assess(m,z,goal,reference,cache,out/f'end_g{scale:g}_f{friction:g}.npz',134);row.update(gain_scale=scale,friction=friction);cases.append(row)
        (out/'review.json').write_text(json.dumps(dict(status='free_cube_recentered_body_grip_research',parameters=vars(a),cases=cases,all_suffix_filters_passed=all(c.get('filter_passed',False) for c in cases),unmodified_forward_prefix_source='right_hand_rotation_20261104/forward_hold/candidate.npz' if a.from_reference else None,full_placement_prefix_not_yet_replayed=True,no_cube_state_reset_within_suffix=True,original_dynamic_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('dynamic','goal','closest_geoms')}),flush=True)
if __name__=='__main__':main()
