"""Compare passive support alternatives while the middle finger withdraws.

No specific finger pair is required for this exploratory task filter. Full
contact, range, tracking and retention safety checks remain unchanged.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,evaluate,motor,inspect_for,angle,native
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from cube_contact_material_policy_20261029 import apply_material
from dynamic_common_grip_search_20261105 import CP,ROOT,R,ramp

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--thumb-backoff',type=int,default=0);a=ap.parse_args()
    if not 0<=a.thumb_backoff<=48:raise ValueError('bounded offline thumb release')
    out=ROOT/f'middle_release/thumb_backoff{a.thumb_backoff}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));source=ROOT.parent/'preintegration/body_grip/lower_m20_t16_n10';z=np.load(source/'end_state.npz');d=restore(m,z);cube=m.geom('cube').id;B0=d.geom_xmat[cube].reshape(3,3).copy();C0=d.geom_xpos[cube].copy();hand,_,_=inspect_for(m);apply_material(m,.3,hand,cube);cache=precompute(m,hand);start=motor(z['ctrl']);open_goal=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz')['start'];lo,hi,_,_=native(m);cases=[]
    for frac in (.25,.5,.75,1.):
        goal=start.copy();goal[8:12]=np.rint(start[8:12]+(open_goal[8:12]-start[8:12])*frac).astype(int);goal[1]-=a.thumb_backoff
        if np.any(goal<lo) or np.any(goal>hi):cases.append(dict(fraction=frac,native_rejected=True));continue
        rows=ramp(start,goal,134);path=out/f'fraction{frac:g}.npz';dynamic=evaluate(m,z,rows,path);fd=restore(m,np.load(path));B=fd.geom_xmat[cube].reshape(3,3);C=fd.geom_xpos[cube];cl=gaps(m,fd,C,B,cache,True);margin=min(v for name,v in cl);yaw=angle(B,B0,z['up']);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        row=dict(fraction=frac,goal=goal.tolist(),goals=len(rows),upper_sweep_gap_mm=margin,yaw_change_deg=yaw,tilt_deg=tilt,center_shift_mm=float(np.linalg.norm(C-C0)*1000),dynamic=dynamic,resting_pose_filter_passed=bool(dynamic['safety_passed'] and margin>.3 and abs(yaw)<5 and tilt<5 and dynamic['hold_drift_deg']<3 and dynamic['hold_shift_mm']<2),force_resistance_not_yet_tested=True);cases.append(row)
        np.savez_compressed(out/f'suffix{frac:g}.npz',start=start,goals=rows)
        (out/'review.json').write_text(json.dumps(dict(status='alternative_support_grip_offline_research',parameters=vars(a),cases=cases,checkpoint_is_nominal_only=True,full_prefix_not_replayed=True,MF_TH_contact_requirement_is_not_used_for_this_alternative=True,all_original_safety_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in row.items() if k!='dynamic'}|{'dynamic':{k:v for k,v in dynamic.items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
