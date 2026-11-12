"""Probe small thumb corrections with a free cube after middle retreat.

The palm/index/thumb grip is a separate research hypothesis. Passing a geometric
screen does not replace the prior MF+TH certificate or prove reaction resistance.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import motor,native
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--coupled',action='store_true');args=ap.parse_args()
    parent=ROOT/'near_quarter_full_replay';r=json.loads((parent/'review.json').read_bytes());model=R/r['source_model'];assert sha(model)==r['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/'postturn_middle_clearance/end_state.npz';z=np.load(source);start=motor(z['ctrl']);lo,hi,_,_=native(m);out=ROOT/('postturn_thumb_coupled' if args.coupled else 'postturn_thumb_clearance');out.mkdir(exist_ok=True);results=[]
    for motor_id in ((3,4) if args.coupled else (1,2,3,4)):
      for delta in (-64,-32,32,64):
        goal=start.copy();goal[motor_id-1]+=delta
        if args.coupled:goal[0]-=64
        if np.any(goal<lo) or np.any(goal>hi):continue
        case=out/f'id{motor_id}_delta{delta:+d}';case.mkdir(exist_ok=True);rows=ramp(start,goal,134);np.savez_compressed(case/'candidate.npz',start=start,goals=rows);v=run(m,z,rows,1,case/'end_state.npz',None,True);tail=v['settled_tail'];contact=v['final_contact_N']
        why=[]
        if not v['safety_passed']:why.append('original_dynamic_bound')
        if abs(v['final_yaw_from_placement_deg']-90)>=5 or abs(v['regrasp_yaw_change_deg'] or 0)>=5 or v['final_up_tilt_deg']>=5:why.append('orientation')
        if v['regrasp_translation_mm'] is None or v['regrasp_translation_mm']>=2:why.append('position')
        if tail['separation_certificate_bad_steps_1ms']!=0 or tail['certificate']['queries']!=4050 or tail['minimum_separating_plane_gap_1ms_mm']<=.3:why.append('exact_upper_space')
        if tail['yaw_drift_deg']>=3 or tail['center_shift_mm']>=2:why.append('hold_drift')
        if contact.get('palm_collision_1',0)<=.2 or contact.get('if_px_collision',0)<=.2 or contact.get('th_tip',0)<=.2:why.append('three_named_contact_pressures')
        results.append(dict(motor_id=motor_id,delta_ticks=delta,also_ID1_minus64=args.coupled,dynamic=v,palm_index_thumb_geometry_screen_passed=not why,reasons=why,force_closure_and_reaction_resistance_not_yet_proved=True))
        report=dict(status='postturn_thumb_geometry_sensitivity_not_grip_certification',source_state=str(source.relative_to(R)),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,only_thumb_goal_changes=True,native_force_geometry_and_dynamic_guards_unchanged=True,prior_MF_TH_grip_acceptance_unchanged=True,hardware_execution_allowed=False,devices_opened=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(id=motor_id,delta=delta,safety=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],gap=tail['minimum_separating_plane_gap_1ms_mm'],thumb_N=contact.get('th_tip',0),screen=not why,reasons=why)),flush=True)
if __name__=='__main__':main()
