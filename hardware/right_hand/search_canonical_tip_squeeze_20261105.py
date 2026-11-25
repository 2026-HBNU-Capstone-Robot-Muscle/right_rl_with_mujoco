"""Small contact-normal-directed endpoint comparison, offline checkpoint only."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import CP,ROOT,R,ramp
from research_rotation_body_hold_20261104 import motor,inspect_for,native
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons
from evaluate_canonical_grip_reaction_20261105 import evaluate
from cube_contact_material_policy_20261029 import apply_material

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--motor',type=int,choices=(1,2,11),required=True);a=ap.parse_args()
    out=ROOT/f'tip_squeeze/id{a.motor}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz')
    base=np.load(ROOT/'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz')
    old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');start=motor(z['ctrl'])
    if not np.array_equal(start,old['goals'][-1]):raise ValueError('prefix/checkpoint mismatch')
    hand,_,_=inspect_for(m);apply_material(m,.3,hand,m.geom('cube').id);lo,hi,_,_=native(m);cases=[]
    for delta in ((4,8,12) if a.motor==1 else (4,8)):
        began=time.perf_counter();folder=out/f'plus{delta}';folder.mkdir(exist_ok=True);goal=base['goals'][-1].copy();goal[a.motor-1]+=delta
        if np.any(goal<lo) or np.any(goal>hi):raise ValueError('native range')
        rows=ramp(start,goal,134);end=folder/'end_state.npz';d=run(m,z,rows,1,end,None,True);why=task_reasons(d);loads=[]
        if not why:
            for torque in (-.01,.01):loads.append(evaluate(m,np.load(end),torque,folder/f'load{torque:+g}.npz'))
        candidate=folder/'candidate.npz';np.savez_compressed(candidate,start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['postturn_body_regrasp']*(len(rows)-134),['postturn_body_hold']*134])
        row=dict(motor=a.motor,delta_ticks=delta,dynamic=d,suffix_reasons=why,hypothetical_reaction=loads,local_filter_passed=not why and len(loads)==2 and all(c['load_filter_passed'] for c in loads),candidate=str(candidate.relative_to(R)),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),elapsed_seconds=time.perf_counter()-began);cases.append(row)
        (out/'review.json').write_text(json.dumps(dict(status='static_jacobian_guided_free_cube_squeeze_probes',cases=cases,full_placement_prefix_not_replayed=True,source_goal_family='selected_v2',current_and_PID_and_dynamic_guards_unchanged=True,actual_layer_load_not_measured=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
        print(json.dumps(dict(motor=a.motor,delta=delta,suffix_reasons=why,yaw=d['regrasp_yaw_change_deg'],gap=d['settled_tail']['minimum_separating_plane_gap_1ms_mm'],loads=[dict(torque=c['hypothetical_reaction_torque_nm'],passed=c['load_filter_passed'],yaw=c['max_yaw_change_deg'],reasons=c['reasons']) for c in loads],accepted=row['local_filter_passed'],elapsed_seconds=row['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
