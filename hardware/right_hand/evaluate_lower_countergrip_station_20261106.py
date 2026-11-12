"""Free replay of accepted nearby lower-grip IK endpoints; no cube resets."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,native,motor
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from upper_layer_cylinder_clearance_20261105 import precompute
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=('base','force2_span12_equilibrium'),default='base');args=ap.parse_args();root=ROOT/('lower_countergrip_station' if args.variant=='base' else 'lower_countergrip_station_'+args.variant);proposal_review=json.loads((root/'review.json').read_bytes());source=R/proposal_review['source_state'];z=np.load(source);model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';m=mujoco.MjModel.from_binary_path(str(model));lo,hi,_,_=native(m);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    vertices={g:v for g,v,e in precompute(m,hand)};results=[]
    for p in proposal_review['cases']:
        if not p['accepted_static_only']:continue
        folder=root/f"seed{p['seed']}";proposal=np.load(folder/'static_proposal.npz');d=restore(m,z);d.qpos[:16]=proposal['actual_joint_proposal'];d.qpos[-7:-4]=proposal['cube_center'];B=proposal['cube_rotation'];q=Rotation.from_matrix(B).as_quat();d.qpos[-4:]=np.r_[q[3],q[:3]];d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        for force in ((None,) if 'force_balanced_control' in proposal else (1.,1.5,2.5)):
            control=proposal['actual_joint_proposal'].copy()
            if force is None:control=proposal['force_balanced_control'].copy()
            else:
                for name,normal,active in (('if_tip',-B[:,2],np.arange(4)),('th_tip',B[:,2],np.arange(12,16))):
                    g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];proj=(world-proposal['cube_center'])@normal;weights=np.exp(-(proj-min(proj))/.0002);point=np.average(world,axis=0,weights=weights);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]));control[active]+=(d.qfrc_bias[active]-d.qfrc_passive[active]-(normal@jp[:,active])*force)/m.actuator_gainprm[active,0]
            # Other fingers retain the current actual, already recorded goals.
            if force is not None:control[4:12]=z['ctrl'][4:12]
            goal=motor(control);row=dict(seed=p['seed'],force_N=force,full_gravity_balanced_control_used=force is None,native_goal_rejected=bool(np.any(goal<lo) or np.any(goal>hi)));label='equilibrium' if force is None else f'force{force:g}'
            if not row['native_goal_rejected']:
                rows=ramp(motor(z['ctrl']),goal,134);np.savez_compressed(folder/f'direct_{label}.npz',start=motor(z['ctrl']),goals=rows);v=run(m,z,rows,1,folder/f'direct_{label}_end.npz',None,True,5,('if_tip','th_tip'));final=restore(m,np.load(folder/f'direct_{label}_end.npz'));targetB=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);reasons=quarter_reasons(v,proposal['cube_center'],True,orientation_error_deg(final.xmat[m.body('cube').id].reshape(3,3),targetB));row.update(dynamic=v,reasons=reasons,checkpoint90_and_grip_passed=not reasons)
            results.append(row);(root/'direct_replay_review.json').write_text(json.dumps(dict(status='nearby_lower_grip_free_cube_goal_replays',source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_proposal_review_sha256=sha(root/'review.json'),cases=results,cube_not_reset_during_replay=True,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(seed=p['seed'],force_N=force,native_goal_rejected=row['native_goal_rejected'],safety=row.get('dynamic',{}).get('safety_passed'),yaw=row.get('dynamic',{}).get('final_yaw_from_placement_deg'),reasons=row.get('reasons'))),flush=True)
if __name__=='__main__':main()
