"""Seek and freely replay lower grips after the saved near-quarter motion.

Only the IK proposal pins the cube. Every evaluated transition starts from the
unchanged recorded free-cube state; no pose, force or device writes occur.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--focus',action='store_true');ap.add_argument('--refine-after-fit',action='store_true');args=ap.parse_args()
    if args.refine_after_fit and not args.focus:raise ValueError('refinement requires focused proposal')
    case=ROOT/'quasistatic_wrench_path/quarter_cap0.15_native_cone_potential_contacts_hybrid_motion_lift4_phase5_margin8'
    source=json.loads((case/'review.json').read_bytes());model=(R/source['source_state']).parent/'model.mjb'
    assert sha(model)==source['source_model_sha256'] and source['guard_stop'] is None
    z=np.load(case/'end_state.npz');m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];lo,hi,nlo,nhi=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,edges in geometry};active=np.r_[np.arange(8),np.arange(12,16)]
    low=np.maximum((np.ceil(lo[ids])+8-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-8-2048)*TICK-pd,nhi)
    preferred=np.load(ROOT/'torque_cap_sensitivity/cap0.15/end_state.npz')['qpos'][:16]
    out=ROOT/('postturn_regrasp_refined' if args.refine_after_fit else ('postturn_regrasp_focused' if args.focus else 'postturn_regrasp'));out.mkdir(exist_ok=True);results=[];began=time.perf_counter()
    # Keep a complete independent fixed-goal replay of the rotation checkpoint.
    raw=np.load(case/'suffix.npz')['goals'];fixed_rows=np.vstack([raw,np.tile(raw[-1],(134,1))]);initial=np.load(R/source['source_state'])
    if not args.focus:
        replay=run(m,initial,fixed_rows,len(raw),out/'rotation_hold_end.npz',out/'rotation_hold_trajectory.npz',True)
        (out/'rotation_hold_review.json').write_text(json.dumps(dict(status='near_quarter_recorded_goals_with_fixed_hold',source_review=str((case/'review.json').relative_to(R)),source_review_sha256=sha(case/'review.json'),source_suffix_sha256=sha(case/'suffix.npz'),dynamic=replay,whole_cube90_and_common_grip_not_certified=True,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    for finger in (('middle',) if args.focus else ('middle','index')):
      for mode in (('zneg',) if args.focus else ('xpos','xneg','zpos','zneg')):
        folder=out/f'{finger}_{mode}';folder.mkdir(exist_ok=True);contacts=[]
        for role,name in enumerate(('mf_tip' if finger=='middle' else 'if_tip','th_tip')):
            g=m.geom(name).id;axis=0 if mode.startswith('x') else 2;normal=B[:,axis]*(1 if mode.endswith('pos') else -1)*(1 if role==0 else -1);side=np.cross(up,normal);side/=np.linalg.norm(side);contacts.append((g,normal,side))
        def measure(x,exact=False):
            C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);tasks=[];details=[]
            for g,normal,side in contacts:
                world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];project=(world-C)@normal;gap=float(min(project)-.0275);weights=np.exp(-(project-min(project))/.0002);point=np.average(world,axis=0,weights=weights);height=float((point-C)@up)*1000;lateral=float((point-C)@side)*1000;tasks.extend([(gap*1000+.05)*5,lateral*.4,(height+2)*.5]);details.append(dict(geom=m.geom(g).name,gap_mm=gap*1000,height_mm=height,side_mm=lateral,point=point.tolist()))
            clearance=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.asarray(tasks),details,clearance,max(sp.values(),default=0.),max(su.values(),default=0.),pen
        do_inside=args.focus and not args.refine_after_fit
        def residual(x):
            task,detail,clear,selfmm,supportmm,pen=measure(x)
            inside=[max(0.,abs(t['side_mm'])-20)*10 for t in detail] if do_inside else []
            return np.r_[task,inside,(x[:-3]-preferred[active])*1.,np.maximum(.8-np.asarray([v for name,v in clear]),0)*10,x[-3:]*.3,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        movement=15. if args.focus and not args.refine_after_fit else 8.;lower=np.r_[low[active],[-movement]*3];upper=np.r_[high[active],[movement]*3];seed=np.r_[np.clip(preferred[active],low[active]+1e-9,high[active]-1e-9),np.zeros(3)]
        fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        if args.refine_after_fit:
            do_inside=True;lower[-3:]=-15.;upper[-3:]=15.;fit=least_squares(residual,fit.x,bounds=(lower,upper),max_nfev=220,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        task,detail,clear,selfmm,supportmm,pen=measure(fit.x,True)
        np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=d.qpos[-7:-4],cube_rotation=B,optimizer_vector=fit.x,static_cube_pose_is_not_dynamic_evidence=True)
        accepted=bool(all(abs(t['gap_mm'])<.2 and abs(t['side_mm'])<22 and -22<t['height_mm']<7 for t in detail) and min(v for name,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
        record=dict(finger=finger,mode=mode,static_proposal_accepted=accepted,contacts=detail,upper_gap_mm=min(v for name,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,proposal_translation_mm=fit.x[-3:].tolist(),dynamic=[])
        if accepted:
            proposed_q=d.qpos.copy();start=motor(z['ctrl']);roles=('mf_tip' if finger=='middle' else 'if_tip','th_tip')
            for force in (1.,1.5):
                control=proposed_q[:16]+pd
                for (g,normal,side),entry in zip(contacts,detail):
                    j=np.zeros((3,m.nv));mujoco.mj_jac(m,d,j,None,np.asarray(entry['point']),int(m.geom_bodyid[g]));a=np.arange(4,8) if g==m.geom('mf_tip').id else (np.arange(4) if g==m.geom('if_tip').id else np.arange(12,16));tau=d.qfrc_bias[a]-d.qfrc_passive[a]-(normal@j[:,a])*force;control[a]=proposed_q[a]+tau/m.actuator_gainprm[a,0]
                goal=motor(control)
                if np.any(goal<lo) or np.any(goal>hi):record['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                rows=ramp(start,goal,134);suffix=folder/f'force{force:g}.npz';np.savez_compressed(suffix,start=start,goals=rows)
                result=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True,5,roles)
                record['dynamic'].append(dict(force_N=force,result=result))
        results.append(record)
        report=dict(status='postturn_static_proposals_and_free_regrasp_replays',source_review=str((case/'review.json').relative_to(R)),source_review_sha256=sha(case/'review.json'),source_state_sha256=sha(case/'end_state.npz'),source_model_sha256=sha(model),cases=results,cube_pinned_only_in_static_proposals=True,free_transitions_do_not_reset_cube=True,native_geometry_and_dynamic_guards_unchanged=True,physical_grip_not_tested=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(dict(finger=finger,mode=mode,static=accepted,gap_mm=record['upper_gap_mm'],dynamic_pass=[v.get('result',{}).get('whole_grip_task_passed',False) for v in record['dynamic']])),flush=True)
if __name__=='__main__':main()
