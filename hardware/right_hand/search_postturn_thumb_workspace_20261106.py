"""Global thumb-only lower-grip proposals after the near-quarter rotation.

Sampling/IK pins the cube only to propose endpoints. Endpoints are not motion
proofs. Free-cube replays, native limits and geometry remain separate checks.
No motor, camera, serial transport or physical execution option is provided.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.stats import qmc
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--samples-power',type=int,choices=(10,12),default=12);ap.add_argument('--seeds-per-face',type=int,choices=(3,6),default=6);ap.add_argument('--finger',choices=('thumb','index'),default='thumb');ap.add_argument('--from-staged',action='store_true');ap.add_argument('--contact-height-mm',type=float,choices=(-5.,-12.),default=-5.);args=ap.parse_args()
    if args.from_staged and args.finger!='index':raise ValueError('staged source is for the index lower regrasp')
    parent=ROOT/'near_quarter_full_replay';raw=json.loads((parent/'review.json').read_bytes());model=R/raw['source_model'];assert sha(model)==raw['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model))
    source=ROOT/'postturn_thumb_clearance/id1_delta-64/end_state.npz'
    if args.from_staged:source=ROOT/'countergrip_level/seed0_source1_lift0_normal1.5_n900_resume_lower_staged/end_state.npz'
    z=np.load(source);d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];hand,_,check=inspect_for(m)
    thumb_proposal=None
    if args.finger=='index' and not args.from_staged:
        thumb_proposal=ROOT/'thumb_global_workspace/p12_seeds6/zpos_3/static_proposal.npz';base[12:16]=np.load(thumb_proposal)['actual_joint_proposal'][12:16];d.qpos[:]=base;mujoco.mj_forward(m,d)
    prefix='th_' if args.finger=='thumb' else 'if_';tip_name=prefix+'tip';all_geometry=precompute(m,hand);thumb_geometry=[v for v in all_geometry if m.body(int(m.geom_bodyid[v[0]])).name.startswith(prefix)];moving_ids={v[0] for v in thumb_geometry};other_geometry=[v for v in all_geometry if v[0] not in moving_ids];vertices={g:v for g,v,e in all_geometry};tip=m.geom(tip_name).id
    lo,hi,nlo,nhi=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;active=np.arange(12,16) if args.finger=='thumb' else np.arange(4);low=np.maximum((np.ceil(lo[ids])+8-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-8-2048)*TICK-pd,nhi);lower=low[active];upper=high[active]
    if args.finger=='index':
        radius=.5 if args.from_staged else .2;lower[0]=max(lower[0],base[0]-radius);upper[0]=min(upper[0],base[0]+radius)
    constant_gap=min(v for name,v in gaps(m,d,C,B,other_geometry,True));start=motor(z['ctrl'])
    tag='index_lower_regrasp_workspace' if args.from_staged else ('thumb_global_workspace' if args.finger=='thumb' else 'index_countergrip_workspace')
    out=ROOT/tag/(f'p{args.samples_power}_seeds{args.seeds_per_face}'+(f'_h{args.contact_height_mm:g}' if args.contact_height_mm!=-5 else ''));out.mkdir(parents=True,exist_ok=True);began=time.perf_counter();results=[]
    samples=qmc.scale(qmc.Sobol(4,scramble=True,seed=20261106).random_base2(args.samples_power),lower,upper);samples=np.vstack([np.clip(base[active],lower+1e-9,upper-1e-9),samples]);scores={mode:[] for mode in (('xpos','xneg','zpos','zneg') if args.finger=='thumb' else ('zneg',))}
    def set_pose(x):
        d.qpos[:]=base;d.qpos[active]=x;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d)
        return vertices[tip]@d.geom_xmat[tip].reshape(3,3).T+d.geom_xpos[tip]
    def face_task(world,mode):
        axis=0 if mode.startswith('x') else 2;normal=B[:,axis]*(1 if mode.endswith('pos') else -1);side=np.cross(up,normal);side/=np.linalg.norm(side);projection=(world-C)@normal;weights=np.exp(-(projection-min(projection))/.0002);point=np.average(world,axis=0,weights=weights)
        return (float(min(projection)-.0275)*1000,float((point-C)@up)*1000,float((point-C)@side)*1000,point,normal)
    for index,x in enumerate(samples):
        world=set_pose(x)
        # This is only a seed-ranking heuristic; final exact checks below are
        # unchanged and never inferred from this rough score.
        for mode in scores:
            gap,height,side,point,normal=face_task(world,mode);score=(gap+.05)**2*4+(height-args.contact_height_mm)**2+max(0.,abs(side)-20)**2*4+side**2*.02;scores[mode].append(float(score))
        if index%1024==0:print(json.dumps(dict(stage='global_seed_sampling',sample=index,total=len(samples))),flush=True)
    np.savez_compressed(out/'sampled_workspace.npz',joint_proposals=samples,**{k:np.asarray(v) for k,v in scores.items()})
    report=dict(status='global_finger_static_workspace_and_independent_free_replays',finger=args.finger,contact_height_target_mm=args.contact_height_mm,static_thumb_proposal_sha256=None if thumb_proposal is None else sha(thumb_proposal),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),sampling_count=len(samples),seeds_per_face=args.seeds_per_face,constant_other_finger_upper_gap_mm=constant_gap,native_geometry_force_caps_and_dynamic_guards_unchanged=True,cube_pinned_only_for_static_proposals=True,free_replays_never_reset_cube=True,perfect_simulated_pose_used=True,physical_grip_not_verified=True,devices_opened=False,hardware_execution_allowed=False,cases=results)
    for mode,ranking in scores.items():
        selected=[]
        for index in np.argsort(ranking):
            x=samples[index]
            if all(np.linalg.norm(x-prev)>.3 for prev in selected):selected.append(x)
            if len(selected)>=args.seeds_per_face:break
        for seed_index,seed in enumerate(selected):
            folder=out/f'{mode}_{seed_index}';folder.mkdir(exist_ok=True)
            def measure(x,exact=False):
                world=set_pose(x);gap,height,side,point,normal=face_task(world,mode);clear=gaps(m,d,C,B,thumb_geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return gap,height,side,point,normal,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
            def residual(x):
                gap,height,side,point,normal,clear,selfmm,supportmm,pen=measure(x)
                return np.r_[(gap+.05)*10,(height-args.contact_height_mm),side*.1,max(0.,abs(side)-20)*10,(x-seed)*.1,np.maximum(.8-np.asarray([v for name,v in clear]),0)*20,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
            fit=least_squares(residual,np.clip(seed,lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=140,ftol=1e-9,xtol=1e-9,gtol=1e-9);gap,height,side,point,normal,clear,selfmm,supportmm,pen=measure(fit.x,True);minimum_gap=min(constant_gap,min(v for name,v in clear));accepted=bool(abs(gap)<.2 and -22<height<7 and abs(side)<22 and minimum_gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
            np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B,seed=seed)
            row=dict(mode=mode,seed_index=seed_index,static_accepted=accepted,face_gap_mm=gap,contact_height_mm=height,contact_side_mm=side,upper_gap_mm=minimum_gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,dynamic=[])
            if accepted:
                actual=d.qpos[:16].copy();jac=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jac,None,point,int(m.geom_bodyid[tip]));bias=d.qfrc_bias[active]-d.qfrc_passive[active]
                for force in ((.5,1.,1.5,2.5) if args.from_staged else (.5,1.,1.5)):
                    control=actual+pd;control[active]=actual[active]+(bias-(normal@jac[:,active])*force)/m.actuator_gainprm[active,0];goal=motor(control)
                    if args.finger=='index':
                        th=m.geom('th_tip').id;world=vertices[th]@d.geom_xmat[th].reshape(3,3).T+d.geom_xpos[th];_,_,_,thpoint,thnormal=face_task(world,'zpos');tj=np.zeros((3,m.nv));mujoco.mj_jac(m,d,tj,None,thpoint,int(m.geom_bodyid[th]));control[12:16]=actual[12:16]+(d.qfrc_bias[12:16]-d.qfrc_passive[12:16]-(thnormal@tj[:,12:16])*force)/m.actuator_gainprm[12:16,0];goal=motor(control)
                    if np.any(goal<lo) or np.any(goal>hi):row['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                    rows=ramp(start,goal,134);np.savez_compressed(folder/f'force{force:g}.npz',start=start,goals=rows);v=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True,5,('if_tip','th_tip') if args.finger=='index' else ('mf_tip','th_tip'));row['dynamic'].append(dict(force_N=force,result=v))
            results.append(row);report['elapsed_seconds']=time.perf_counter()-began;(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(mode=mode,seed=seed_index,static=accepted,gap=gap,height=height,side=side,upper_gap=minimum_gap,self_mm=selfmm,dynamic=[dict(force=t['force_N'],safety=t.get('result',{}).get('safety_passed'),yaw=t.get('result',{}).get('final_yaw_from_placement_deg'),thumb_N=t.get('result',{}).get('final_contact_N',{}).get('th_tip',0)) for t in row['dynamic']])),flush=True)
if __name__=='__main__':main()
