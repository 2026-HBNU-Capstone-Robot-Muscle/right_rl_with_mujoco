"""Search small right-hand goal changes around the supported83deg grip.

Compiled rollouts generate proposals; they do not bypass final1ms checks.
The cube starts from a saved free hold and is never pinned or externally
forced. The nominal terminal center may be selected by this design search;
it must then stay fixed for independent replays and later robustness tests.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from mujoco import rollout
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize,Bounds,LinearConstraint
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--generations',type=int,default=12);ap.add_argument('--population',type=int,default=18);ap.add_argument('--active-count',type=int,choices=(8,12),default=8);ap.add_argument('--resume-best',action='store_true');ap.add_argument('--resume-mf-best',action='store_true');ap.add_argument('--local-steps',type=int,default=0);ap.add_argument('--fixed-forward',type=int,choices=(0,64),default=0);ap.add_argument('--stage',choices=('first','second'),default='first');a=ap.parse_args()
    if not 0<=a.generations<=20 or not 6<=a.population<=24 or not 0<=a.local_steps<=12 or a.generations+a.local_steps==0:raise ValueError('bounded offline search')
    source=ROOT/'temporary_handoff_load/torque+0.npz';model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb'
    if a.stage=='second':
        parent=json.loads((ROOT/'supported_handoff_first_full/review.json').read_bytes());assert parent['dynamic']['safety_passed'] and parent['source_model_sha256']==sha(model);source=ROOT/'supported_handoff_first_full/end_state.npz'
    z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);initial=np.zeros(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_FULLPHYSICS));mujoco.mj_getState(m,d,initial,mujoco.mjtState.mjSTATE_FULLPHYSICS);warm=z['qacc_warmstart'];start=motor(z['ctrl']);lo,hi,nlo,nhi=native(m);planlo=np.ceil(lo+4).astype(int);planhi=np.floor(hi-4).astype(int);planlo=np.minimum(planlo,start);planhi=np.maximum(planhi,start);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;hand,support,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id;body=m.body('cube').id;C0=z['qpos'][-7:-4];up=z['up'];targetB=upright_yaw_target(z['initial_cube_rotation'],up,90);data=mujoco.MjData(m);inspect=mujoco.MjData(m);rng=np.random.default_rng(20261106+a.active_count+(100 if a.stage=='second' else 0));out=ROOT/('supported_handoff_search' if a.active_count==8 else 'supported_handoff_search_mf_active');records=[];cache={};best=None;best_rows=None;mean=np.zeros(a.active_count);cov=np.eye(a.active_count)*32**2;began=time.perf_counter();verified=[]
    if a.local_steps:out=ROOT/'supported_handoff_search_mf_local'
    if a.stage=='second':out=out.with_name(out.name+'_second')
    out.mkdir(exist_ok=True)
    resume_source=None
    if a.resume_best or a.resume_mf_best:
        resume_source=ROOT/(('supported_handoff_search_mf_active' if a.resume_mf_best else 'supported_handoff_search')+('_second' if a.stage=='second' else ''))/'review.json';previous=json.loads(resume_source.read_bytes());assert previous['source_state_sha256']==sha(source) and previous['source_model_sha256']==sha(model);prior_vector=previous['best_proposal']['vector_ticks'];mean[:len(prior_vector)]=prior_vector;cov[:8,:8]=np.eye(8)*12**2
    def make_rows(goal,hold):
        if not a.fixed_forward:return ramp(start,goal,hold)
        if max(abs(goal-start))>2*a.fixed_forward:raise ValueError('fixed-duration proposal exceeds2ticks per goal')
        forward=np.rint(np.linspace(start,goal,a.fixed_forward+1)[1:]).astype(int);rows=np.vstack([forward,np.tile(goal,(hold,1))]);assert max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())<=2;return rows
    def evaluate(vector):
        goal=start.copy();goal[:a.active_count]=np.clip(np.rint(start[:a.active_count]+vector),planlo[:a.active_count],planhi[:a.active_count]);delta=goal[:a.active_count]-start[:a.active_count];rows=make_rows(goal,54);key=hashlib.sha256(np.asarray(rows,dtype=np.int64).tobytes()).hexdigest()
        if key in cache:return cache[key]
        controls=np.repeat((rows[:,ids]-2048)*TICK,75,axis=0);states,_=rollout.rollout(m,data,initial,controls,initial_warmstart=warm);states=states[0];qpos=states[:,1:1+m.nq];qvel=states[:,1+m.nq:1+m.nq+m.nv];finite=bool(np.isfinite(states).all());record=dict(index=len(records),vector_ticks=delta.tolist(),goal_array_sha256=key,proposal_goal_count=len(rows),score=1e9,compiled_rollout_only=True)
        if finite:
            range_violation=float(max(0,np.max(nlo-qpos[:,:16]),np.max(qpos[:,:16]-nhi)));tracking=float(np.max(abs(qpos[:,:16]-controls)));translation=float(np.max(np.linalg.norm(qpos[:,-7:-4]-C0,axis=1))*1000);selfmm=supportmm=cubemm=0.;external=False
            for step in np.unique(np.r_[np.arange(0,len(states),375),len(states)-1]):
                inspect.qpos[:]=qpos[step];inspect.qvel[:]=qvel[step];inspect.ctrl[:]=controls[step];mujoco.mj_forward(m,inspect);su,sp=check(inspect);selfmm=max(selfmm,max(sp.values(),default=0.));supportmm=max(supportmm,max(su.values(),default=0.))
                for c in inspect.contact:
                    x,y=int(c.geom1),int(c.geom2)
                    if cube not in (x,y):continue
                    other=y if x==cube else x
                    if int(m.geom_bodyid[other]) in hand:cubemm=max(cubemm,max(0.,-float(c.dist)*1000))
                    elif other in support and c.dist<.0004:external=True
            C=qpos[-1,-7:-4].copy();B=Rotation.from_quat(np.r_[qpos[-1,-3:],qpos[-1,-4]]).as_matrix();pose=orientation_error_deg(B,targetB);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))));yaw=angle(B,z['initial_cube_rotation'],up);hold_start=len(states)-54*75;holdC=qpos[hold_start,-7:-4];holdB=Rotation.from_quat(np.r_[qpos[hold_start,-3:],qpos[hold_start,-4]]).as_matrix();drift=orientation_error_deg(B,holdB);shift=float(np.linalg.norm(C-holdC)*1000);gaps_tail=[];pressure_good=[]
            for step in np.linspace(max(hold_start,len(states)-2250),len(states)-1,12,dtype=int):
                inspect.qpos[:]=qpos[step];inspect.qvel[:]=qvel[step];inspect.ctrl[:]=controls[step];mujoco.mj_forward(m,inspect);Ci=inspect.geom_xpos[cube];Bi=inspect.geom_xmat[cube].reshape(3,3);pressures={'if_tip':0.,'th_tip':0.}
                for ci,c in enumerate(inspect.contact):
                    x,y=int(c.geom1),int(c.geom2)
                    if cube not in (x,y):continue
                    other=y if x==cube else x;name=m.geom(other).name
                    if name in pressures and -float((c.pos-Ci)@Bi[:,1])*1000<55/6:
                        ff=np.zeros(6);mujoco.mj_contactForce(m,inspect,ci,ff);pressures[name]+=float(ff[0])
                pressure_good.append(all(v>.2 for v in pressures.values()))
                if step in (max(hold_start,len(states)-2250),len(states)-1):gaps_tail.append(min(v for name,v in gaps(m,inspect,Ci,Bi,geometry,True)))
            minimum_gap=min(gaps_tail);fraction=float(np.mean(pressure_good));safety_penalty=max(0,range_violation-1e-6)*1e7+max(0,tracking-.18)*1e5+max(0,selfmm-1e-6)*1e5+max(0,supportmm-.15)*1000+max(0,cubemm-1)*1000+(10000 if external else 0)+max(0,translation-80)*1000
            score=pose*5+tilt+max(0,1.2-minimum_gap)*25+max(0,.85-fraction)*60+.2*np.linalg.norm(C-C0)*1000+max(0,drift-1)*15+max(0,shift-1)*15+safety_penalty
            record.update(score=float(score),yaw_deg=yaw,tilt_deg=tilt,orientation_error_deg=pose,rotation_error_vector_deg=np.degrees(Rotation.from_matrix(targetB@B.T).as_rotvec()).tolist(),nominal_terminal_center_world_m=C.tolist(),center_shift_from_source_mm=float(np.linalg.norm(C-C0)*1000),minimum_upper_gap_sampled_mm=minimum_gap,lower_contact_fraction_sampled=fraction,fixed_hold_orientation_drift_deg=drift,fixed_hold_shift_mm=shift,range_rad_all1ms=range_violation,tracking_rad_all1ms=tracking,self_mm_sampled=selfmm,support_mm_sampled=supportmm,cube_mm_sampled=cubemm,external_cube_support_sampled=external,coarse_guard_penalty=float(safety_penalty))
        records.append(record);cache[key]=record;return record
    baseline=evaluate(mean);best=baseline
    for generation in range(a.generations):
        proposals=[mean.copy(),np.asarray(best['vector_ticks'],float)]
        proposals.extend(rng.multivariate_normal(mean,cov,size=a.population-2));generation_results=[evaluate(np.clip(v,-96,96)) for v in proposals];elite=sorted(generation_results,key=lambda r:r['score'])[:max(4,a.population//4)];weights=np.linspace(1.5,.5,len(elite));weights/=sum(weights);vectors=np.asarray([r['vector_ticks'] for r in elite],float);newmean=np.average(vectors,axis=0,weights=weights);centered=vectors-newmean;cov=.35*cov+.65*((centered*weights[:,None]).T@centered)+np.eye(a.active_count)*2**2;mean=newmean
        if elite[0]['score']<best['score']:best=elite[0]
        goal=start.copy();goal[:a.active_count]+=np.asarray(best['vector_ticks']);best_rows=make_rows(goal,134);np.savez_compressed(out/'best_candidate.npz',start=start,goals=best_rows)
        if best.get('coarse_guard_penalty',1)>0:eligible=False
        else:eligible=best.get('orientation_error_deg',100)<5 and best.get('tilt_deg',100)<5 and best.get('minimum_upper_gap_sampled_mm',-100)>.3 and best.get('lower_contact_fraction_sampled',0)>.8 and best.get('fixed_hold_shift_mm',100)<2
        if eligible and not any(v['goal_hash']==best['goal_array_sha256'] for v in verified):
            folder=out/f'verified_generation{generation}';folder.mkdir(exist_ok=True);np.savez_compressed(folder/'candidate.npz',start=start,goals=best_rows);v=run(m,z,best_rows,1,folder/'end_state.npz',folder/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(folder/'end_state.npz'));why=quarter_reasons(v,best['nominal_terminal_center_world_m'],True,orientation_error_deg(end.xmat[body].reshape(3,3),targetB));verified.append(dict(goal_hash=best['goal_array_sha256'],dynamic=v,reasons=why,checkpoint90_and_grip_passed=not why));print(json.dumps(dict(event='full1ms_replay',generation=generation,passed=not why,reasons=why)),flush=True)
        report=dict(status='compiled_rollout_supported_handoff_goal_search',parameters=vars(a),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),resume_source_sha256=None if resume_source is None else sha(resume_source),generation_completed=generation+1,unique_rollouts=len(records),best_proposal=best,all_proposals=records,full1ms_replays=verified,common_center_selected_from_nominal_design_not_robustness_proof=True,compiled_proposal_checks_not1ms_contact_certificates=True,full_initial_placement_not_replayed=True,no_cube_pose_reset_or_external_force=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(generation=generation+1,rollouts=len(records),score=best['score'],yaw=best.get('yaw_deg'),tilt=best.get('tilt_deg'),gap=best.get('minimum_upper_gap_sampled_mm'),pressure_fraction=best.get('lower_contact_fraction_sampled'),guard_penalty=best.get('coarse_guard_penalty'),seconds=time.perf_counter()-began)),flush=True)
        if verified and verified[-1]['checkpoint90_and_grip_passed']:break
    for iteration in range(a.local_steps):
        p=np.asarray(best['vector_ticks'],float);r=np.asarray(best['rotation_error_vector_deg']);J=np.zeros((3,a.active_count));G=np.zeros(a.active_count)
        for j in range(a.active_count):
            va=p.copy();vb=p.copy();va[j]=min(96,va[j]+4);vb[j]=max(-96,vb[j]-4);plus=evaluate(va);minus=evaluate(vb);den=plus['vector_ticks'][j]-minus['vector_ticks'][j]
            if den and 'rotation_error_vector_deg' in plus and 'rotation_error_vector_deg' in minus:
                J[:,j]=(np.asarray(plus['rotation_error_vector_deg'])-minus['rotation_error_vector_deg'])/den;G[j]=(plus['minimum_upper_gap_sampled_mm']-minus['minimum_upper_gap_sampled_mm'])/den
        H=25*J.T@J+np.eye(a.active_count)*.03;v=25*J.T@r;lower=np.maximum(-24,np.maximum(-96,planlo[:a.active_count]-start[:a.active_count])-p);upper=np.minimum(24,np.minimum(96,planhi[:a.active_count]-start[:a.active_count])-p);fit=minimize(lambda delta:float(.5*delta@H@delta+v@delta),np.zeros(a.active_count),jac=lambda delta:H@delta+v,method='SLSQP',bounds=Bounds(lower,upper),constraints=[LinearConstraint(G[None,:],[.8-best['minimum_upper_gap_sampled_mm']],[np.inf])],options=dict(maxiter=100,ftol=1e-10));previous_score=best['score'];options=[evaluate(np.clip(p+fit.x*scale,-96,96)) for scale in (1.,.5,.25,.125)];trial=min(options,key=lambda row:row['score'])
        if trial['score']<best['score']:best=trial
        goal=start.copy();goal[:a.active_count]+=np.asarray(best['vector_ticks']);best_rows=make_rows(goal,134);np.savez_compressed(out/'best_candidate.npz',start=start,goals=best_rows);eligible=best.get('coarse_guard_penalty',1)==0 and best.get('orientation_error_deg',100)<5 and best.get('tilt_deg',100)<5 and best.get('minimum_upper_gap_sampled_mm',-100)>.3 and best.get('lower_contact_fraction_sampled',0)>.8 and best.get('fixed_hold_shift_mm',100)<2
        if eligible and not any(v['goal_hash']==best['goal_array_sha256'] for v in verified):
            folder=out/f'verified_local{iteration}';folder.mkdir(exist_ok=True);np.savez_compressed(folder/'candidate.npz',start=start,goals=best_rows);check_result=run(m,z,best_rows,1,folder/'end_state.npz',folder/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(folder/'end_state.npz'));why=quarter_reasons(check_result,best['nominal_terminal_center_world_m'],True,orientation_error_deg(end.xmat[body].reshape(3,3),targetB));verified.append(dict(goal_hash=best['goal_array_sha256'],dynamic=check_result,reasons=why,checkpoint90_and_grip_passed=not why));print(json.dumps(dict(event='full1ms_replay',local=iteration+1,passed=not why,reasons=why)),flush=True)
        report=dict(status='compiled_rollout_supported_handoff_local_refinement',parameters=vars(a),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),resume_source_sha256=None if resume_source is None else sha(resume_source),local_iteration_completed=iteration+1,unique_rollouts=len(records),best_proposal=best,all_proposals=records,full1ms_replays=verified,common_center_selected_from_nominal_design_not_robustness_proof=True,compiled_proposal_checks_not1ms_contact_certificates=True,full_initial_placement_not_replayed=True,no_cube_pose_reset_or_external_force=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(local=iteration+1,rollouts=len(records),score=best['score'],yaw=best.get('yaw_deg'),tilt=best.get('tilt_deg'),gap=best.get('minimum_upper_gap_sampled_mm'),pressure_fraction=best.get('lower_contact_fraction_sampled'),guard_penalty=best.get('coarse_guard_penalty'),improved=best['score']<previous_score,seconds=time.perf_counter()-began)),flush=True)
        if verified and verified[-1]['checkpoint90_and_grip_passed']:break
        if best['score']>=previous_score:break
if __name__=='__main__':main()
