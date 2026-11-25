"""Offline yaw search allowing all four joints of the supporting index.

Earlier searches moved eight thumb/middle joints or only the index root.
This nominal checkpoint comparison explores a different contact strategy.
Every trial restores the same saved free cube once, then runs unmodified
physics and guards. No endpoint is an executable hardware policy.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R,CP,ramp
from research_rotation_body_hold_20261104 import evaluate,restore,motor,native
from cube_task_frame_20261105 import orientation_error_deg

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--family',choices=('index','all'),required=True);ap.add_argument('--seed',type=int,choices=(31,47),default=31);ap.add_argument('--generations',type=int,default=3);ap.add_argument('--batch',type=int,default=16);a=ap.parse_args()
    if not 1<=a.generations<=4 or not 8<=a.batch<=20:raise ValueError('bounded research workload')
    active=np.arange(4,8) if a.family=='index' else np.arange(12)
    out=ROOT/f'index_assisted_yaw/{a.family}_seed{a.seed}';out.mkdir(parents=True,exist_ok=True)
    source=CP/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(source);start=motor(z['ctrl']);lo,hi,_,_=native(m)
    low=np.maximum(np.ceil(lo[active]).astype(int)-start[active],-192);high=np.minimum(np.floor(hi[active]).astype(int)-start[active],192)
    rng=np.random.default_rng(a.seed);prefix=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');cases=[];best=None;mean=np.zeros(len(active));sigma=np.full(len(active),64.);bindings={}
    target=Rotation.from_rotvec(z['up']*np.pi/2).as_matrix()@z['initial_cube_rotation']
    def trial(delta,origin,generation):
        nonlocal best
        delta=np.asarray(delta,int);goal=start.copy();goal[active]+=delta
        if np.any(goal<lo) or np.any(goal>hi):return None
        rows=ramp(start,goal,67);began=time.perf_counter();temporary=out/'last_trial_end.npz';dynamic=evaluate(m,z,rows,temporary);d=restore(m,np.load(temporary));B=d.xmat[m.body('cube').id].reshape(3,3)
        tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))));orientation_error=orientation_error_deg(B,target);shift=float(np.linalg.norm(d.qpos[-7:-4]-z['qpos'][-7:-4])*1000)
        score=orientation_error+3*max(0,tilt-5)+2*max(0,dynamic['hold_drift_deg']-3)+2*max(0,dynamic['hold_shift_mm']-2)+.2*max(0,shift-20)
        if not dynamic['safety_passed']:score=1000+100*(1-dynamic['samples_completed']/len(rows))
        passed=bool(dynamic['safety_passed'] and orientation_error<5 and tilt<5 and dynamic['hold_evaluated_steps']==4050 and dynamic['hold_drift_deg']<3 and dynamic['hold_shift_mm']<2)
        row=dict(index=len(cases),origin=origin,generation=generation,deltas_by_motor={str(mid+1):int(v) for mid,v in zip(active,delta)},goal=goal.tolist(),orientation_error_to_true90_deg=orientation_error,whole_yaw_deg=dynamic['whole_yaw_deg'],up_tilt_deg=tilt,center_shift_mm=shift,score=score,target90_direction_filter_passed=passed,dynamic=dynamic,elapsed_seconds=time.perf_counter()-began);cases.append(row)
        if best is None or score<best['score']:
            best=row;(out/'best_end_state.npz').write_bytes(temporary.read_bytes());np.savez_compressed(out/'best_suffix.npz',start=start,goals=rows)
            np.savez_compressed(out/'candidate.npz',start=prefix['start'],goals=np.vstack([prefix['goals'],rows]),phases=np.r_[prefix['phases'],['index_assisted_yaw']*(len(rows)-67),['index_assisted_yaw_hold']*67])
        report=dict(status='nominal_index_assisted_free_cube_yaw_search',family=a.family,seed=a.seed,active_motor_ids=(active+1).tolist(),source_checkpoint=str(source.relative_to(R)),source_checkpoint_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),prior_report_bindings=bindings.copy(),task_target_is_full_rotation90_not_yaw_label_only=True,cases=cases,best_index=best['index'],best_target90_passed=best['target90_direction_filter_passed'],best_yaw_deg=best['whole_yaw_deg'],best_orientation_error_deg=best['orientation_error_to_true90_deg'],native_goal_bounds_and_original_dynamic_guards_unchanged=True,goal_step_limit_ticks=2,maximum_endpoint_change_ticks=192,current_PID_and_geometry_not_modified=True,cube_pose_never_reset_after_trial_start=True,full_initial_placement_not_replayed=True,common_grip_return_and_upper_layer_clearance_not_yet_validated=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');return row
    trial(np.zeros(len(active),int),'same_state_baseline',-1)
    if a.family=='index':
        for mid in range(4):
            for amount in (-96,96):
                delta=np.zeros(4,int);delta[mid]=amount;trial(delta,'index_joint_sensitivity',-1)
    else:
        for seed in (23,37):
            p=ROOT/f'free_extension_search/seed{seed}/review.json';raw=json.loads(p.read_bytes());bindings[str(p.relative_to(R))]=hashlib.sha256(p.read_bytes()).hexdigest();row=raw['cases'][raw['best_index']];delta=np.zeros(12,int)
            for mid,v in row['deltas_by_motor'].items():delta[int(mid)-1]=v
            trial(delta,'previous_eight_joint_best',-1)
        safe=sorted((c for c in cases if c['dynamic']['safety_passed']),key=lambda c:c['score'])
        if safe:mean=np.array([safe[0]['deltas_by_motor'][str(mid+1)] for mid in active],float)
        sigma[:4]=40.;sigma[8:]=40.;sigma[4:8]=96.
    for generation in range(a.generations):
        for member in range(a.batch):
            delta=[]
            for j in range(len(active)):
                options=rng.normal(mean[j],sigma[j],80);valid=np.rint(options).astype(int);valid=valid[(valid>=low[j])&(valid<=high[j])];delta.append(int(valid[0]) if len(valid) else int(rng.integers(low[j],high[j]+1)))
            trial(delta,'truncated_normal',generation)
        elite=sorted((c for c in cases if c['dynamic']['safety_passed']),key=lambda c:c['score'])[:max(3,a.batch//4)]
        if elite:
            values=np.array([[c['deltas_by_motor'][str(mid+1)] for mid in active] for c in elite]);mean=.4*mean+.6*values.mean(axis=0);sigma=np.maximum(16.,.4*sigma+.6*values.std(axis=0))
        print(json.dumps(dict(family=a.family,generation=generation,cases=len(cases),safe=sum(c['dynamic']['safety_passed'] for c in cases),best_yaw=best['whole_yaw_deg'],best_orientation_error=best['orientation_error_to_true90_deg'],best_tilt=best['up_tilt_deg'],target90=best['target90_direction_filter_passed'],best_delta=best['deltas_by_motor'])),flush=True)
if __name__=='__main__':main()
