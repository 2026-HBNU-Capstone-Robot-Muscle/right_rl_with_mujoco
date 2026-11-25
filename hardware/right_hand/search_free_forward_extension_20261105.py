"""Bounded stochastic search for additional yaw in a genuinely free cube.

Existing geometry, forces, current assumptions, joint bounds and1ms dynamic
guards are retained. No pinned-cube path counts as success. This is a nominal
checkpoint search, not a complete placement matrix or a physical command.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,CP,ramp
from research_rotation_body_hold_20261104 import evaluate,restore,motor,native

ACTIVE=np.array([1,2,3,4,9,10,11,12])-1

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seed',type=int,choices=(11,23,37),required=True);ap.add_argument('--generations',type=int,default=3);ap.add_argument('--batch',type=int,default=18);a=ap.parse_args()
    if not 1<=a.generations<=4 or not 8<=a.batch<=24:raise ValueError('bounded offline search workload')
    out=ROOT/f'free_extension_search/seed{a.seed}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');start=motor(z['ctrl']);lo,hi,_,_=native(m)
    low=np.maximum(np.ceil(lo[ACTIVE]).astype(int)+4-start[ACTIVE],-192);high=np.minimum(np.floor(hi[ACTIVE]).astype(int)-4-start[ACTIVE],192)
    if np.any(low>=high):raise ValueError('no bounded native interval')
    rng=np.random.default_rng(a.seed);old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');turn=old['goals'][old['phases']=='stage_7_horizontal_yaw'];trend=turn[-1,ACTIVE]-turn[max(0,len(turn)-129),ACTIVE];cases=[];best=None;elite=[];mean=np.zeros(len(ACTIVE));sigma=np.full(len(ACTIVE),64.)
    source_sha=hashlib.sha256((CP/'end_state.npz').read_bytes()).hexdigest()
    def trial(delta,origin,generation):
        nonlocal best
        goal=start.copy();goal[ACTIVE]+=np.asarray(delta,int)
        if np.any(goal<lo) or np.any(goal>hi):return None
        rows=ramp(start,goal,67);began=time.perf_counter();temp=out/'latest_trial_end.npz';d=evaluate(m,z,rows,temp);fd=restore(m,np.load(temp));B=fd.xmat[m.body('cube').id].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))));yaw=float(d['whole_yaw_deg']);error=abs((yaw-90+180)%360-180)
        score=error+3*max(0,tilt-5)+2*max(0,d['hold_drift_deg']-3)+2*max(0,d['hold_shift_mm']-2)
        if not d['safety_passed']:score=1000+100*(1-d['samples_completed']/len(rows))
        passed=bool(d['safety_passed'] and error<5 and tilt<5 and d['hold_evaluated_steps']==4050 and d['hold_drift_deg']<3 and d['hold_shift_mm']<2)
        row=dict(index=len(cases),generation=generation,origin=origin,deltas_by_motor={str(mid+1):int(value) for mid,value in zip(ACTIVE,delta)},goal=goal.tolist(),score=score,whole_yaw_deg=yaw,final_up_tilt_deg=tilt,model_target90_direction_filter_passed=passed,elapsed_seconds=time.perf_counter()-began,dynamic=d);cases.append(row)
        if best is None or score<best['score']:
            best=row
            (out/'best_end_state.npz').write_bytes(temp.read_bytes());np.savez_compressed(out/'best_suffix.npz',start=start,goals=rows)
            np.savez_compressed(out/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['experimental_yaw_extension']*(len(rows)-67),['experimental_yaw_hold']*67])
        report=dict(status='nominal_free_cube_additional_yaw_search',seed=a.seed,source_checkpoint=str((CP/'end_state.npz').relative_to(R)),source_checkpoint_sha256=source_sha,active_motor_ids=(ACTIVE+1).tolist(),maximum_sampled_delta_ticks=192,goal_increment_limit_ticks=2,task_target_whole_yaw_deg=90,cases=cases,best_index=best['index'],best_score=best['score'],best_model_target90_passed=best['model_target90_direction_filter_passed'],full_initial_placement_not_replayed=True,upper_layer_clearance_and_regrasp_not_yet_evaluated=True,geometry_native_and_all_dynamic_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        return row
    trial(np.zeros(len(ACTIVE),int),'baseline',-1)
    for fraction in (.25,.5,1.):
        delta=np.rint(trend*fraction).astype(int)
        if np.all(delta>=low) and np.all(delta<=high):trial(delta,'recorded_tangent_extrapolation',-1)
    for generation in range(a.generations):
        for _ in range(a.batch):
            delta=[]
            for j in range(len(ACTIVE)):
                value=None
                for attempt in range(80):
                    draw=int(np.rint(rng.normal(mean[j],sigma[j])))
                    if low[j]<=draw<=high[j]:value=draw;break
                if value is None:value=int(rng.integers(low[j],high[j]+1))
                delta.append(value)
            trial(np.array(delta),'truncated_normal',generation)
        safe=[c for c in cases if c['dynamic']['safety_passed']];elite=sorted(safe,key=lambda c:c['score'])[:max(3,a.batch//4)]
        if elite:
            values=np.array([[c['deltas_by_motor'][str(mid+1)] for mid in ACTIVE] for c in elite]);mean=.5*mean+.5*values.mean(axis=0);sigma=np.maximum(16.,.5*sigma+.5*values.std(axis=0))
        print(json.dumps(dict(seed=a.seed,generation=generation,cases=len(cases),safe=len(safe),best_score=best['score'],best_yaw=best['whole_yaw_deg'],best_tilt=best['final_up_tilt_deg'],best_target90_passed=best['model_target90_direction_filter_passed'],best_delta=best['deltas_by_motor'])),flush=True)
if __name__=='__main__':main()
