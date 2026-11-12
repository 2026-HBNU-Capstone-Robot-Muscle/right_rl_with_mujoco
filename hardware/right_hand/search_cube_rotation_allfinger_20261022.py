"""Offline bounded rotation search from the completed grasp observation.

CEM trajectory search, NOT SAC and NOT a deployable motor program. Uses the
unchanged native hand/support contacts and provisional seven cube resets.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim,sim_to_ticks

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261022'
SOURCE=ROOT/'cube_grasp_trial_20261022_141534_666300/result.json'
RESETS=ROOT/'prepared_cube_review_20261022/loaded_approach_sensitivity.json'
VARIABLES=np.arange(16)  # Ring tip/PIP/root may extend; no deeper palm curl.


def write(path,data):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,indent=2),encoding='utf-8');temp.replace(path)


def rotation_components(initial,current,axis):
    # MuJoCo wxyz -> scipy xyzw, then world-frame relative rotation.
    a=Rotation.from_quat(np.asarray(initial)[[1,2,3,0]])
    b=Rotation.from_quat(np.asarray(current)[[1,2,3,0]])
    v=(b*a.inv()).as_rotvec()*180/np.pi
    yaw=float(v@axis)
    return yaw,float(np.linalg.norm(v-yaw*axis))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iterations',type=int,default=5)
    parser.add_argument('--population',type=int,default=36);parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--resume');parser.add_argument('--all-resets',action='store_true')
    parser.add_argument('--endpoint-limit-ticks',type=int,default=192)
    parser.add_argument('--target-deg',type=float,default=-10.)
    parser.add_argument('--name',default='small_yaw_search');args=parser.parse_args()
    OUT.mkdir(exist_ok=True);began=time.monotonic()
    if not 96<=args.endpoint_limit_ticks<=256:raise ValueError('Offline endpoint bound')
    amplitude=args.endpoint_limit_ticks;first_limit=int(amplitude*2/3);leg_steps=int(np.ceil(amplitude/2))
    source=json.loads(SOURCE.read_bytes());base=np.array(source['final_on_state']['goal'])
    actual=np.array(source['final_on_state']['position'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);physics=set_cube_physics(m,side_m=.055,mass_kg=.100)
    lower=np.maximum(m.jnt_range[m.actuator_trnid[:,0],0],m.actuator_ctrlrange[:,0])
    upper=np.minimum(m.jnt_range[m.actuator_trnid[:,0],1],m.actuator_ctrlrange[:,1])
    low_raw=sim_to_ticks(lower);high_raw=sim_to_ticks(upper)
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')};cube=m.geom('cube').id
    up=-np.array(m.opt.gravity);up/=np.linalg.norm(up)
    seeds=[r for r in json.loads(RESETS.read_bytes())['trials'] if r['gain_scale']==1.]
    originals=(m.jnt_range.copy(),m.actuator_ctrlrange.copy(),m.geom_friction.copy())

    def contacts(d):
        self_depth=support_depth=cube_depth=0.;touched=set()
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000)
            if cube in (a,b):
                cube_depth=max(cube_depth,depth);touched.add(m.geom(b if a==cube else a).name)
            elif a in support or b in support:support_depth=max(support_depth,depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:self_depth=max(self_depth,depth)
        return self_depth,support_depth,cube_depth,sorted(touched)

    starts=[]
    for seed in seeds:
        d=mujoco.MjData(m);d.qpos[:]=seed['trace'][-1]['qpos'];d.ctrl[:]=ticks_to_sim(base)
        d.qpos[:16]=ticks_to_sim(actual)
        mujoco.mj_forward(m,d)
        for _ in range(round(.5/m.opt.timestep)):mujoco.mj_step(m,d)
        distance={name:float(mujoco.mj_geomDistance(m,d,cube,m.geom(name).id,1.,np.zeros(6)))*1000
                  for name in ('if_tip','mf_tip','rf_tip','th_tip')}
        starts.append(dict(qpos=d.qpos.copy(),qvel=d.qvel.copy(),
            reset=seed['reset'],offset=seed['offset_palm_mm'],distances=distance))
    print('STARTS',json.dumps([{k:(v.tolist() if isinstance(v,np.ndarray) else v)
          for k,v in s.items() if k not in ('qpos','qvel')} for s in starts]),flush=True)

    def targets(delta):
        delta=np.rint(np.asarray(delta).reshape(2,len(VARIABLES))).astype(int)
        delta[0]=np.clip(delta[0],-first_limit,first_limit)
        delta[1]=np.clip(delta[1],-amplitude,amplitude)
        delta[1]=np.minimum(np.maximum(delta[1],delta[0]-amplitude),delta[0]+amplitude)
        ends=np.tile(base,(2,1));ends[:,VARIABLES]+=delta
        ends=np.minimum(np.maximum(ends,low_raw),high_raw)
        # Roots already outside the measured support never bend backward.
        ends[:,[7,11]]=np.maximum(ends[:,[7,11]],base[[7,11]])
        ends[:,[12,14,15]]=np.minimum(ends[:,[12,14,15]],base[[12,14,15]])
        return ends

    def path(delta):
        rows=[];last=base.copy()
        for end in targets(delta):
            rows.extend(np.rint(last+(end-last)*k/leg_steps).astype(int) for k in range(1,leg_steps+1));last=end
        return np.asarray(rows)

    def evaluate(delta,index=0,record=False):
        start=starts[index];d=mujoco.MjData(m);d.qpos[:]=start['qpos'];d.qvel[:]=start['qvel']
        d.ctrl[:]=ticks_to_sim(base);mujoco.mj_forward(m,d)
        origin=d.qpos[-7:].copy();goals=path(delta)
        self_peak=support_peak=cube_peak=tracking=max_shift=drop=0.;fault=None;trace=[];touched=[]
        stable=longest=0
        for k,q in enumerate(list(goals)+[goals[-1]]*20):
            goal=ticks_to_sim(q);d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                sd,wd,cd,touched=contacts(d)
                self_peak=max(self_peak,sd);support_peak=max(support_peak,wd);cube_peak=max(cube_peak,cd)
                tracking=max(tracking,float(max(abs(d.qpos[:16]-goal))))
            shift=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3])*1000);max_shift=max(max_shift,shift)
            drop=max(drop,float((origin[:3]-d.qpos[-7:-4])@up)*1000)
            if not np.isfinite(d.qpos).all():fault='nonfinite'
            elif any(n in touched for n in ('support_glass','support_wood')):fault='cube_external_support'
            elif self_peak>1e-6:fault='new_self_contact'
            elif support_peak>.15:fault='support_contact'
            elif tracking>.12:fault='tracking'
            elif max_shift>20 or drop>8:fault='cube_left_start_region'
            elif cube_peak>1.5:fault='deep_cube_contact'
            yaw_now,off_now=rotation_components(origin[3:],d.qpos[-4:],up)
            active_now=any(name in touched for name in ('if_tip','mf_tip','rf_tip','th_tip'))
            qualifies=(not fault and abs(yaw_now-args.target_deg)<=4 and off_now<=8
                       and max_shift<=12 and drop<=4 and active_now)
            stable=stable+1 if k>=len(goals) and qualifies else 0
            longest=max(longest,stable)
            if record and (k%2==0 or k==len(goals)+19):trace.append(dict(time_s=(k+1)*.05,qpos=d.qpos.tolist(),goal_ticks=q.tolist()))
            if fault:break
        yaw,off=rotation_components(origin[3:],d.qpos[-4:],up)
        active=any(name in touched for name in ('if_tip','mf_tip','rf_tip','th_tip'))
        success=bool(not fault and abs(yaw-args.target_deg)<=4 and off<=8 and max_shift<=12
                     and drop<=4 and active and longest>=10)
        cost=abs(yaw-args.target_deg)/10+.3*off/10+.15*max_shift/12+(10 if fault else 0)+(0 if active else .15)
        result=dict(cost=float(cost),reset=index,yaw_deg=yaw,off_axis_deg=off,max_cube_shift_mm=max_shift,
            downward_shift_mm=drop,max_self_mm=self_peak,max_support_mm=support_peak,max_cube_mm=cube_peak,
            max_tracking_rad=tracking,final_cube_contacts=touched,success=success,fault=fault,
            longest_qualified_hold_seconds=longest*.05,
            delta=(targets(delta)[:,VARIABLES]-base[VARIABLES]).ravel().tolist(),endpoint_ticks=targets(delta).tolist())
        if record:result['trace']=trace;result['initial_qpos']=start['qpos'].tolist()
        return result

    validation_indices=list(range(len(starts))) if args.all_resets else [0,2,4]
    def score(delta):
        results=[evaluate(delta,i) for i in validation_indices]
        worst=dict(max(results,key=lambda r:r['cost']))
        worst['cost']=max(r['cost'] for r in results)+.2*np.mean([r['cost'] for r in results])
        worst['success']=all(r['success'] for r in results);worst['trials']=results
        return worst

    zero=np.zeros((2,len(VARIABLES)))
    hold=[evaluate(zero,i) for i in range(len(starts))]
    print('ZERO BASELINE',json.dumps([{k:v for k,v in r.items() if k not in ('delta','endpoint_ticks')} for r in hold]),flush=True)
    rng=np.random.default_rng(20261022);mean=zero.ravel();std=np.full_like(mean,32.);best=None;history=[]
    if args.resume:
        prior=json.loads((OUT/args.resume).read_bytes())
        if prior['source_sha256']!=hashlib.sha256(SOURCE.read_bytes()).hexdigest():
            raise RuntimeError('Resume source differs from the completed grasp')
        prior_delta=np.asarray(prior['validation'][0]['delta']).reshape(2,-1)
        padded=np.zeros((2,len(VARIABLES)));padded[:,:prior_delta.shape[1]]=prior_delta
        mean=padded.ravel();std[:]=28.
        best=score(mean)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for iteration in range(args.iterations):
            population=rng.normal(mean,std,(args.population,len(mean)));population[0]=mean
            if best is not None:population[1]=np.asarray(best['delta']).ravel()
            if best is not None and iteration==0:
                for index,scale in enumerate((1.25,1.5,1.75,2.,2.25),2):
                    if index<len(population):population[index]=np.asarray(best['delta']).ravel()*scale
            if best is not None and iteration==0:
                for index,travel in enumerate((64,128,192,256),7):
                    if index>=len(population):break
                    trial=np.asarray(best['delta']).reshape(2,16).copy()
                    trial[:,[12,14,15]]-=np.array([[travel//2],[travel]])
                    population[index]=trial.ravel()
            ranked=sorted(pool.map(score,population),key=lambda r:r['cost'])
            elite=np.asarray([r['delta'] for r in ranked[:max(4,args.population//6)]])
            mean=.25*mean+.75*elite.mean(axis=0).ravel()
            std=np.maximum(8.,.25*std+.75*elite.std(axis=0).ravel())
            if best is None or ranked[0]['cost']<best['cost']:best=ranked[0]
            row=dict(iteration=iteration+1,evaluations=(iteration+1)*args.population,
                best={k:v for k,v in best.items() if k not in ('delta','trials','endpoint_ticks')})
            history.append(row);print(json.dumps(row),flush=True)
            write(OUT/(args.name+'_progress.json'),dict(status='running',algorithm='CEM_not_SAC',history=history))
            if best['success']:break
    validation=[evaluate(best['delta'],i,record=True) for i in range(len(starts))]
    assert all(np.array_equal(a,b) for a,b in zip(originals,(m.jnt_range,m.actuator_ctrlrange,m.geom_friction)))
    result=dict(status='offline_small_rotation_search_complete_NOT_SAC',elapsed_seconds=time.monotonic()-began,
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        cube_resets_sha256=hashlib.sha256(RESETS.read_bytes()).hexdigest(),
        model_manifest=manifest,cube_physics=physics,gravity_up_axis=up.tolist(),
        actual_grasp_ticks=actual.tolist(),base_goal_ticks=base.tolist(),
        goal_period_s=.05,motion_samples=len(path(best['delta'])),hold_samples=20,
        endpoint_limit_ticks=amplitude,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        requested_signed_yaw_deg=args.target_deg,native_ranges_unchanged=True,friction_unchanged=True,
        cube_poses_provisional=True,real_cube_registration_verified=False,
        algorithm='CEM_not_SAC',live_sac=False,hardware_execution_allowed=False,hardware_commands_sent=0,
        history=history,baseline=hold,validation=validation,
        successful_cases=sum(r['success'] for r in validation),total_cases=len(validation),
        candidate_goal_ticks=path(best['delta']).tolist(),
        caveat='No hardware policy exported. Synthetic cube starts and native actuation remain approximations; finger motion alone is not cube rotation success.')
    write(OUT/(args.name+'_result.json'),result)
    print('RESULT',json.dumps({k:v for k,v in result.items() if k in ('status','elapsed_seconds','successful_cases','total_cases')}),flush=True)


if __name__=='__main__':main()
