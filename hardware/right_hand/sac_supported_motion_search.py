"""CEM motion search to diagnose support-constrained reachability, NOT SAC.

No hardware imports. Even a successful candidate requires independent replay
and policy learning; this script never exports a motor policy.
"""
import json
import argparse
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_forward_geometry import measure,eligible
ROOT=Path(__file__).resolve().parent

def save(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iterations',type=int,default=6);parser.add_argument('--population',type=int,default=48)
    parser.add_argument('--waypoints',type=int,default=2);parser.add_argument('--resume');parser.add_argument('--name',default='sac_supported_motion_search')
    parser.add_argument('--control-steps',type=int,default=1);parser.add_argument('--robust',action='store_true')
    args=parser.parse_args()
    if args.waypoints<2 or args.population<8 or args.iterations<1:parser.error('Invalid search size')
    started=time.monotonic();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);model,manifest=build(env)
    report=json.loads((ROOT/'sac_supported_reset_result.json').read_text());base=np.asarray(report['candidates'][0]['ctrl'])
    lower=np.array(env._safe_lowers);lower[[0,4,8]]=np.maximum(lower[[0,4,8]],.65);upper=np.array(env._safe_uppers)
    support={model.geom(n).id for n in ('support_glass','support_wood')};dt=float(model.opt.timestep)
    def evaluate(delta,offset=0,record=False):
        d=mujoco.MjData(model);initial=np.asarray(report['candidates'][offset]['qpos']);d.qpos[:]=initial;d.ctrl[:]=report['candidates'][offset]['ctrl'];mujoco.mj_forward(model,d)
        targets=np.clip(base+np.asarray(delta).reshape(args.waypoints,16),lower,upper);minimum=np.pi/2;hold=0;longest=0;fault=None;trace=[];max_shift=0.
        target_error=np.pi/2;tip=axis=0.;shift=0.
        for k in range(args.waypoints*120+130):
            if k<args.waypoints*120 and k%args.control_steps==0:
                phase=k//120;previous=base if phase==0 else targets[phase-1]
                d.ctrl[:]=previous+(targets[phase]-previous)*min((k%120)+args.control_steps,120)/120
            elif k>=args.waypoints*120:d.ctrl[:]=targets[-1]
            mujoco.mj_step(model,d)
            if k%5!=4:continue
            if not np.isfinite(d.qpos).all():fault='nonfinite';break
            shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[-7:-4]));max_shift=max(max_shift,shift)
            penetration=max([-float(c.dist) for c in d.contact if c.geom1 in support or c.geom2 in support]or[0])
            if penetration>.0005:fault='support_contact'
            elif shift>.08:fault='cube_left_hand_region'
            tip,axis,target_error=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));minimum=min(minimum,target_error)
            hold=hold+1 if eligible(tip,axis,target_error,shift,xp=np) and not fault else 0;longest=max(longest,hold)
            if record:trace.append(dict(step=k+1,qpos=d.qpos.tolist(),ctrl=d.ctrl.tolist(),tip_deg=float(np.rad2deg(tip)),target_error_deg=float(np.rad2deg(target_error)),shift_mm=shift*1000))
            if fault:break
        cost=target_error+.35*minimum+.2*axis+min(shift/.03,4)*.3+(10 if fault else 0)-(5 if longest>=23 else 0)
        result=dict(cost=float(cost),minimum_target_error_deg=float(np.rad2deg(minimum)),final_target_error_deg=float(np.rad2deg(target_error)),
                    final_tip_deg=float(np.rad2deg(tip)),off_axis_deg=float(np.rad2deg(axis)),final_shift_mm=shift*1000,max_shift_mm=max_shift*1000,
                    longest_hold_steps=longest,success=longest>=23,fault=fault,delta=np.asarray(delta).tolist())
        if record:result['trace']=trace
        return result
    def score(delta):
        results=[evaluate(delta,i) for i in range(5)] if args.robust else [evaluate(delta)]
        worst=dict(max(results,key=lambda r:r['cost']))
        worst['cost']=max(r['cost'] for r in results)+.2*np.mean([r['cost'] for r in results]) if args.robust else worst['cost']
        worst['success']=all(r['success'] for r in results);worst['successful_offsets']=sum(r['success'] for r in results)
        return worst
    rng=np.random.default_rng(12012);mean=np.zeros(args.waypoints*16);std=np.full_like(mean,.65);best=None;iterations=[]
    if args.resume:
        prior=json.loads((ROOT/args.resume).read_text());seed=np.array(prior['validation'][0]['delta']).reshape(-1,16)
        seed=np.r_[seed,np.repeat(seed[-1:],max(0,args.waypoints-len(seed)),axis=0)][:args.waypoints]
        mean=seed.ravel();best=score(mean)
    with ThreadPoolExecutor(max_workers=4) as pool:
        for iteration in range(args.iterations):
            population=rng.normal(mean,std,(args.population,len(mean)));population[0]=mean
            if best is not None:
                population[1]=best['delta']
                count=args.population//2;population[-count:]=rng.normal(best['delta'],.35,(count,len(mean)))
            results=list(pool.map(score,population));ranked=sorted(results,key=lambda r:r['cost']);elites=np.asarray([r['delta'] for r in ranked[:8]])
            mean=.25*mean+.75*elites.mean(0);std=np.maximum(.18,.25*std+.75*elites.std(0))
            if best is None or ranked[0]['cost']<best['cost']:best=ranked[0]
            row=dict(iteration=iteration+1,evaluations=(iteration+1)*args.population,faults=sum(r['fault'] is not None for r in results),best={k:v for k,v in best.items() if k!='delta'})
            iterations.append(row);print(json.dumps(row),flush=True)
            save(ROOT/(args.name+'_progress.json'),dict(status='running',algorithm='CEM_not_SAC',motor_commands_sent=0,iterations=iterations,best=best))
            if best['success']:break
    validation=[evaluate(best['delta'],i,record=i==0) for i in range(len(report['candidates']))]
    result=dict(status='offline_CEM_search_complete_NOT_SAC',motor_commands_sent=0,model=manifest,evaluations=iterations[-1]['evaluations'],waypoints=args.waypoints,
                control_period_seconds=dt*args.control_steps,robust_optimization=args.robust,
                elapsed_seconds=time.monotonic()-started,iterations=iterations,validation=validation,
                caveat='Native model only, two interpolated waypoint targets, provisional actuation/friction. Search failure is not proof of physical impossibility. No hardware policy exported.')
    save(ROOT/(args.name+'_result.json'),result)
    print(json.dumps(dict(status=result['status'],elapsed_seconds=result['elapsed_seconds'],validation=[{k:v for k,v in r.items() if k not in ('delta','trace')} for r in validation]),indent=2),flush=True)

if __name__=='__main__':main()
