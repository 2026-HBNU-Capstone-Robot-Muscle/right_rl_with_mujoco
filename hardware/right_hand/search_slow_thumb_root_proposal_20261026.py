"""OFFLINE comparison: thumb retreat first, then raise index/middle roots.

The measured slow joint path is prescribed. Cube seating, friction, gravity
alignment and effective infinite hand inertia remain diagnostic assumptions.
No generated candidate is approved for hardware by this study.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,hashlib,itertools,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'slow_forward_placement_20261026_045454_900706'
BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
MOTION=('stage_6_loaded_quarter_reference','stage_7_loaded_root_finish','stage_8_loaded_observation','stage_2_fixed_hold','loaded_turn_observation')
def rot(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
def quat(R):return Rotation.from_matrix(R).as_quat()[[3,0,1,2]]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['baseline','search','validate'],default='baseline')
    ap.add_argument('--s',nargs='+',type=int,default=[36,40,44]);ap.add_argument('--dt',type=float,default=.002)
    ap.add_argument('--tag',default='thumb_then_roots_dt002');ap.add_argument('--friction',type=float,default=.3)
    ap.add_argument('--root-values',nargs='+',type=int)
    ap.add_argument('--small-thumb-only',action='store_true')
    ap.add_argument('--large-thumb-only',action='store_true')
    ap.add_argument('--sequential-thumb',action='store_true')
    ap.add_argument('--baseline-cache-tag')
    args=ap.parse_args()
    if Path(args.tag).name!=args.tag:raise ValueError('Single output folder name required')
    out=BASE/args.tag;out.mkdir(exist_ok=True)
    blob=(RUN/'result.json').read_bytes();saved=json.loads(blob)
    measured=[s for s in saved['samples'] if s['phase'] in MOTION]
    assert len([s for s in measured if s['phase']==MOTION[0]])==760
    assert len([s for s in measured if s['phase']==MOTION[1]])==64
    raw=np.array([s['actual'] for s in measured]);q=ticks_to_sim(raw)
    grid=np.array([s['t'] for s in measured]);grid-=grid[0];endraw=raw[-1].copy()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,out/'scene.xml')
    set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6;m.opt.timestep=args.dt
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;m.geom_friction[cube,0]=args.friction
    probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id
    R=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm].copy();up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x)
    frame=np.column_stack([x,np.cross(up,x),up]);cubeR=np.column_stack([x,-up,np.cross(x,-up)])
    m.opt.gravity[:]=-9.81*up
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    step=float(m.opt.timestep)
    def observe(d):
        names=set();cd=sd=hd=0.;external=False;selfpairs={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            dep=max(0,-float(c.dist)*1000)
            if ha and hb:
                sd=max(sd,dep)
                if dep>0:
                    pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                    selfpairs[pair]=max(selfpairs.get(pair,0.),dep)
            if (a in supports and hb) or (b in supports and ha):hd=max(hd,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cd=max(cd,dep)
                    if c.dist<.0003:names.add(m.geom(other).name)
                elif c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return dict(supports=sorted(names),cube_overlap_mm=cd,self_overlap_mm=sd,support_overlap_mm=hd,external=external,self_pairs=selfpairs)
    def final_metrics(d,target):
        rf=rot(d.qpos[-4:]);error=float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1))))
        return dict(target_error_deg=error,red_up_error_deg=float(np.degrees(np.arccos(np.clip(rf[:,0]@up,-1,1)))),
                    green_wrist_error_deg=float(np.degrees(np.arccos(np.clip((-rf[:,1])@(-x),-1,1)))))
    def baseline(s):
        cache=out/f'baseline_s{s}.npz';report=out/f'baseline_s{s}.json'
        if args.baseline_cache_tag:
            if Path(args.baseline_cache_tag).name!=args.baseline_cache_tag:raise ValueError('Invalid baseline cache tag')
            prior=BASE/args.baseline_cache_tag
            provenance=json.loads((prior/'search.json').read_bytes())
            if provenance['source_sha256']!=hashlib.sha256(blob).hexdigest() or provenance['timestep']!=step or provenance['cube_friction']!=args.friction:
                raise ValueError('Baseline provenance mismatch')
            cache=prior/f'baseline_s{s}.npz';report=prior/f'baseline_s{s}.json'
        if cache.exists() and report.exists():return json.loads(report.read_bytes()),np.load(cache)
        d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];height=45.
        d.qpos[-4:]=quat(cubeR)
        while True:
            d.qpos[-7:-4]=origin+frame@(np.array([s,15,height])*.001);mujoco.mj_forward(m,d)
            if observe(d)['cube_overlap_mm']<=.05 or height>=95:break
            height+=.5
        start=d.qpos[-7:].copy()
        for _ in range(round(1/step)):
            d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();initial=observe(d)
        initial_green=float(np.degrees(np.arccos(np.clip((-rot(settled[-4:])[:,1])@up,-1,1))))
        initial_valid=bool(initial['supports'] and initial_green<=12 and not initial['external'] and np.linalg.norm(settled[:3]-start[:3])<=.012)
        target=target_quaternion(settled[-4:])
        ts=np.arange(0,grid[-1]+step/2,step);poses=np.column_stack([np.interp(ts,grid,q[:,j]) for j in range(16)])
        vel=np.gradient(poses,step,axis=0);trace=[];tt=[];maxdepth=0.;maxself=0.;maxsupport=0.;external=False
        for i,t in enumerate(ts):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            if i%max(1,round(.01/step))==0:
                obs=observe(d);maxdepth=max(maxdepth,obs['cube_overlap_mm']);maxself=max(maxself,obs['self_overlap_mm']);maxsupport=max(maxsupport,obs['support_overlap_mm']);external|=obs['external']
            if i%max(1,round(.025/step))==0:trace.append(d.qpos.copy());tt.append(t)
            mujoco.mj_step(m,d)
        d.qpos[:16]=q[-1];d.qvel[:16]=0;mujoco.mj_forward(m,d)
        result=dict(s=s,y=15,source='measured slow run',height_mm=height,initial_valid=initial_valid,
                    initial_green_up_error_deg=initial_green,initial_contacts=initial['supports'],
                    max_cube_overlap_mm=maxdepth,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,
                    external_cube_contact=external,final_contacts=observe(d)['supports'],**final_metrics(d,target))
        np.savez_compressed(cache,qpos=d.qpos.copy(),qvel=d.qvel.copy(),target=target,trace=np.array(trace),times=np.array(tt),endraw=endraw)
        report.write_text(json.dumps(result,indent=2),encoding='utf-8');print('BASELINE',json.dumps(result),flush=True)
        return result,np.load(cache)
    def trajectory(delta,extra):
        thumb=endraw.copy();thumb[:4]+=np.array(delta,dtype=int)
        end=thumb.copy();end[[7,11]]+=extra
        # Clear thumb completely, then lift. Never combine either with a jump.
        nt=max(1,int(np.ceil(max(abs(np.asarray(delta)))/2)));nr=max(1,int(np.ceil(extra/2)))
        knots=np.array([0,nt*.075,(nt+nr)*.075,(nt+nr)*.075+3.])
        raw_knots=np.vstack([endraw,thumb,end,end])
        if args.sequential_thumb:
            lift=endraw.copy();lift[3]=thumb[3]
            nl=max(1,int(np.ceil(abs(delta[3])/2)));nt=max(1,int(np.ceil(max(abs(np.asarray(delta)[:3]))/2)))
            knots=np.array([0,nl*.075,(nl+nt)*.075,(nl+nt+nr)*.075,(nl+nt+nr)*.075+3.])
            raw_knots=np.vstack([endraw,lift,thumb,end,end])
        qs=ticks_to_sim(raw_knots);ts=np.arange(0,knots[-1]+step/2,step)
        poses=np.column_stack([np.interp(ts,knots,qs[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0)
        return ts,poses,vel,knots
    def evaluate(seed,delta,extra):
        ts,poses,vel,knots=trajectory(delta,extra)
        row=dict(thumb_delta_ticks=list(delta),additional_root_ticks=extra,duration_s=float(ts[-1]),order='thumb CMC lift -> MCP/axial retreat -> root lift' if args.sequential_thumb else 'thumb clearance first, root lift second')
        if np.any(poses<m.jnt_range[:16,0]-1e-6) or np.any(poses>m.jnt_range[:16,1]+1e-6):return dict(**row,rejected='native_range'),None
        # Kinematic hand-only screen can reject obvious interference cheaply.
        d=mujoco.MjData(m);d.qpos[:]=seed['qpos'];d.qvel[:]=0
        for i in np.linspace(0,len(ts)-1,81).astype(int):
            d.qpos[:16]=poses[i];mujoco.mj_forward(m,d);obs=observe(d)
            if obs['self_overlap_mm']>.01 or obs['support_overlap_mm']>.15:
                return dict(**row,rejected='hand_or_support_contact',at_s=float(ts[i]),contact=obs),None
        d=mujoco.MjData(m);d.qpos[:]=seed['qpos'];d.qvel[:]=seed['qvel'];target=seed['target']
        maxdepth=maxself=maxsupport=gap=maxgap=0.;external=False;trace=[];times=[];hold=[];peakpairs={};complete=True
        sample_every=max(1,round(.01/step));save_every=max(1,round(.025/step))
        for i,t in enumerate(ts):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            if i%sample_every==0:
                obs=observe(d);maxdepth=max(maxdepth,obs['cube_overlap_mm']);maxself=max(maxself,obs['self_overlap_mm']);maxsupport=max(maxsupport,obs['support_overlap_mm']);external|=obs['external']
                gap=0 if obs['supports'] else gap+sample_every*step;maxgap=max(maxgap,gap)
                for p,dep in obs['self_pairs'].items():peakpairs[p]=max(peakpairs.get(p,0),dep)
                if t>=knots[-1]-3:hold.append(final_metrics(d,target)['target_error_deg'])
                if maxself>.01 or maxsupport>.15 or maxdepth>1. or external or maxgap>.2:
                    complete=False;break
            if i%save_every==0:trace.append(d.qpos.copy());times.append(t)
            mujoco.mj_step(m,d)
        final=final_metrics(d,target);obs=observe(d)
        accepted=bool(complete and final['target_error_deg']<=20 and final['red_up_error_deg']<=20 and final['green_wrist_error_deg']<=20 and hold and max(hold)<=20 and obs['supports'] and maxgap<=.1)
        row.update(complete=complete,**final,hold_error_deg=max(hold) if hold else None,final_contacts=obs['supports'],max_cube_overlap_mm=maxdepth,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,
                   max_no_hand_contact_s=maxgap,external_cube_contact=external,self_contact_peaks_mm=peakpairs,candidate_accepted=accepted,simulated_seconds=float(t))
        return row,dict(qpos=np.array(trace),times=np.array(times))
    results=[]
    for s in args.s:
        base,seed=baseline(s)
        if args.mode=='baseline':continue
        if not base['initial_valid']:
            print('SKIP INVALID SEATING',s,flush=True);continue
        deltas=[(0,0,0,0),(0,-64,0,0),(0,0,128,0),(0,-64,128,0),(0,-128,128,0),(0,-64,256,0),(0,-128,256,0),(0,-64,128,-64),(0,-64,256,-64),(0,-128,256,-128)]
        extras=(128,256,384)
        if args.small_thumb_only:deltas=[(0,-64,128,-64)]
        if args.large_thumb_only:
            if args.small_thumb_only:raise ValueError('Choose one thumb candidate')
            deltas=[(0,-128,256,-128)]
        if args.root_values:extras=tuple(args.root_values)
        if args.mode=='validate':
            chosen=json.loads((BASE/'thumb_then_roots_dt002/search.json').read_bytes())['best_candidates']
            combos=[(tuple(v['thumb_delta_ticks']),v['additional_root_ticks']) for v in chosen[:3]]
        else:combos=list(itertools.product(deltas,extras))
        for number,(delta,extra) in enumerate(combos):
            row,trace=evaluate(seed,delta,extra);row.update(s=s,case_index=number)
            results.append(row);print('CASE',json.dumps(row),flush=True)
            if row.get('candidate_accepted'):
                np.savez_compressed(out/f'accepted_s{s}_{number:03d}.npz',**trace)
            report=dict(cases=results,model=manifest,source_run=RUN.name,source_sha256=hashlib.sha256(blob).hexdigest(),timestep=step,cube_friction=args.friction,
                        cube_mass_kg=.1,contact_time_s=.004,gravity_aligned_to_book_is_hypothesis=True,measured_hand_trajectory_prescribed=True,
                        actual_cube_seating_not_registered=True,physical_friction_not_identified=True,electrical_model_absent=True,
                        hardware_execution_allowed=False,hardware_commands_sent=0,
                        best_candidates=sorted([v for v in results if v.get('candidate_accepted')],key=lambda v:(v['hold_error_deg'],v['duration_s'])))
            (out/'search.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if results:print('SUMMARY',json.dumps(dict(cases=len(results),accepted=sum(v.get('candidate_accepted',False) for v in results),best=report['best_candidates'][:3])),flush=True)
if __name__=='__main__':main()
