"""Cube diagnostic using the complete newly measured EMPTY joint trace.

The hand is prescribed with high effective inertia. Seating, friction, mass,
gravity alignment and contact parameters are assumptions, not calibrated input.
This diagnostic does not open hardware or itself authorize a loaded experiment.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,itertools,json,time
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from search_slow_thumb_root_proposal_20261026 import rot,quat
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'thumb_clear_root_empty_20261026_125659_859144'
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/measured_thumb_125659_cube_replay'
PHASES=('stage_6_empty_quarter_reference','stage_7_empty_known_root_finish','stage_8a_empty_thumb_lift_clearance',
    'stage_9_empty_thumb_retreat','stage_10_empty_index_middle_lift','stage_11_empty_new_pose_hold','stage_2_fixed_hold')
def main():
    started=time.monotonic();OUT.mkdir(exist_ok=True)
    blob=(RUN/'result.json').read_bytes();saved=json.loads(blob)
    if saved['status']!='thumb_clear_root_empty_complete' or not saved['all_torque_off_verified'] or not saved['settings_restored']:
        raise ValueError('Completed EMPTY source is incomplete')
    rows=[s for s in saved['samples'] if s['phase'] in PHASES]
    expected=(760,64,64,128,192,60,20)
    if tuple(sum(s['phase']==phase for s in rows) for phase in PHASES)!=expected:raise ValueError('Incomplete measured stages')
    q=ticks_to_sim(np.array([s['actual'] for s in rows]));grid=np.array([s['t'] for s in rows]);grid-=grid[0]
    if np.any(np.diff(grid)<=0):raise ValueError('Non-monotonic recorded samples')
    dt=.002;times=np.arange(0,grid[-1]+dt/2,dt)
    poses=np.column_stack([np.interp(times,grid,q[:,j]) for j in range(16)]);vel=np.gradient(poses,dt,axis=0)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1)
    m.dof_armature[:16]=1e6;m.opt.timestep=dt
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id
    R=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm].copy();up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x)
    frame=np.column_stack([x,np.cross(up,x),up]);cubeR=np.column_stack([x,-up,np.cross(x,-up)])
    m.opt.gravity[:]=-9.81*up
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        names=set();depth=0.;selfpairs={};support_depth=0.;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            dep=max(0.,-float(c.dist)*1000)
            if ha and hb and dep:
                pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)));selfpairs[pair]=max(selfpairs.get(pair,0.),dep)
            if (a in supports and hb) or (b in supports and ha):support_depth=max(support_depth,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    depth=max(depth,dep)
                    if c.dist<.0003:names.add(m.geom(other).name)
                elif c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return dict(hand_contacts=sorted(names),cube_overlap_mm=depth,self_pairs=selfpairs,support_overlap_mm=support_depth,external=external)
    def metrics(d,target):
        C=rot(d.qpos[-4:]);qfinal=d.qpos[-4:]/np.linalg.norm(d.qpos[-4:])
        return dict(target_error_deg=float(np.degrees(2*np.arccos(np.clip(abs(qfinal@target),0,1)))),
            red_up_error_deg=float(np.degrees(np.arccos(np.clip(C[:,0]@up,-1,1)))),
            green_wrist_error_deg=float(np.degrees(np.arccos(np.clip((-C[:,1])@(-x),-1,1)))))
    results=[]
    for s,friction in itertools.product((36,40,44),(.2,.3,.5)):
        m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-4:]=quat(cubeR)
        height=45.
        while True:
            d.qpos[-7:-4]=origin+frame@(np.array([s,15,height])*.001);mujoco.mj_forward(m,d)
            if observe(d)['cube_overlap_mm']<=.05 or height>=95:break
            height+=.5
        initial=d.qpos[-7:].copy()
        for _ in range(round(1/dt)):
            d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();obs=observe(d)
        green=float(np.degrees(np.arccos(np.clip((-rot(settled[-4:])[:,1])@up,-1,1))))
        valid=bool(obs['hand_contacts'] and green<=12 and not obs['external'] and np.linalg.norm(settled[:3]-initial[:3])<=.012)
        target=target_quaternion(settled[-4:]);maximum_cube=maximum_support=gap=maxgap=0.;selfpeaks={};external=False;hold=[];trace=[];tt=[]
        for i,t in enumerate(times):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            if i%5==0:
                obs=observe(d);maximum_cube=max(maximum_cube,obs['cube_overlap_mm']);maximum_support=max(maximum_support,obs['support_overlap_mm']);external|=obs['external']
                gap=0 if obs['hand_contacts'] else gap+.01;maxgap=max(maxgap,gap)
                for pair,dep in obs['self_pairs'].items():selfpeaks[pair]=max(selfpeaks.get(pair,0.),dep)
                if t>=times[-1]-3:hold.append(metrics(d,target)['target_error_deg'])
            if i%25==0:trace.append(d.qpos.copy());tt.append(float(t))
            mujoco.mj_step(m,d)
        d.qpos[:16]=q[-1];d.qvel[:16]=0;mujoco.mj_forward(m,d);obs=observe(d);final=metrics(d,target)
        accepted=bool(valid and all(v<=20 for v in final.values()) and hold and max(hold)<=20 and obs['hand_contacts']
            and not external and maximum_cube<=1. and maximum_support<=.15 and max(selfpeaks.values(),default=0.)<=.01 and maxgap<=.1)
        row=dict(s=s,friction=friction,initial_height_mm=height,initial_valid=valid,initial_green_up_error_deg=green,
            **final,hold_maximum_error_deg=max(hold),final_contacts=obs['hand_contacts'],
            maximum_cube_overlap_mm=maximum_cube,maximum_support_overlap_mm=maximum_support,
            raw_self_contact_maxima_mm=selfpeaks,maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external,candidate_accepted=accepted)
        np.savez_compressed(OUT/f'trace_s{s}_mu{friction:.1f}.npz',qpos=np.asarray(trace),times=np.asarray(tt))
        results.append(row);print('CASE',json.dumps(row),flush=True)
        review=dict(source_run=RUN.name,source_sha256=hashlib.sha256(blob).hexdigest(),model=manifest,cases=results,
            measured_hand_samples=len(rows),measured_hand_duration_seconds=float(grid[-1]),timestep_seconds=dt,
            cube_edge_m=.055,cube_mass_kg=.1,contact_time_s=.004,
            hand_kinematic_with_effective_infinite_inertia=True,book_gravity_alignment_assumed=True,
            actual_cube_placement_not_registered=True,friction_not_measured=True,
            hardware_execution_allowed=False,physical_rotation_verified=False,hardware_commands_sent=0,elapsed_seconds=time.monotonic()-started)
        (OUT/'review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(dict(cases=len(results),accepted=sum(c['candidate_accepted'] for c in results))),flush=True)
if __name__=='__main__':main()
