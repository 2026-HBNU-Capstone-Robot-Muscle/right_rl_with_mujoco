"""Native-motor quarter-turn search: stop after reaching the goal, clear thumb.

Changes only proposed goals and stop time. Keeps CAD, contact forces, native
limits, initial Cube state and torque parameters. No hardware execution.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,sys
from pathlib import Path
import numpy as np
import mujoco,mujoco.rollout
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model/short_turn_search'

def densify(raw):
    result=[raw[0]]
    for a,b in zip(raw[:-1],raw[1:]):
        n=max(1,int(np.ceil(max(abs(b-a))/2)))
        result.extend(np.rint(a+(b-a)*k/n).astype(int) for k in range(1,n+1))
    return np.array(result)

def main():
    global OUT
    full_grid='--full-grid' in sys.argv
    robust_grid='--robust-grid' in sys.argv
    if full_grid:OUT=OUT.with_name('tip_clearance_search_v2')
    if robust_grid:OUT=OUT.with_name('tip_clearance_search_v3')
    OUT.mkdir(exist_ok=True)
    trace=np.load(OUT.parent/'nominal_trace.npz');original=trace['goal_ticks'];initial=trace['initial_state']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);cube=m.geom('cube').id;m.geom_friction[cube,0]=.8
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    substeps=round(.05/m.opt.timestep);kind=mujoco.mjtState.mjSTATE_FULLPHYSICS;target=target_quaternion(initial[1+m.nq-4:1+m.nq])
    candidates=[]
    for gain_scale in ((.5,1.) if robust_grid else (1.,)):
     for stop in ((38.,(len(original)-1)*.05) if full_grid or robust_grid else (38.,39.5,41.)):
        for offset in ((-20,-24,-28) if robust_grid else (-16,-24,-32) if full_grid else (0,-4,-8,-16)):
          for ring_offset in ((-32,-40,-48,-56) if robust_grid else (-4,-8,-16,-24) if full_grid else (0,)):
            raw=original[:round(stop/.05)+1].copy();weight=np.clip((np.arange(len(raw))*.05-22)/2,0,1)
            raw[:,0]+=np.rint(offset*weight).astype(int)
            raw[:,12]+=np.rint(ring_offset*weight).astype(int);raw=densify(raw)
            candidates.append(dict(gain_scale=gain_scale,stop_seconds=stop,thumb_tip_offset_ticks=offset,ring_tip_offset_ticks=ring_offset,raw=raw))
    count=max(len(r['raw'])-1 for r in candidates)+80
    commands=[]
    for r in candidates:
        q=ticks_to_sim(r['raw']);commands.append(np.r_[q[1:],np.repeat(q[-1:],count-len(q)+1,axis=0)])
    controls=np.repeat(np.array(commands),substeps,axis=1)
    references=json.loads((BASE/'reverse_original_native_verification.json').read_bytes())
    initial_by_gain={g:np.array(next(r['initial_state'] for r in references['runs'] if r.get('mode')=='moving_path' and r['gain_scale']==g and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])) for g in {r['gain_scale'] for r in candidates}}
    native_gain=m.actuator_gainprm.copy();native_bias=m.actuator_biasprm.copy();states=[]
    with mujoco.rollout.Rollout(nthread=4) as pool:
        for g in sorted(initial_by_gain):
            m.actuator_gainprm[:]=native_gain;m.actuator_biasprm[:]=native_bias;m.actuator_gainprm[:,0]*=g;m.actuator_biasprm[:,1:3]*=g
            indices=[i for i,r in enumerate(candidates) if r['gain_scale']==g]
            ss,_=pool.rollout(m,[mujoco.MjData(m) for _ in range(4)],initial_by_gain[g][None],controls[indices])
            states.extend(ss)
    inspect=mujoco.MjData(m);results=[]
    for r,ss,ctrl in zip(candidates,states,controls):
        # Batch padding is not part of this candidate's four-second hold.
        own_end=(len(r['raw'])-1+80)*substeps;ss=ss[:own_end];ctrl=ctrl[:own_end]
        initial=initial_by_gain[r['gain_scale']];target=target_quaternion(initial[1+m.nq-4:1+m.nq])
        poses=ss[:,1:1+m.nq];errors=2*np.arccos(np.clip(abs(poses[:,-4:]@target)/np.maximum(np.linalg.norm(poses[:,-4:],axis=1),1e-12),0,1))
        hold_error=float(np.degrees(errors[-round(4/m.opt.timestep):].max()))
        record={k:v for k,v in r.items() if k!='raw'};record.update(hold_max_target_error_deg=hold_error,
            maximum_tracking_rad=float(np.max(abs(poses[:,:16]-ctrl))),native_self_clear=False,full_static_clear=False)
        first=None;gap=maxgap=0.;external=False;lasttouch=False;selfmax=supportmax=0.
        if hold_error<15:
            for i,state in enumerate(ss):
                mujoco.mj_setState(m,inspect,state,kind);inspect.ctrl[:]=ctrl[i];mujoco.mj_forward(m,inspect);touch=False
                for c in inspect.contact:
                    a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                    if c.dist<0 and ha and hb:
                        selfmax=max(selfmax,-float(c.dist)*1000)
                        if -c.dist*1000>1e-6:first=dict(time_seconds=(i+1)*m.opt.timestep,pair=[m.geom(a).name,m.geom(b).name],depth_mm=-float(c.dist)*1000)
                    if c.dist<0 and (a in support and hb or b in support and ha):supportmax=max(supportmax,-float(c.dist)*1000)
                    if cube in (a,b) and c.dist<.0003:
                        other=b if a==cube else a
                        if int(m.geom_bodyid[other]) in hand:touch=True
                        if other in support or m.geom(other).name=='floor':external=True
                if first:break
                gap=0. if touch else gap+m.opt.timestep;maxgap=max(maxgap,gap);lasttouch=touch
            record.update(native_self_clear=first is None,first_self_contact=first,maximum_self_overlap_mm=selfmax,
                maximum_support_overlap_mm=supportmax,maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external)
            if first is None:
                maximum=0.
                for q in ticks_to_sim(r['raw']):
                    inspect.qpos[:16]=q;inspect.qpos[-7:]=[10,10,10,1,0,0,0];inspect.qvel[:]=0;mujoco.mj_forward(m,inspect)
                    for c in inspect.contact:
                        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                        if c.dist<0 and (ha and hb or a in support and hb or b in support and ha):maximum=max(maximum,-float(c.dist)*1000)
                record.update(full_static_clear=maximum<=1e-6,maximum_static_overlap_mm=maximum)
        record['complete_nominal_model_pass']=bool(hold_error<15 and record['native_self_clear'] and record['full_static_clear'] and record['maximum_tracking_rad']<.12 and not external and maxgap<=.1 and lasttouch and supportmax<=.15)
        if record['complete_nominal_model_pass']:
            name=f"candidate_g{r['gain_scale']:g}_stop{r['stop_seconds']:g}_tip{r['thumb_tip_offset_ticks']}_ring{r['ring_tip_offset_ticks']}"
            np.savez_compressed(OUT/(name+'.npz'),states=ss,controls=ctrl,goal_ticks=r['raw'],initial_state=initial)
            record['candidate_path']=name+'.npz'
        results.append(record);print(json.dumps(record),flush=True)
    (OUT/'summary.json').write_text(json.dumps(dict(cases=results,model=manifest,hardware_execution_allowed=False,hardware_commands_sent=0),indent=2),encoding='utf-8')

if __name__=='__main__':main()
