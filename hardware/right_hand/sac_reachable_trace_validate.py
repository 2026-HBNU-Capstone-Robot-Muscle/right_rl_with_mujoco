"""Independent quantized-path evaluation, native MuJoCo and MJX; no hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import numpy as np
import mujoco
from mujoco import mjx
import jax
import jax.numpy as jp
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
from sac_forward_geometry import measure, eligible
ROOT=Path(__file__).resolve().parent
TICK=2*np.pi/4096

def main():
    p=argparse.ArgumentParser();p.add_argument('--backend',choices=['native','mjx'],required=True)
    p.add_argument('--id16-cap-ticks',type=int)
    p.add_argument('--id16-assumed-upper-ticks',type=int)
    args=p.parse_args()
    bounded=args.id16_cap_ticks is not None
    if bounded != (args.id16_assumed_upper_ticks is not None):p.error('Provide both the goal cap and hypothetical mechanical upper bound')
    if bounded and not 2048 < args.id16_cap_ticks < args.id16_assumed_upper_ticks < 4096:p.error('Invalid bounded diagnostic angles')
    path=ROOT/'sac_reachable_trace_search_result.json';doc=json.loads(path.read_text())
    candidate=next(r for r in doc['results'] if r['compensation_rad']==.07 and r['time_scale']==1.)
    if not candidate['passed']:raise ValueError('Candidate search failed')
    # Evaluate the exact integer motor goals, including their quantization.
    goals=np.rint((np.array(candidate['goals_sim_order'])+np.pi)/TICK)*TICK-np.pi
    goals=np.r_[goals,np.repeat(goals[-1:],30,axis=0)]
    resets_path=ROOT/'sac_cube55_resets.json';resets=json.loads(resets_path.read_text())['candidates']
    if not all(r['passed'] for r in resets):raise ValueError('Reset screening failed')
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    model=env.mj_model;support=np.array([model.geom(n).id for n in ('support_glass','support_wood')]);cube=model.geom('cube').id
    if bounded:
        joint=int(model.actuator_trnid[11,0])
        if model.joint(joint).name!='rf_dip':raise ValueError('ID16 joint mapping mismatch')
        upper=args.id16_assumed_upper_ticks*TICK-np.pi
        model.jnt_range[joint,1]=upper
        goals[:,11]=np.minimum(goals[:,11],args.id16_cap_ticks*TICK-np.pi)
    original=(model.geom_contype[cube],model.geom_conaffinity[cube]);substeps=round(.05/model.opt.timestep)
    output=ROOT/f'sac_reachable_trace_{args.backend}_validation.json';rows=[]
    if bounded:output=ROOT/f'sac_id16_cap{args.id16_cap_ticks}_{args.backend}_validation.json'
    for loaded in (False,True):
        model.geom_contype[cube]=original[0] if loaded else 0;model.geom_conaffinity[cube]=original[1] if loaded else 0
        initial=np.array([r['qpos'] for r in resets]);start=initial[:,-7:]
        if bounded:initial[:,11]=np.minimum(initial[:,11],upper)
        if args.backend=='native':
            samples=[]
            for q in initial:
                d=mujoco.MjData(model);d.qpos[:]=q;d.ctrl[:]=goals[0];mujoco.mj_forward(model,d)
                trial=[]
                for goal in goals:
                    d.ctrl[:]=goal;penetration=0.
                    for _ in range(substeps):
                        mujoco.mj_step(model,d)
                        penetration=max(penetration,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                    trial.append((d.qpos.copy(),float(np.max(np.abs(d.qpos[:16]-goal))),penetration,False))
                samples.append(trial)
            qs=np.array([[t[0] for t in row] for row in samples]).transpose(1,0,2)
            errors=np.array([[t[1] for t in row] for row in samples]).T
            pens=np.array([[t[2] for t in row] for row in samples]).T;saturated=np.zeros_like(pens,dtype=bool)
        else:
            mm=mjx.put_model(model,impl='jax');base=mjx.make_data(model,impl='jax')
            data=jax.jit(jax.vmap(lambda q:mjx.forward(mm,base.replace(qpos=q,ctrl=jp.asarray(goals[0])))))(jp.asarray(initial))
            def one(d,goal):
                d=d.replace(ctrl=goal)
                def physics(c,_):
                    d,pen,sat=c;d=mjx.step(mm,d);contact=d.contact
                    mask=jp.any(contact.geom[...,None]==jp.asarray(support),axis=(-1,-2))
                    pen=jp.maximum(pen,jp.max(jp.where(mask,jp.maximum(-contact.dist,0),0))*1000)
                    return (d,pen,sat|jp.all(contact.dist<=0)),None
                (d,pen,sat),_=jax.lax.scan(physics,(d,jp.asarray(0.),jp.asarray(False)),None,length=substeps)
                return d,(d.qpos,jp.max(jp.abs(d.qpos[:16]-goal)),pen,sat)
            @jax.jit
            def block(data,batch):
                return jax.lax.scan(lambda d,g:jax.vmap(one,in_axes=(0,None))(d,g),data,batch)
            saved=[]
            for first in range(0,len(goals),10):
                data,records=block(data,jp.asarray(goals[first:first+10]));saved.append(jax.device_get(records))
                print(f'MJX loaded={loaded} step {min(first+10,len(goals))}/{len(goals)}',flush=True)
            qs,errors,pens,saturated=[np.concatenate([record[k] for record in saved],axis=0) for k in range(4)]
        for i,r in enumerate(resets):
            hold=longest=0;maxshift=0
            for t,q in enumerate(qs[:,i]):
                tip,axis,err=map(float,measure(start[i,-4:],q[-4:],xp=np));shift=float(np.linalg.norm(q[-7:-4]-start[i,:3]));maxshift=max(maxshift,shift)
                valid=eligible(tip,axis,err,shift,xp=np) and pens[t,i]<=.5 and not saturated[t,i]
                hold=hold+1 if valid else 0;longest=max(longest,hold)
            finite=bool(np.isfinite(qs[:,i]).all())
            passed=finite and errors[:,i].max()<=.18 and pens[:,i].max()<=.5 and not saturated[:,i].any() and (not loaded or longest>=23)
            row=dict(loaded=loaded,reset_index=i,passed=bool(passed),longest_hold_steps=longest,
                     max_tracking_rad=float(errors[:,i].max()),max_support_penetration_mm=float(pens[:,i].max()),
                     contact_buffer_full=bool(saturated[:,i].any()),finite=finite,final_target_error_deg=float(np.degrees(err)) if loaded else None,
                     final_tip_deg=float(np.degrees(tip)) if loaded else None,max_center_shift_mm=maxshift*1000 if loaded else None)
            rows.append(row);print(json.dumps(row),flush=True)
        out=dict(status='offline_fixed_reference_validation',backend=args.backend,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                 reset_sha256=hashlib.sha256(resets_path.read_bytes()).hexdigest(),compensation_rad=.07,time_scale=1.,quantized_ticks=True,
                 control_dt=.05,extra_hold_steps=30,results=rows,all_passed=bool(all(r['passed'] for r in rows)),
                 motor_commands_sent=0,live_policy=False,caveat='Refined fixed trajectory, not retrained SAC or physical success.')
        if bounded:
            out.update(hypothetical_id16_upper_ticks=args.id16_assumed_upper_ticks,id16_goal_cap_ticks=args.id16_cap_ticks,
                       prepared_start_ticks_sim_order=np.rint((goals[0]+np.pi)/TICK).astype(int).tolist(),
                       hardware_authorized=False,physical_contact_cause='unknown; not a measured permanent joint limit')
        output.write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
