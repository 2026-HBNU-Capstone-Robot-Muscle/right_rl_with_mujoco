"""Offline geometric path projection from simulated attained angles, not hardware."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import minimize
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim,sim_to_ticks,metadata
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure,eligible

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def main():
    source=OUT/'native_replay.json';doc=json.loads(source.read_bytes())
    review=json.loads((OUT/'placement_review.json').read_bytes())
    ref=next(t for t in doc['trials'] if t['loaded'] and t['synthetic_reset']==0)
    measured=ticks_to_sim(review['placement_position_ticks'])
    start=ticks_to_sim(review['placement_goal_ticks'])
    seed=np.array([r['qpos'][:16] for r in ref['trace'][20:20+doc['candidate_samples']]])
    seed+=np.maximum(0,1-np.arange(len(seed))/10)[:,None]*(start-seed[0])[None,:]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.100)
    bounds=m.jnt_range[m.actuator_trnid[:,0]].copy();cb=m.actuator_ctrlrange.copy()
    lo=np.maximum(bounds[:,0],cb[:,0]);hi=np.minimum(bounds[:,1],cb[:,1])
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    cube=m.geom('cube').id;masks=(int(m.geom_contype[cube]),int(m.geom_conaffinity[cube]))
    d=mujoco.MjData(m)
    def forward(q):
        d.qpos[:16]=q;d.qpos[-7:]=[0,0,2,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
    def violations():
        rows=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            handhand=int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand
            handsupport=(a in support and int(m.geom_bodyid[b]) in hand) or (b in support and int(m.geom_bodyid[a]) in hand)
            if handhand or handsupport:rows.append((min(a,b),max(a,b),float(c.dist)))
        return rows
    pairs=set()
    for q in seed:
        forward(q)
        pairs.update((a,b) for a,b,dist in violations())
    def gap(a,b):
        if int(m.geom_type[a])==6 and int(m.geom_type[b])==6:
            ra=d.geom_xmat[a].reshape(3,3);rb=d.geom_xmat[b].reshape(3,3)
            axes=np.concatenate((ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)))
            norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-10]/norm[norm>1e-10,None]
            gaps=np.abs(axes@(d.geom_xpos[b]-d.geom_xpos[a]))
            gaps-=np.abs(axes@ra)@m.geom_size[a]+np.abs(axes@rb)@m.geom_size[b]
            return float(gaps.max())
        return float(mujoco.mj_geomDistance(m,d,a,b,.1,None))
    projected=[];details=[]
    for i,qref in enumerate(seed):
        previous=start if i==0 else projected[-1]
        if i==0:
            q=start.copy();success=True;message='preserve actually prepared command'
        else:
            # Each pose may move at most 20 degrees from the attained reference.
            # No native joint/control range change is made.
            lower=np.maximum(lo,qref-.35);upper=np.minimum(hi,qref+.35)
            q=np.minimum(np.maximum(qref,lower),upper)
            for attempt in range(3):
                plist=sorted(pairs)
                def con(x):
                    forward(x)
                    return np.array([(gap(a,b)-.001)/.01 for a,b in plist])
                def cost(x):
                    delta=x-qref;v=x-previous
                    return float(delta@delta+.04*(v@v))
                result=minimize(cost,q,method='SLSQP',bounds=list(zip(lower,upper)),
                    constraints=[dict(type='ineq',fun=con)],options=dict(maxiter=70,ftol=1e-9,eps=1e-5))
                q=result.x;forward(q)
                new={(a,b) for a,b,dist in violations() if dist<-.00001}-pairs
                if not new:break
                pairs.update(new)
            success=bool(result.success and min(con(q),default=1)>-1e-4)
            message=str(result.message)
        # Quantize candidate inward to the original ranges, then recheck it.
        raw=sim_to_ticks(q)
        qi=ticks_to_sim(raw);forward(qi)
        depth=max([-dist*1000 for a,b,dist in violations()]+[0])
        projected.append(qi)
        details.append(dict(sample=i,optimizer_success=success,message=message,max_static_overlap_mm=depth,
            max_change_from_reference_deg=float(np.degrees(np.max(abs(qi-qref))))))
        if i%20==0:print(json.dumps(details[-1]),flush=True)
    goals=np.array(projected);raw=sim_to_ticks(goals)
    candidates=dict(status='offline_geometric_projection',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mapping=metadata(),model_manifest=manifest,hardware_execution_allowed=False,motor_commands_sent=0,live_sac=False,
        native_ranges_unchanged=True,details=details,
        failed_samples=[r['sample'] for r in details if not r['optimizer_success'] or r['max_static_overlap_mm']>.01],
        start_ticks=raw[0].tolist(),goals_id_order=raw[1:].tolist(),trials=[],
        caveat='Geometric repair of an ideal-model attained path. Neither a learned policy nor a physical success.')
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates']
    for loaded,ri in [(False,0)]+[(True,i) for i in range(len(resets))]:
        m.geom_contype[cube]=masks[0] if loaded else 0;m.geom_conaffinity[cube]=masks[1] if loaded else 0
        d=mujoco.MjData(m);d.qpos[:16]=measured;d.qpos[-7:]=resets[ri]['qpos'][-7:];d.ctrl[:]=goals[0];mujoco.mj_forward(m,d)
        origin=d.qpos[-7:].copy();maxself=maxsupport=tracking=0.;best=streak=0;trace=[]
        for k,g in enumerate([goals[0]]*20+list(goals)+[goals[-1]]*20):
            d.ctrl[:]=g
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                for a,b,dist in violations():
                    if a in support or b in support:maxsupport=max(maxsupport,-dist*1000)
                    else:maxself=max(maxself,-dist*1000)
            tracking=max(tracking,float(np.max(abs(d.qpos[:16]-g))))
            tip,axis,error=map(float,measure(origin[-4:],d.qpos[-4:]))
            shift=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3]))
            ok=loaded and bool(eligible(tip,axis,error,shift));streak=streak+1 if ok else 0;best=max(best,streak)
            trace.append(dict(time_s=(k+1)*.05,qpos=d.qpos.tolist()))
        row=dict(loaded=loaded,reset=ri,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,
            max_tracking_rad=tracking,tip_deg=float(np.degrees(tip)) if loaded else None,
            off_axis_deg=float(np.degrees(axis)) if loaded else None,
            error_deg=float(np.degrees(error)) if loaded else None,shift_mm=shift*1000 if loaded else None,
            target_hold_s=best*.05,trace=trace)
        print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True);candidates['trials'].append(row)
    assert np.array_equal(bounds,m.jnt_range[m.actuator_trnid[:,0]]) and np.array_equal(cb,m.actuator_ctrlrange)
    (OUT/'projected_path_OFFLINE_ONLY.json').write_text(json.dumps(candidates,indent=2))
    print('Failed static projection samples:',candidates['failed_samples'],flush=True)


if __name__=='__main__': main()
