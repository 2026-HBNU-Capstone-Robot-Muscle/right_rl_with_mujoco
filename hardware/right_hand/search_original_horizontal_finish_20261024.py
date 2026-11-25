"""Exploratory completion toward a whole 90-degree yaw, without devices.

Starts after the original teacher's horizontal push, then searches new contact
goals. Native limits and self collision tests remain; cube state is never set
to the target. This is prescribed-hand simulation, not physical validation.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from adapt_original_sequence_clearance_20261024 import build, OUT
from hand_motor_mapping import ticks_to_sim, sim_to_ticks, TICK
from sac_cube_model import set_cube_physics


def main():
    prior=json.loads((OUT/'horizontal_cube_dynamics.json').read_bytes())
    controls=[r for r in prior['runs'] if r['valid_start'] and r['control_dt_s']==.05]
    controls.sort(key=lambda r:(not r['final_three_second_hold_retained'],r['final']['ninety_yaw_target_error_deg']))
    case=controls[0]['placement']
    path=json.loads((OUT/'horizontal_reference_primitive.json').read_bytes());qpath=ticks_to_sim(path['goal_ticks'])
    m,manifest=build();set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);q0=qpath[-1].copy()
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);dt=m.opt.timestep
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    kind=mujoco.mjtState.mjSTATE_INTEGRATION;state_size=mujoco.mj_stateSize(m,kind)
    lo,hi=m.jnt_range[:16].T;moving=np.array([0,1,2,3,4,5,6,7,12,13,14,15])
    lower=np.maximum(lo[moving],q0[moving]-.55);upper=np.minimum(hi[moving],q0[moving]+.55)
    def pin(q,velocity=None):
        d.qpos[:16]=q;d.qvel[:16]=0 if velocity is None else velocity;d.ctrl[:]=q;mujoco.mj_step(m,d)
    def baseline(friction):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction;d.qpos[:16]=qpath[0];d.ctrl[:]=qpath[0]
        d.qpos[-7:]=np.r_[case['center_m'],Rotation.from_matrix(case['R']).as_quat()[[3,0,1,2]]];mujoco.mj_forward(m,d)
        for _ in range(round(1.5/dt)):pin(qpath[0])
        R0=d.xmat[body].reshape(3,3).copy();p0=d.qpos[-7:-4].copy()
        times=np.arange(len(qpath))*.05;elapsed=0.
        while elapsed<=times[-1]+2.:
            q=np.array([np.interp(elapsed,times,qpath[:,j]) for j in range(16)])
            nxt=np.array([np.interp(elapsed+dt,times,qpath[:,j]) for j in range(16)])
            pin(q,(nxt-q)/dt);elapsed+=dt
        state=np.empty(state_size);mujoco.mj_getState(m,d,state,kind)
        return dict(state=state,R0=R0,p0=p0,target=Rotation.from_rotvec(up*np.pi/2).as_matrix()@R0)
    caches={f:baseline(f) for f in (.4,.8,1.2)}
    def restore(friction):
        m.geom_friction[cube,0]=friction;mujoco.mj_setState(m,d,caches[friction]['state'],kind);mujoco.mj_forward(m,d)
    def route(x):
        goal=q0.copy();goal[moving]=x;raw0=sim_to_ticks(q0);raw1=sim_to_ticks(goal)
        steps=max(40,int(np.ceil(np.max(abs(raw1-raw0))/2)))
        # Linear interpolation has <=2 ticks at every hardware-style sample.
        raw=np.array([np.rint(raw0+(raw1-raw0)*k/steps).astype(int) for k in range(1,steps+1)])
        return ticks_to_sim(raw),raw
    def static_free(path):
        for q in np.vstack([path[::max(1,len(path)//30)],path[-1:]]):
            d.qpos[:16]=q;d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
            for c in d.contact:
                if c.dist>=-1e-7:continue
                a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if ha and hb or a in supports and hb or b in supports and ha:return False
        return True
    def status(friction):
        cache=caches[friction];R=d.xmat[body].reshape(3,3).copy();p=d.qpos[-7:-4].copy()
        target_error=float(np.degrees(Rotation.from_matrix(R@cache['target'].T).magnitude()))
        a=cache['R0'][:,0]-up*(up@cache['R0'][:,0]);b=R[:,0]-up*(up@R[:,0])
        yaw=float(np.degrees(np.arctan2(up@np.cross(a,b),a@b)))
        tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0,-1.,0]))@(cache['R0']@np.array([0,-1.,0])),-1,1))))
        touch=[]
        for c in d.contact:
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y) or c.dist>.0003:continue
            g=y if x==cube else x;name=m.body(int(m.geom_bodyid[g])).name
            if name=='palm' or name.startswith(('if_','mf_','rf_','th_')):touch.append(name)
        shift=float(np.linalg.norm(p-cache['p0'])*1000)
        return dict(target_error_deg=target_error,yaw_heading_deg=yaw,green_normal_tilt_deg=tilt,
            displacement_mm=shift,hand_contacts=sorted(set(touch)),retained=bool(touch and shift<100),
            cube_center_m=p.tolist(),cube_quat_wxyz=d.qpos[-4:].tolist())
    def simulate(path,friction=.8,record=False):
        restore(friction);previous=q0;hold=[];frames=[]
        for goal in path:
            velocity=(goal-previous)/.05
            cycles=round(.05/dt)
            for k in range(1,cycles+1):pin(previous+(goal-previous)*k/cycles,velocity)
            previous=goal
            if record:frames.append(dict(qpos=d.qpos.tolist()))
            if np.linalg.norm(d.qpos[-7:-4]-caches[friction]['p0'])>.15:break
        for k in range(round(2/dt)):
            pin(previous)
            if (k+1)%round(.25/dt)==0:hold.append(status(friction))
            if record and k%5==0:frames.append(dict(qpos=d.qpos.tolist()))
        final=status(friction);final['hold_retained']=all(h['retained'] for h in hold)
        final['frames']=frames if record else None
        return final
    def score(run):
        penalty=200 if not run['retained'] or not run['hold_retained'] else 0
        penalty+=3*max(0,run['green_normal_tilt_deg']-15)+2*max(0,run['displacement_mm']-65)
        return float(-run['target_error_deg']-penalty)
    control=simulate(np.tile(q0,(100,1)));print('NO_ADDITIONAL_PUSH',json.dumps(control),flush=True)
    rng=np.random.default_rng(202610242305);mean=q0[moving].copy();history=[];all_rows=[]
    for generation,spread in enumerate((.24,.18,.12,.08)):
        population=[mean]
        for _ in range(47):population.append(np.clip(mean+rng.normal(0,spread,len(moving)),lower,upper))
        rows=[]
        for x in population:
            path,raw=route(x)
            if not static_free(path):continue
            run=simulate(path);row=dict(x=np.asarray(x).tolist(),score=score(run),nominal=run,
                samples=len(path),last_goal_ticks=raw[-1].tolist())
            rows.append(row)
        rows.sort(key=lambda r:r['score'],reverse=True)
        history.append(dict(generation=generation,valid=len(rows),best=rows[:5]))
        if not rows:print('GENERATION',generation,'no static free candidate',flush=True);break
        mean=np.mean([r['x'] for r in rows[:6]],axis=0);all_rows.extend(rows)
        print('GENERATION',generation,'valid',len(rows),'best',rows[0]['nominal'],flush=True)
    all_rows.sort(key=lambda r:r['score'],reverse=True);finalists=[]
    for row in all_rows[:8]:
        path,raw=route(row['x']);runs=[]
        for friction in (.4,.8,1.2):runs.append(dict(friction=friction,result=simulate(path,friction)))
        accepted=all(s['result']['retained'] and s['result']['hold_retained'] and s['result']['target_error_deg']<15
            and s['result']['green_normal_tilt_deg']<20 for s in runs)
        finalists.append(dict(goal_ticks=raw.tolist(),runs=runs,robust_ninety_target=accepted))
        if accepted:break
    report=dict(optimizer='offline_bounded_cross_entropy_not_SAC',reference_primitive='horizontal_reference_primitive.json',
        initial_placement=case,held_baseline_control=control,history=history,finalists=finalists,
        collision_screened_middle_ring_hulls=True,native_ranges_unchanged=True,joint_zeros_not_fitted=True,
        cube_state_never_attached_or_forced_to_target=True,desired_whole_cube_yaw_deg=90,
        robust_candidate_found=any(f['robust_ninety_target'] for f in finalists),
        actual_motor_response_identified=False,hardware_execution_allowed=False,hardware_commands_sent=0,
        physical_ninety_degree_rotation_verified=False,support=manifest)
    (OUT/'whole_ninety_finish_search.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('FINAL',json.dumps(dict(robust_candidate_found=report['robust_candidate_found'],
        best=finalists[0]['runs'] if finalists else None)),flush=True)


if __name__=='__main__':main()
