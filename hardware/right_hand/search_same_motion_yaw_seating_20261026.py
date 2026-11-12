"""Search conditional seating for a horizontal turn using already measured motion.

No new motor goals are generated here. A promising prefix is still only a
model diagnostic; the actual seating and loaded response are unregistered.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,hashlib,itertools,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'loaded_repeat_20261026_170932_163649/result.json'
OUT=ROOT/'right_hand_rotation_20261026/yaw_same_reference_seating_v1'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--original-yaw',choices=('original','ring_fixed'));parser.add_argument('--goal-path');parser.add_argument('--max-initial-tilt',type=float,default=12.);args=parser.parse_args()
    output=(OUT.parent/('yaw_clear_candidate_'+Path(args.goal_path).stem) if args.goal_path else (OUT if args.original_yaw is None else OUT.parent/('yaw_original_from_ready_'+args.original_yaw)))
    output.mkdir(parents=True,exist_ok=True);saved=json.loads(SOURCE.read_bytes());rows=[s for s in saved['samples'] if s['phase']=='stage_6_loaded_quarter_reference']
    grid=np.asarray([s['t'] for s in rows]);grid-=grid[0];q=ticks_to_sim([s['actual'] for s in rows]);dt=.002
    raw_goals=None
    if args.goal_path:
        path=(ROOT/args.goal_path).resolve();path.relative_to(ROOT);bundle=np.load(path);raw_goals=np.asarray(bundle['goals']);residual=np.asarray(bundle['response_residual_ticks'])
        from loaded_quarter_reentry_20261025 import limits
        lo,hi=limits();assert np.all(raw_goals>=lo) and np.all(raw_goals<=hi) and np.max(abs(np.diff(raw_goals,axis=0)))<=2
        raw_goals=np.vstack([raw_goals,np.repeat(raw_goals[-1][None,:],27,axis=0)]);q=ticks_to_sim(raw_goals+residual);grid=np.arange(len(q))*.075
    if args.original_yaw:
        from loaded_thumb_rotation_plan_20261026 import motion_goals
        original=json.loads((ROOT/'sequence_flip_poses.json').read_bytes());start=motion_goals()[0].copy();previous=start.copy();raw_goals=[start.copy()]
        for v in original['poses'][3:]:
            target=np.asarray(v['raw_ticks']).copy()
            if args.original_yaw=='ring_fixed':target[12:]=start[12:]
            n=max(1,int(np.ceil(np.max(abs(target-previous))/2)))
            raw_goals.extend(np.rint(previous+(target-previous)*i/n).astype(int) for i in range(1,n+1));previous=target
        raw_goals=np.asarray(raw_goals);raw_goals=np.vstack([raw_goals,np.repeat(raw_goals[-1][None,:],27,axis=0)])
        from loaded_quarter_reentry_20261025 import limits
        lo,hi=limits();assert np.all(raw_goals>=lo) and np.all(raw_goals<=hi) and np.max(abs(np.diff(raw_goals,axis=0)))<=2
        residual=np.asarray(rows[0]['actual'])-start;q=ticks_to_sim(raw_goals+residual);grid=np.arange(len(q))*.075
    times=np.arange(0,grid[-1]+dt/2,dt);poses=np.column_stack([np.interp(times,grid,q[:,j]) for j in range(16)]);vel=np.gradient(poses,dt,axis=0)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,output/'scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6;m.opt.timestep=dt
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id;R=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm].copy();up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(up@x);x/=np.linalg.norm(x);y=np.cross(up,x);frame=np.column_stack([x,y,up]);cubeR=np.column_stack([x,-up,np.cross(x,-up)])
    quat=Rotation.from_matrix(cubeR).as_quat()[[3,0,1,2]];m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);m.geom_friction[cube,0]=.3
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    if raw_goals is not None:
        static_self=static_support=0.;bad=[]
        for gi,pose in enumerate(q):
            probe.qpos[:16]=pose;probe.qpos[-7:-4]=[0,0,10];mujoco.mj_forward(m,probe)
            for c in probe.contact:
                a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if ha and hb:static_self=max(static_self,dep)
                if (a in support and hb) or (b in support and ha):static_support=max(static_support,dep)
                if dep>0 and ((ha and hb and dep>.1) or ((a in support and hb) or (b in support and ha)) and dep>.15):
                    bad.append(dict(goal_index=gi,pair=[m.geom(a).name,m.geom(b).name],depth_mm=dep))
            if bad:break
        static=dict(goals=len(raw_goals),nominal_seconds=float(grid[-1]),maximum_self_overlap_mm=static_self,maximum_support_overlap_mm=static_support,first_rejected=bad)
        (output/'hand_path_screen.json').write_text(json.dumps(static,indent=2)+'\n',encoding='utf-8');print('STATIC',json.dumps(static),flush=True)
        if bad:return
    def observe(d):
        names=set();overlap=0.;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:
                overlap=max(overlap,-float(c.dist)*1000)
                if c.dist<.0003:names.add(m.geom(other).name)
            elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return names,overlap,external
    def angles(d,R0):
        C=d.xmat[body].reshape(3,3);a=-R0[:,0];b=-C[:,0];a-=up*(up@a);b-=up*(up@b)
        a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        return float(np.degrees(np.arctan2(up@np.cross(a,b),a@b))),float(np.degrees(np.arccos(np.clip((-C[:,1])@up,-1,1))))
    results=[];started=time.perf_counter()
    for sx,sy in itertools.product((-40,-25,-10,5,20,35),(-15,0,15)):
        d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-4:]=quat;height=45.
        while True:
            d.qpos[-7:-4]=origin+frame@(np.array([sx,sy,height])*.001);mujoco.mj_forward(m,d)
            if observe(d)[1]<=.05 or height>=95:break
            height+=.5
        initial=d.qpos[-7:].copy()
        for _ in range(500):d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);R0=d.xmat[body].reshape(3,3).copy();names,cp,ext=observe(d);_,green=angles(d,R0)
        valid=bool(names and green<=args.max_initial_tilt and not ext and np.linalg.norm(d.qpos[-7:-4]-initial[:3])<=.015)
        row=dict(x_mm=sx,y_mm=sy,height_mm=height,initial_valid=valid,initial_green_up_error_deg=green,friction=.3)
        if valid:
            settled=d.qpos[-7:].copy();maximum_cube=gap=maxgap=0.;external=False;best=None;last_branch=-10.;stop=None
            for i,t in enumerate(times):
                d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
                if i%10==0:
                    names,cp,ext=observe(d);maximum_cube=max(maximum_cube,cp);external|=ext;gap=0 if names else gap+.02;maxgap=max(maxgap,gap)
                    yaw,green=angles(d,R0)
                    score=abs(abs(yaw)-90)+2*green
                    if best is None or score<best['score']:best=dict(time_s=float(t),yaw_deg=yaw,green_up_error_deg=green,score=score)
                    if 70<=abs(yaw)<=110 and green<=18 and t-last_branch>2 and names and not external:
                        branch=mujoco.MjData(m);branch.qpos[:]=d.qpos;branch.qvel[:]=d.qvel;branch.ctrl[:]=poses[i];branch.time=d.time
                        for _ in range(1000):branch.qpos[:16]=poses[i];branch.qvel[:16]=0;branch.ctrl[:]=poses[i];mujoco.mj_step(m,branch)
                        mujoco.mj_forward(m,branch);byaw,bgreen=angles(branch,R0);bn,bcp,bext=observe(branch);last_branch=float(t)
                        if abs(abs(byaw)-90)<=15 and bgreen<=15 and bn and not bext and bcp<=1 and maximum_cube<=1 and maxgap<=.1:
                            row['promising_prefix']=dict(stop_time_s=float(t),yaw_after_hold_deg=byaw,green_up_after_hold_error_deg=bgreen,
                                cube_center_translation_mm=float(1000*np.linalg.norm(branch.qpos[-7:-4]-settled[:3])),final_contacts=sorted(bn))
                            np.savez_compressed(output/f'promising_x{sx}_y{sy}.npz',qpos_at_stop=d.qpos.copy(),qvel_at_stop=d.qvel.copy(),hand_angles=poses[i],settled_start=settled)
                            break
                    if external or maxgap>.3:stop='cube_lost_or_external_contact';break
                mujoco.mj_step(m,d)
            row.update(best=best,maximum_cube_overlap_mm=maximum_cube,maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external,stopped=stop,
                final_cube_qpos=d.qpos[-7:].tolist(),final_contact_geoms=sorted(observe(d)[0]))
        results.append(row);print('SEATING',json.dumps(row),flush=True)
        report=dict(status='same_measured_motion_yaw_seating_diagnostic',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),model=manifest,cases=results,
            actual_cube_placement_not_registered=True,friction_not_measured=True,hand_response_prescribed_from_other_seating=True,
            source_motion_seconds=float(grid[-1]),hardware_execution_allowed=False,real_devices_opened=False,elapsed_seconds=time.perf_counter()-started)
        report['initial_up_error_limit_deg']=args.max_initial_tilt
        report['motion_source']=args.goal_path if args.goal_path else ('original_recorded_yaw_stages_modified_only_to_hold_ring' if args.original_yaw=='ring_fixed' else ('original_recorded_yaw_stages' if args.original_yaw else 'measured_successful_vertical_reference'))
        (output/'seating_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(results),promising=sum('promising_prefix' in v for v in results)),flush=True)
if __name__=='__main__':main()
