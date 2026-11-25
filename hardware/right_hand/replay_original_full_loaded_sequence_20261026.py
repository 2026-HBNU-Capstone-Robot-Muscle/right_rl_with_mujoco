"""Full recorded seven-pose sequence from its original loaded pose, offline.

Unlike partial-yaw and after-flip studies, the cube remains present from P0.
No recorded source, native range or actual execution program is modified.
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
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/original_full_loaded_v1'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--slow-two-tick',action='store_true');parser.add_argument('--forward-seating',action='store_true');args=parser.parse_args();out=OUT/(('slow' if args.slow_two_tick else 'five_seconds_per_stage')+('_forward' if args.forward_seating else ''));out.mkdir(parents=True,exist_ok=True);raw=ROOT/'sequence_flip_poses.json';record=json.loads(raw.read_bytes());poses=np.asarray([p['raw_ticks'] for p in record['poses']]);lo,hi=limits();dt=.002
    starts=[0.];times=[0.];goals=[poses[0]];stage=[]
    for si,(a,b) in enumerate(zip(poses[:-1],poses[1:])):
        n=max(1,int(np.ceil(np.max(abs(b-a))/2))) if args.slow_two_tick else 70;period=.075 if args.slow_two_tick else 5./n
        for k in range(1,n+1):goals.append(np.rint(a+(b-a)*k/n).astype(int));times.append(times[-1]+period);stage.append(si+1)
        starts.append(times[-1])
    goals=np.asarray(goals);times=np.asarray(times);native=bool(np.all(goals>=lo) and np.all(goals<=hi));cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,out/'scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=dt;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id;P=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm].copy();up=-P[:,2];front=P[:,0];left=np.cross(up,front);frame=np.column_stack([front,left,up]);R0=np.column_stack([front,-up,np.cross(front,-up)]);quat=Rotation.from_matrix(R0).as_quat()[[3,0,1,2]];cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);m.opt.gravity[:]=-9.81*up;m.geom_friction[cube,0]=.3;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        sp=sup=cp=0.;contacts=set();external=False;pairs=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:
                sp=max(sp,dep)
                if dep>.1:pairs.append([m.geom(a).name,m.geom(b).name,dep])
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,dep)
                    if c.dist<.0003:contacts.add(m.geom(other).name)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return dict(self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=cp,contacts=sorted(contacts),external_cube_contact=external,deep_self_pairs=pairs)
    def angles(d):
        C=d.xmat[cb].reshape(3,3);green=-C[:,1];orange=-C[:,0];heading=orange-up*(orange@up);heading/=max(1e-12,np.linalg.norm(heading));start=-front
        return float(np.degrees(np.arctan2(up@np.cross(start,heading),start@heading))),float(np.degrees(np.arccos(np.clip(green@up,-1,1))))
    grid=np.arange(0,times[-1]+2,dt);rad=ticks_to_sim(goals);q=np.column_stack([np.interp(grid,times,rad[:,j]) for j in range(16)]);velocity=np.gradient(q,dt,axis=0)
    static_max=0.;static_bad=None
    for i,g in enumerate(goals):
        probe.qpos[:16]=ticks_to_sim(g);probe.qpos[-7:-4]=[0,0,10];mujoco.mj_forward(m,probe);o=observe(probe);static_max=max(static_max,o['self_overlap_mm'])
        if static_bad is None and (o['self_overlap_mm']>.1 or o['support_overlap_mm']>.15):static_bad=dict(sample=i,stage=int(np.searchsorted(starts,times[i])),time_s=float(times[i]),observation=o)
    print('STATIC',json.dumps(dict(native_goals=native,maximum_self_overlap_mm=static_max,first_bad=static_bad)),flush=True)
    results=[];began=time.perf_counter()
    for sx,sy in itertools.product((15.,30.,45.,60.) if args.forward_seating else (-30.,-15.,0.,15.),(-15.,0.,15.)):
        d=mujoco.MjData(m);d.qpos[:16]=rad[0];d.ctrl[:]=rad[0];d.qpos[-4:]=quat;height=45.
        while height<145:
            d.qpos[-7:-4]=origin+frame@np.array([sx,sy,height])*.001;mujoco.mj_forward(m,d)
            if observe(d)['cube_overlap_mm']<=.05:break
            height+=.5
        original=d.qpos[-7:].copy()
        for _ in range(500):d.qpos[:16]=rad[0];d.qvel[:16]=0;d.ctrl[:]=rad[0];mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);o=observe(d);_,tilt=angles(d);valid=bool(o['contacts'] and not o['external_cube_contact'] and o['cube_overlap_mm']<=1 and tilt<15 and np.linalg.norm(d.qpos[-7:-4]-original[:3])<.015);row=dict(x_mm=sx,y_mm=sy,height_mm=height,initial_valid=valid,initial_green_up_error_deg=tilt,initial_cube_overlap_mm=o['cube_overlap_mm'])
        if valid:
            trace=[];maxcp=maxsp=maxsup=gap=maxgap=0.;best=None;stop=None;stage_poses=[];next_stage=1;best_state=None
            for i,t in enumerate(grid):
                d.qpos[:16]=q[i];d.qvel[:16]=velocity[i];d.ctrl[:]=q[i];mujoco.mj_forward(m,d)
                if i%10==0:
                    o=observe(d);yaw,tilt=angles(d);maxcp=max(maxcp,o['cube_overlap_mm']);maxsp=max(maxsp,o['self_overlap_mm']);maxsup=max(maxsup,o['support_overlap_mm']);gap=0 if o['contacts'] else gap+.02;maxgap=max(maxgap,gap)
                    if best is None or abs(abs(yaw)-90)+2*tilt<best['score']:best=dict(time_s=float(t),yaw_deg=yaw,green_up_error_deg=tilt,score=abs(abs(yaw)-90)+2*tilt)
                    if next_stage<len(starts) and t>=starts[next_stage]:stage_poses.append(dict(stage=next_stage,time_s=float(t),yaw_deg=yaw,green_up_error_deg=tilt,contacts=o['contacts']));next_stage+=1
                    if abs(abs(yaw)-90)<=12 and tilt<=12 and maxcp<=1 and maxsp<=.1 and maxsup<=.15 and maxgap<=.1:
                        b=mujoco.MjData(m);b.qpos[:]=d.qpos;b.qvel[:]=d.qvel
                        for _ in range(1000):b.qpos[:16]=q[i];b.qvel[:16]=0;b.ctrl[:]=q[i];mujoco.mj_step(m,b)
                        mujoco.mj_forward(m,b);by,bt=angles(b);bo=observe(b)
                        if abs(abs(by)-90)<=15 and bt<=15 and bo['contacts'] and bo['cube_overlap_mm']<=1 and not bo['external_cube_contact']:
                            row['conditional_yaw_prefix']=dict(stop_time_s=float(t),yaw_after_hold=by,green_up_after_hold=bt,contacts=bo['contacts']);best_state=b.qpos.copy();break
                    if o['external_cube_contact'] or maxgap>.3:stop='cube_lost';break
                    if maxcp>1:stop='cube_contact_model_rejected';break
                if i%50==0:trace.append(d.qpos.copy())
                mujoco.mj_step(m,d)
            row.update(best=best,stopped=stop,max_cube_overlap_mm=maxcp,max_self_overlap_mm=maxsp,max_support_overlap_mm=maxsup,max_no_hand_contact_s=maxgap,stage_states=stage_poses,final_cube_qpos=d.qpos[-7:].tolist())
            np.savez_compressed(out/f'trace_x{sx:g}_y{sy:g}.npz',qpos=np.asarray(trace),end_qpos=d.qpos.copy(),promising_state=np.asarray([]) if best_state is None else best_state,dt_trace=.1)
        results.append(row);print('CASE',json.dumps(row),flush=True)
        report=dict(status='original_full_sequence_loaded_from_pose0',raw_source_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),cube_present_from_original_pose0=True,mode=out.name,native_goals=native,path_seconds=float(times[-1]),static_max_self_overlap_mm=static_max,static_first_rejected=static_bad,cases=results,friction_assumed=.3,initial_seating_unregistered=True,hand_trajectory_prescribed=True,hardware_execution_allowed=False,real_devices_opened=False,elapsed_wall_seconds=time.perf_counter()-began)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(results),conditional_yaw=sum('conditional_yaw_prefix' in v for v in results)),flush=True)
if __name__=='__main__':main()
