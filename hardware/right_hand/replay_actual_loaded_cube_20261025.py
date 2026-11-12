"""OFFLINE only: measured loaded joint path and uncertain free-cube seating.

Prescribed hand positions use effectively infinite hand inertia. This compares
contact hypotheses and does not identify loaded actuator dynamics or authorize
new targets on the robot.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,hashlib,json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'loaded_quarter_trial_20261025_171607_019752/result.json'
OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607';OUT.mkdir(exist_ok=True)
def rot(q):return Rotation.from_quat(q[[1,2,3,0]])
def quat(R):return R.as_quat()[[3,0,1,2]]
def green_tilt(q):return float(np.degrees(np.arccos(np.clip(-(rot(q).as_matrix()[2,1]),-1,1))))
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--finish-sweep',action='store_true');parser.add_argument('--stalled-finish-sweep',action='store_true');args=parser.parse_args()
    r=json.loads(SOURCE.read_bytes());rows=[s for s in r['samples'] if s['phase'] in ('stage_6_loaded_quarter_turn','stage_7_loaded_observation','stage_2_fixed_hold')]
    raw=np.asarray([s['actual'] for s in rows]);q=ticks_to_sim(raw);times=np.asarray([s['t'] for s in rows]);times-=times[0]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'actual_loaded_contact_hypothesis_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')};cube=m.geom('cube').id
    original_friction=m.geom_friction[cube].copy()
    if args.finish_sweep:m.geom_friction[cube,0]=.8
    def contacts(d):
        selfdepth=supportdepth=cubedepth=0.;touch=False;external=False;names=set()
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if c.dist<0 and ha and hb:selfdepth=max(selfdepth,-float(c.dist)*1000)
            if c.dist<0 and (a in supports and hb or b in supports and ha):supportdepth=max(supportdepth,-float(c.dist)*1000)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cubedepth=max(cubedepth,-float(c.dist)*1000)
                    if c.dist<.0003:touch=True;names.add(m.geom(other).name)
                if c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return selfdepth,supportdepth,cubedepth,touch,external,sorted(names)
    ref=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_clearance/reverse_original_native_verification.json').read_bytes())
    initial=np.array(next(x for x in ref['runs'] if x.get('mode')=='moving_path' and x['gain_scale']==1 and x['friction']==.8 and x['offset_from_nominal_mm']==[0,0])['initial_cube_qpos'])
    upright=initial.copy();R=rot(initial[-4:]).as_matrix();axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);y=np.array([0.,0.,-1.]);upright[-4:]=quat(Rotation.from_matrix(np.column_stack([axis,y,np.cross(axis,y)])))
    step=float(m.opt.timestep);cases=[]
    experiments=[(orientation,dx,dy,0) for orientation in ('prior_model','green_upright') for dx,dy in ((0,0),(-4,0),(4,0),(0,-4),(0,4))]
    experiments += [('green_upright',0,0,n) for n in (64,128)]
    if args.finish_sweep:
        experiments=[('prior_model',dx,0,0) for dx in (-4,0,4)]
        experiments += [('prior_model',-4,0,n) for n in (64,128,256,384)]
        experiments += [('prior_model',dx,0,128) for dx in (0,4)]
    if args.stalled_finish_sweep:
        experiments=[('prior_model',-4,0,n) for n in (0,64,128,256,384,512,640)]
        experiments += [('prior_model',0,0,n) for n in (256,512)]
    for orientation,dx,dy,extra in experiments:
        # Candidate suffix is a hypothesis only, not a production goal file.
        trail=np.vstack([q,q[-1]+ticks_to_sim(raw[-1]+np.array([extra if i in (7,11) else 0 for i in range(16)]))-q[-1]]) if extra else q
        # ticks_to_sim applies an offset; compute the delta in joint space.
        if extra:
            delta=ticks_to_sim(raw[-1]+np.array([extra if i in (7,11) else 0 for i in range(16)]))-ticks_to_sim(raw[-1])
            trail=np.vstack([q,q[-1]+delta,q[-1]+delta]);gridtimes=np.r_[times,times[-1]+extra*.05,times[-1]+extra*.05+3]
        else:gridtimes=times
        path_times=np.arange(0,gridtimes[-1]+step/2,step);poses=np.column_stack([np.interp(path_times,gridtimes,trail[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0)
        d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:]=initial if orientation=='prior_model' else upright;d.qpos[-7]+=dx*.001;d.qpos[-6]+=dy*.001
        mujoco.mj_forward(m,d);lift=0
        while contacts(d)[2]>.05 and lift<30:d.qpos[-5]+=.001;lift+=1;mujoco.mj_forward(m,d)
        if contacts(d)[2]>.05:raise RuntimeError('Initial cube penetration unresolved')
        for _ in range(round(1/step)):d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();target=target_quaternion(settled[-4:]);startR=rot(settled[-4:]);snapshots=[];maxself=maxsupport=gap=maxgap=0.;external=False;endnames=[];errors=[];trace=[]
        for i,t in enumerate(path_times):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            selfdepth,supportdepth,_,touch,other,names=contacts(d);maxself=max(maxself,selfdepth);maxsupport=max(maxsupport,supportdepth);external|=other;gap=0 if touch else gap+step;maxgap=max(maxgap,gap);endnames=names
            error=float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1))));errors.append(error)
            if not snapshots or t-snapshots[-1]['t']>=4.99:snapshots.append(dict(t=float(t),rotation_deg=float((rot(d.qpos[-4:])*startR.inv()).magnitude()*180/np.pi),green_tilt_world_deg=green_tilt(d.qpos[-4:])))
            trace.append(d.qpos.copy());mujoco.mj_step(m,d)
        trace=np.asarray(trace);errors=np.asarray(errors);last=trace[-1,-4:]
        row=dict(initial_orientation=orientation,offset_xy_mm=[dx,dy],hypothetical_finish_extra_root_ticks=extra,start_lift_mm=lift,
            initial_green_tilt_deg=green_tilt(initial[-4:] if orientation=='prior_model' else upright[-4:]),settled_green_tilt_deg=green_tilt(settled[-4:]),
            final_rotation_from_settled_deg=float((rot(last)*startR.inv()).magnitude()*180/np.pi),final_target_error_deg=float(errors[-1]),
            last_three_seconds_maximum_target_error_deg=float(errors[path_times>=path_times[-1]-3].max()),maximum_hand_self_overlap_mm=maxself,
            maximum_hand_support_overlap_mm=maxsupport,maximum_no_hand_contact_s=maxgap,external_cube_contact=external,end_cube_contacts=endnames,timeline=snapshots,
            settled_cube_qpos=settled.tolist(),final_cube_qpos=trace[-1,-7:].tolist())
        cases.append(row);print('CASE',json.dumps({k:v for k,v in row.items() if k not in ('timeline','settled_cube_qpos','final_cube_qpos')}),flush=True)
        if args.finish_sweep or args.stalled_finish_sweep or dx==dy==0:
            suffix=f'_dx{dx}_stalled_finish_v3' if args.stalled_finish_sweep else f'_dx{dx}_finish_v2' if args.finish_sweep else ''
            np.savez_compressed(OUT/f'loaded_measured_{orientation}_extra{extra}{suffix}.npz',qpos=trace,times=path_times,goal_errors_deg=errors)
    result=dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),model=manifest,cases=cases,
        loaded_joint_positions_measured=True,initial_cube_pose_not_registered=True,prescribed_hand_infinite_inertia_assumption=True,
        diagnostic_only=True,physical_rotation_verified=False,new_hardware_execution_allowed=False,hardware_commands_sent=0,
        original_cube_friction=original_friction.tolist(),diagnostic_cube_friction=m.geom_friction[cube].tolist(),friction_not_physically_identified=True)
    name='actual_loaded_stalled_finish_candidates_v3.json' if args.stalled_finish_sweep else 'actual_loaded_finish_candidates_v2.json' if args.finish_sweep else 'actual_loaded_contact_hypotheses.json'
    (OUT/name).write_text(json.dumps(result,indent=2),encoding='utf-8')
if __name__=='__main__':main()
