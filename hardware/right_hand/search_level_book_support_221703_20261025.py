"""OFFLINE initial seating consistent with a level book and two support links."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json,hashlib,argparse
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating';OUT.mkdir(exist_ok=True)
def rot(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
def quat(R):return Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--s',nargs='+',type=int,default=[20,28,36,44,52]);ap.add_argument('--y',nargs='+',type=int,default=[10,15,20]);ap.add_argument('--finish-extra',type=int,default=0);ap.add_argument('--timestep',type=float,default=.002);ap.add_argument('--contact-time',type=float);ap.add_argument('--friction',type=float,default=.3);ap.add_argument('--source',default='loaded_quarter_trial_20261025_171607_019752/result.json');ap.add_argument('--motion-phases',nargs='+',default=['stage_6_loaded_quarter_turn','stage_7_loaded_observation','stage_2_fixed_hold']);ap.add_argument('--output-tag');ap.add_argument('--allow-fixed-root-support',action='store_true');a=ap.parse_args()
    dest=OUT if a.finish_extra==0 and a.timestep==.002 and a.contact_time is None and a.friction==.3 else OUT/f'whole_finish{a.finish_extra}_dt{a.timestep:.4f}_tc{a.contact_time}_mu{a.friction}'
    if a.output_tag:
        if Path(a.output_tag).name!=a.output_tag:raise ValueError('Output tag must be a single folder name')
        dest=OUT/a.output_tag
    dest.mkdir(exist_ok=True)
    source=ROOT/a.source;r=json.loads(source.read_bytes())
    rows=[s for s in r['samples'] if s['phase'] in a.motion_phases]
    raw=np.array([s['actual'] for s in rows]);q=ticks_to_sim(raw);grid=np.array([s['t'] for s in rows]);grid-=grid[0]
    if a.finish_extra:
        delta=np.zeros(16);delta[[7,11]]=a.finish_extra;extra=ticks_to_sim(delta)-ticks_to_sim(np.zeros(16));last=q[-1]+extra;duration=a.finish_extra/2*.05
        q=np.vstack([q,last,last]);grid=np.r_[grid,grid[-1]+duration,grid[-1]+duration+3]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,dest/'seating_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6;m.opt.timestep=a.timestep;m.geom_friction[m.geom('cube').id,0]=a.friction
    if a.contact_time is not None:
        if a.contact_time<2*a.timestep:raise ValueError('Contact time must be at least twice the time step')
        mask=m.geom_solref[:,0]>0;m.geom_solref[mask,0]=np.minimum(m.geom_solref[mask,0],a.contact_time)
    probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id;R=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm].copy();up=-R[:,2];up=up if up[2]>0 else -up;x=R[:,0];x-=up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x);frame=np.column_stack([x,y,up]);cubeR=np.column_stack([x,-up,np.cross(x,-up)])
    originalG=m.opt.gravity.copy();m.opt.gravity[:]=-np.linalg.norm(originalG)*up;cube=m.geom('cube').id
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def contact(d):
        names=set();depth=0;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:
                depth=max(depth,-float(c.dist)*1000)
                if c.dist<.0003:names.add(m.geom(other).name)
            elif c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return names,depth,external
    step=float(m.opt.timestep);times=np.arange(0,grid[-1]+step/2,step);poses=np.column_stack([np.interp(times,grid,q[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0);cases=[]
    for s,yy in itertools.product(a.s,a.y):
        d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];height=45.
        d.qpos[-7:-4]=origin+frame@(np.array([s,yy,height])*.001);d.qpos[-4:]=quat(cubeR);mujoco.mj_forward(m,d)
        while contact(d)[1]>.05 and height<95:
            height+=.5;d.qpos[-7:-4]=origin+frame@(np.array([s,yy,height])*.001);mujoco.mj_forward(m,d)
        start=d.qpos[-7:].copy()
        for _ in range(round(1/step)):d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();names,depth,external=contact(d);green=-rot(settled[-4:])[:,1];green_error=float(np.degrees(np.arccos(np.clip(green@up,-1,1))))
        both=any(n.startswith('if_') for n in names) and any(n.startswith('mf_') for n in names)
        if a.allow_fixed_root_support:
            both=bool(names) and any(n.startswith(('if_','mf_','palm_collision_')) for n in names)
        valid=bool(both and green_error<=12 and not external and np.linalg.norm(settled[:3]-start[:3])<=.012)
        row=dict(book_frame_position_mm=[s,yy,height],initial_supports=sorted(names),settled_green_up_error_deg=green_error,
            settle_shift_mm=float(np.linalg.norm(settled[:3]-start[:3])*1000),settled_cube_qpos=settled.tolist(),valid_two_link_upright_start=valid)
        if valid:
            target=target_quaternion(settled[-4:]);errors=[];gap=maxgap=depthmax=0.;external=False;trace=[];endnames=set()
            for i,t in enumerate(times):
                d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d);names,depth,other=contact(d);depthmax=max(depthmax,depth);external|=other;gap=0 if names else gap+step;maxgap=max(maxgap,gap);endnames=names
                errors.append(float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1)))));trace.append(d.qpos.copy());mujoco.mj_step(m,d)
            trace=np.array(trace);errors=np.array(errors);Rf=rot(trace[-1,-4:]);red=float(np.degrees(np.arccos(np.clip(Rf[:,0]@up,-1,1))));green=float(np.degrees(np.arccos(np.clip((-Rf[:,1])@(-x),-1,1))));hold=float(errors[times>=times[-1]-3].max())
            row.update(final_target_error_deg=float(errors[-1]),hold_error_deg=hold,red_up_error_deg=red,green_wrist_error_deg=green,
                external_cube_contact=external,max_no_hand_contact_s=maxgap,max_cube_overlap_mm=depthmax,final_cube_supports=sorted(endnames),
                orientation_retention_pass=bool(errors[-1]<=20 and hold<=20 and red<=20 and green<=20 and endnames and maxgap<=.1 and not external),
                final_cube_qpos=trace[-1,-7:].tolist())
            row['candidate_accepted']=bool(row['orientation_retention_pass'] and depthmax<=1)
            np.savez_compressed(dest/f'case_s{s}_y{yy}.npz',times=times,qpos=trace,goal_errors_deg=errors,settled_cube=settled)
        cases.append(row);print('CASE',json.dumps(row),flush=True)
    result=dict(model=manifest,cases=cases,source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        book_frame_level_is_a_setup_hypothesis=True,original_gravity_world=originalG.tolist(),diagnostic_gravity_world=m.opt.gravity.tolist(),
        measured_loaded_joint_path_prescribed=True,infinite_effective_hand_inertia_assumption=True,physical_friction_identified=False,
        hypothetical_contact_time=a.contact_time,contact_parameters_identified=False,finish_extra_root_ticks=a.finish_extra,cube_friction=a.friction,timestep=a.timestep,
        fixed_root_support_allowed_in_initial_diagnostic=a.allow_fixed_root_support,
        actual_cube_position_in_hand_not_registered=True,hardware_execution_allowed=False,hardware_commands_sent=0)
    (dest/'seating_grid.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('SUMMARY',sum(c['valid_two_link_upright_start'] for c in cases),sum(c.get('orientation_retention_pass',False) for c in cases),flush=True)
if __name__=='__main__':main()
