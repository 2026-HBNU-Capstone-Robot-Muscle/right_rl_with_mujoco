"""Offline surface-specific traction hypotheses, not physical friction fitting."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,datetime,hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--timestep',type=float);parser.add_argument('--scopes',default='baseline,four_tips,index_middle_links,tips_and_links');parser.add_argument('--x-offsets',default='0,-4,4');parser.add_argument('--convergence',action='store_true');parser.add_argument('--book-level-gravity',action='store_true');args=parser.parse_args()
    if args.timestep is not None:OUT=OUT/f'timestep_{args.timestep:.4f}'
    if args.book_level_gravity:OUT=OUT/'book_level_gravity'
    OUT.mkdir(exist_ok=True,parents=True)
    source=ROOT/'loaded_quarter_trial_20261025_171607_019752/result.json';report=json.loads(source.read_bytes())
    rows=[s for s in report['samples'] if s['phase'] in ('stage_6_loaded_quarter_turn','stage_7_loaded_observation','stage_2_fixed_hold')]
    raw=np.array([s['actual'] for s in rows]);q=ticks_to_sim(raw);grid=np.array([s['t'] for s in rows]);grid-=grid[0]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'surface_comparison_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6
    original_step=float(m.opt.timestep)
    original_gravity=m.opt.gravity.copy();geometry_probe=mujoco.MjData(m);mujoco.mj_forward(m,geometry_probe)
    book_up=geometry_probe.geom_xmat[m.geom('support_book').id].reshape(3,3)[:,2].copy()
    if book_up@np.array([0,0,1])<0:book_up=-book_up
    book_tilt_deg=float(np.degrees(np.arccos(np.clip(book_up@np.array([0,0,1]),-1,1))))
    if args.book_level_gravity:m.opt.gravity[:]=-np.linalg.norm(original_gravity)*book_up
    if args.timestep is not None:
        if not 0<args.timestep<=original_step:raise ValueError('Convergence step must not exceed original step')
        m.opt.timestep=args.timestep
    cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    tips=['if_tip','mf_tip','rf_tip','th_tip'];links=['if_px_collision','mf_px_collision']
    baseline=m.geom_friction.copy();all_scopes={'baseline':[],'four_tips':tips,'index_middle_links':links,'tips_and_links':tips+links}
    scopes={name:all_scopes[name] for name in args.scopes.split(',')}
    ref=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_clearance/reverse_original_native_verification.json').read_bytes())
    initial=np.array(next(x for x in ref['runs'] if x.get('mode')=='moving_path' and x['gain_scale']==1 and x['friction']==.8 and x['offset_from_nominal_mm']==[0,0])['initial_cube_qpos'])
    step=float(m.opt.timestep);times=np.arange(0,grid[-1]+step/2,step)
    poses=np.column_stack([np.interp(times,grid,q[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0)
    def contacts(d):
        selfdep=supportdep=cubedep=0.;touch=False;external=False;names=set();points=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if c.dist<0 and ha and hb:selfdep=max(selfdep,-float(c.dist)*1000)
            if c.dist<0 and (a in supports and hb or b in supports and ha):supportdep=max(supportdep,-float(c.dist)*1000)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cubedep=max(cubedep,-float(c.dist)*1000)
                    if c.dist<.0003:
                        touch=True;name=m.geom(other).name;names.add(name)
                        point=d.geom_xmat[other].reshape(3,3).T@(c.pos-d.geom_xpos[other]);points.append((name,point.tolist()))
                elif c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return selfdep,supportdep,cubedep,touch,external,names,points
    cases=[]
    for scope,names in scopes.items():
      m.geom_friction[:]=baseline
      for name in names:m.geom_friction[m.geom(name).id,0]=.8
      offsets=[(int(x),0) for x in args.x_offsets.split(',')] if args.convergence else [(0,0),(-4,0),(4,0),(0,-4),(0,4)]
      for dx,dy in offsets:
        d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:]=initial;d.qpos[-7]+=dx*.001;d.qpos[-6]+=dy*.001;mujoco.mj_forward(m,d);lift=0
        while contacts(d)[2]>.05 and lift<30:d.qpos[-5]+=.001;lift+=1;mujoco.mj_forward(m,d)
        if contacts(d)[2]>.05:raise RuntimeError('Invalid cube initialization')
        for _ in range(round(1/step)):d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();target=target_quaternion(settled[-4:]);errors=[];maxself=maxsupport=maxdepth=gap=maxgap=0.;external=False;endtouch=False;touch_names=set();point_rows=[]
        for i,t in enumerate(times):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            a,b,c,touch,other,touched,points=contacts(d);maxself=max(maxself,a);maxsupport=max(maxsupport,b);maxdepth=max(maxdepth,c);external|=other;touch_names.update(touched)
            gap=0 if touch else gap+step;maxgap=max(maxgap,gap);endtouch=touch
            error=float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1))));errors.append(error)
            if i%10==0:point_rows.extend(dict(t=float(t),geom=name,point_local_mm=(np.array(p)*1000).tolist()) for name,p in points)
            mujoco.mj_step(m,d)
        err=np.array(errors);hold=float(err[times>=times[-1]-3].max());eligible=bool(err[-1]<=15 and hold<=20 and not external and endtouch and maxgap<=.1 and maxself<=1e-6 and maxsupport<=.15)
        case=dict(scope=scope,offset_xy_mm=[dx,dy],final_target_error_deg=float(err[-1]),hold_error_deg=hold,external_cube_contact=external,
            max_no_hand_contact_s=maxgap,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,max_cube_overlap_mm=maxdepth,
            hand_contact_at_end=bool(endtouch),eligible_diagnostic_case=eligible,contact_geoms=sorted(touch_names),initial_lift_mm=lift,
            modified_friction_geoms=names,initial_settled_cube_qpos=settled.tolist(),final_cube_qpos=d.qpos[-7:].tolist())
        case['final_snapshot_contact_penetration_mm']=float(contacts(d)[2]);cases.append(case)
        (OUT/f'contacts_{scope}_x{dx}_y{dy}.json').write_text(json.dumps(point_rows),encoding='utf-8')
        print('CASE',json.dumps({k:v for k,v in case.items() if k not in ('contact_geoms','initial_settled_cube_qpos','final_cube_qpos')}),flush=True)
    summary={name:dict(cases=sum(c['scope']==name for c in cases),eligible=sum(c['scope']==name and c['eligible_diagnostic_case'] for c in cases)) for name in scopes}
    result=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model=manifest,
        scopes=scopes,cases=cases,summary=summary,original_frictions={n:baseline[m.geom(n).id].tolist() for n in ['cube']+tips+links},
        geom_priorities={n:int(m.geom_priority[m.geom(n).id]) for n in ['cube']+tips+links},explicit_contact_pairs=int(m.npair),
        original_simulator_timestep=original_step,diagnostic_timestep=float(m.opt.timestep),
        original_gravity_world=original_gravity.tolist(),diagnostic_gravity_world=m.opt.gravity.tolist(),book_top_up_normal_model_world=book_up.tolist(),
        native_book_plane_tilt_from_native_horizontal_deg=book_tilt_deg,book_level_gravity_is_a_physical_setup_hypothesis=bool(args.book_level_gravity),
        contact_solref={n:m.geom_solref[m.geom(n).id].tolist() for n in ['cube']+tips+links},
        user_confirmed_contact_material='rigid plastic',physical_friction_identified=False,initial_cube_pose_registered=False,
        prescribed_measured_loaded_hand_path=True,infinite_effective_hand_inertia_assumption=True,
        full_geom_friction_change_is_only_a_patch_upper_bound=True,patch_thickness_geometry_not_included=True,
        hardware_commands_sent=0,hardware_execution_allowed=False,physical_rotation_verified=False)
    (OUT/'surface_friction_comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(summary),flush=True)
if __name__=='__main__':main()
