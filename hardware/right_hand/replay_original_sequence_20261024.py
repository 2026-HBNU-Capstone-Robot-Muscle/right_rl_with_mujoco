"""Free-cube diagnostic of original raw-ID sequence, without robot access.

Both prescribed hand motion and native actuator responses are exposed. Cube
placement, friction and motor physics are hypotheses, not fitted calibration.
No original pose is clipped and no runtime motor configuration is modified.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from review_original_sequence_20261024 import OUT, ROOT, load_source, make_models, save
from hand_motor_mapping import hw_to_sim, TICK


def setup():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    model,_,manifest,_=make_models(env)
    return model,manifest


def placement_cases(model):
    old=json.loads((ROOT/'sac_cube_rim_register_1010_stage0_result.json').read_bytes())
    center=np.array(old['cube_center_world_m'])
    up=-model.opt.gravity/np.linalg.norm(model.opt.gravity)
    forward=np.array([1.,0,0]);forward-=up*np.dot(forward,up);forward/=np.linalg.norm(forward)
    right=np.cross(forward,up)
    quat=np.array(old['cube_quat_world_candidate'])
    return [dict(center_m=(center+forward*x*.001+right*y*.001+up*z*.001).tolist(),
        quat_wxyz=quat.tolist(),offset_mm=[x,y,z],initial_registration='historical approximate marker/rim registration')
        for x in (-6,0,6) for y in (-6,0,6) for z in (0,6,12)]


def simulate(model, hw, placement, friction=.8, mode='prescribed', schedule='original', record=False,
             stop_stage=6, final_hold_s=0.):
    d=mujoco.MjData(model)
    q=hw_to_sim(hw)
    d.qpos[:16]=q[0];d.ctrl[:]=q[0]
    d.qpos[-7:]=np.r_[placement['center_m'],placement['quat_wxyz']]
    cube=model.geom('cube').id;body=int(model.geom_bodyid[cube])
    model.geom_friction[cube,0]=friction
    mujoco.mj_forward(model,d)
    up=-model.opt.gravity/np.linalg.norm(model.opt.gravity)
    forward=np.array([1.,0,0]);forward-=up*np.dot(forward,up);forward/=np.linalg.norm(forward)
    right=np.cross(forward,up)
    axes={'yaw':up,'forward_flip':right,'side_flip':forward}
    max_overlap=0.
    for c in d.contact:
        if cube in (int(c.geom1),int(c.geom2)):
            max_overlap=max(max_overlap,-float(c.dist)*1000)
    frames=[];next_frame=0.;tracking=0.;states=[]
    def step(target,velocity):
        nonlocal next_frame,tracking
        d.ctrl[:]=target
        if mode=='prescribed':
            d.qpos[:16]=target;d.qvel[:16]=velocity
        mujoco.mj_step(model,d)
        tracking=max(tracking,float(np.max(abs(d.qpos[:16]-target))))
        if record and d.time>=next_frame:
            frames.append(dict(t=float(d.time),qpos=d.qpos.tolist()))
            next_frame=d.time+.125
    # Settling cannot count toward rotation, and its displacement is recorded.
    for _ in range(round(1.5/model.opt.timestep)):
        step(q[0],np.zeros(16))
    R0=d.xmat[body].reshape(3,3).copy();p0=d.qpos[-7:-4].copy()
    settle_shift=float(np.linalg.norm(p0-placement['center_m'])*1000)
    # Choose a stable projected original cube axis for heading; no Euler wrap.
    heading_index=int(np.argmax([np.linalg.norm(R0[:,j]-up*(up@R0[:,j])) for j in range(3)]))
    v0=R0[:,heading_index]-up*(up@R0[:,heading_index]);v0/=np.linalg.norm(v0)
    targets={f'{name}_{sign*90:+d}':Rotation.from_rotvec(axis*sign*np.pi/2).as_matrix()@R0
             for name,axis in axes.items() for sign in (-1,1)}
    def state(stage):
        R=d.xmat[body].reshape(3,3).copy();p=d.qpos[-7:-4].copy()
        rv=Rotation.from_matrix(R@R0.T).as_rotvec()
        h=R[:,heading_index]-up*(up@R[:,heading_index])
        heading=None if np.linalg.norm(h)<.05 else float(np.degrees(np.arctan2(up@np.cross(v0,h),v0@h)))
        touching=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b) or c.dist>.0003:continue
            other=b if a==cube else a
            touching.append(model.body(int(model.geom_bodyid[other])).name)
        hand_contacts=sorted(set(n for n in touching if n=='palm' or n.startswith(('th_','if_','mf_','rf_'))))
        return dict(stage=stage,time_s=float(d.time),cube_center_m=p.tolist(),cube_quat_wxyz=d.qpos[-4:].tolist(),
            change_angle_deg=float(np.degrees(np.linalg.norm(rv))),heading_change_deg=heading,
            target_errors_deg={name:float(np.degrees(Rotation.from_matrix(R@target.T).magnitude())) for name,target in targets.items()},
            displacement_mm=float(np.linalg.norm(p-p0)*1000),height_change_mm=float((p-p0)@up*1000),
            hand_contacts=hand_contacts,all_contact_bodies=sorted(set(touching)),
            cube_retained_model=bool(hand_contacts and np.linalg.norm(p-p0)<.10),
            tracking_error_rad=float(np.max(abs(d.qpos[:16]-d.ctrl))))
    states.append(state(0))
    for stage in range(1,stop_stage+1):
        previous=q[stage-1].copy() if mode=='prescribed' else d.qpos[:16].copy()
        # Mirrors original read_pos-at-stage-start semantics in actuator mode.
        duration=2.1 if schedule=='original' else max(2.1,np.max(abs(q[stage]-previous))/TICK/2*.05)
        steps=round(duration/model.opt.timestep)
        for k in range(1,steps+1):
            target=previous+(q[stage]-previous)*(k/steps)
            step(target,(q[stage]-previous)/duration)
        for _ in range(round(.3/model.opt.timestep)):step(q[stage],np.zeros(16))
        states.append(state(stage))
    hold_observations=[]
    for k in range(round(final_hold_s/model.opt.timestep)):
        step(q[stop_stage],np.zeros(16))
        if (k+1)%round(.25/model.opt.timestep)==0:
            hold_observations.append(state(f'hold_{(k+1)*model.opt.timestep:.2f}s'))
    if final_hold_s:
        states.append(state('final_hold'))
    final=states[-1]
    result=dict(placement=placement,friction=friction,hand_response_mode=mode,schedule=schedule,
        native_actuators_not_identified_hardware_motors=(mode=='native_actuators'),
        original_static_self_overlap_disqualifies_hardware_execution=True,
        initial_cube_overlap_mm=max_overlap,settling_displacement_mm=settle_shift,
        max_native_model_tracking_rad=tracking,stages=states,stop_stage=stop_stage,final_hold_s=final_hold_s,
        hold_observations=hold_observations,
        best_retained_90_degree_error_deg=min([min(s['target_errors_deg'].values()) for s in states[1:] if s['cube_retained_model']] or [180.]),
        final_retained=final['cube_retained_model'],finite=bool(np.isfinite(d.qpos).all()),
        hardware_commands_sent=0,physical_rotation_verified=False)
    if record:result['frames']=frames
    return result


def render(model, result):
    from PIL import Image,ImageDraw
    renderer=mujoco.Renderer(model,height=480,width=640)
    camera=mujoco.MjvCamera();camera.lookat[:]=[.10,0,.01];camera.distance=.40;camera.azimuth=145;camera.elevation=-32
    option=mujoco.MjvOption();option.geomgroup[4]=0
    d=mujoco.MjData(model);images=[]
    # Color the collision box visibly; no decorative cube driven to a target.
    model.geom_rgba[model.geom('cube').id]=[.12,.7,.25,1]
    for f in result['frames']:
        d.qpos[:]=f['qpos'];mujoco.mj_forward(model,d)
        renderer.update_scene(d,camera=camera,scene_option=option)
        image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
        draw.rectangle((0,0,640,42),fill=(20,20,20))
        draw.text((10,8),f'SIMULATION ONLY | Original seven poses | t={f["t"]:.1f}s',fill='white')
        draw.text((10,24),'Free cube dynamics; prescribed hand; uncalibrated contact',fill='white')
        images.append(image)
    images[0].save(OUT/'original_sequence_free_cube.gif',save_all=True,append_images=images[1:],duration=125,loop=0)
    indices=[min(range(len(result['frames'])),key=lambda i:abs(result['frames'][i]['t']-s['time_s'])) for s in result['stages']]
    sheet=Image.new('RGB',(640*3,480*3),(20,20,20))
    for stage,index in enumerate(indices):
        im=images[index].copy();draw=ImageDraw.Draw(im)
        s=result['stages'][stage]
        draw.text((10,450),f'Pose {stage}: model angle {s["change_angle_deg"]:.1f}deg; retained {s["cube_retained_model"]}',fill='white')
        sheet.paste(im,((stage%3)*640,(stage//3)*480))
    sheet.save(OUT/'free_cube_stage_comparison.jpg');renderer.close()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--search',action='store_true')
    parser.add_argument('--prefix',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    names,hw,source=load_source();model,manifest=setup()
    cases=placement_cases(model)
    if args.prefix:
        full=json.loads((OUT/'free_cube_replay.json').read_bytes())
        candidates=[]
        for i,baseline in enumerate(full['placement_search']):
            if baseline['initial_cube_overlap_mm']>.5 or baseline['settling_displacement_mm']>30:continue
            for stage in (1,2,3):
                run=simulate(model,hw,baseline['placement'],stop_stage=stage,final_hold_s=4.)
                final=run['stages'][-1]
                run['sustained_retention_during_hold']=all(s['cube_retained_model'] for s in run['hold_observations'])
                run['final_90_error_deg']=min(final['target_errors_deg'].values())
                candidates.append(run)
            if i%3==2:print('PREFIX_PLACEMENTS',i+1,'runs',len(candidates),flush=True)
        candidates.sort(key=lambda r:(not r['sustained_retention_during_hold'],r['final_90_error_deg']))
        rows=[]
        for friction in (.4,.8,1.2):
            for mode in ('prescribed','native_actuators'):
                best=candidates[0]
                row=simulate(model,hw,best['placement'],friction,mode,stop_stage=best['stop_stage'],final_hold_s=4.)
                rows.append(row)
                print('PREFIX_SENSITIVITY',friction,mode,'stage',best['stop_stage'],'retained',row['final_retained'],
                    '90_error',min(row['stages'][-1]['target_errors_deg'].values()),flush=True)
        save(OUT/'ninety_degree_prefix_review.json',dict(candidates=candidates,sensitivity=rows,
            original_joint_goals_unchanged=True,modified_only_stop_stage_and_hold=True,
            full_original_path_not_robust=True,hardware_execution_allowed=False,hardware_commands_sent=0))
    elif args.search:
        results=[]
        for i,p in enumerate(cases):
            run=simulate(model,hw,p)
            results.append(run)
            if i%3==2:print('PLACEMENTS',i+1,'best_retained_90_error',min(r['best_retained_90_degree_error_deg'] for r in results),flush=True)
        # No fitting or altered goals: rank only for diagnostic visualization.
        results.sort(key=lambda r:(r['initial_cube_overlap_mm']>.5,
            r['settling_displacement_mm']>30,not r['final_retained'],r['best_retained_90_degree_error_deg']))
        best=results[0]
        comparison=[]
        for friction in (.4,.8,1.2):
            for mode in ('prescribed','native_actuators'):
                run=simulate(model,hw,best['placement'],friction,mode)
                comparison.append(run)
                print('SENSITIVITY',friction,mode,'final_retained',run['final_retained'],'90_error',run['best_retained_90_degree_error_deg'],flush=True)
        save(OUT/'free_cube_replay.json',dict(source=source,mapping='user_confirmed_20261018_v2',
            cube_side_mm=55,cube_mass_g=100,support=manifest,placement_search=results,
            sensitivity=comparison,original_poses_and_native_ranges_unchanged=True,
            placement_calibration_verified=False,hardware_execution_allowed=False,
            physical_rotation_verified=False,hardware_commands_sent=0))
        best=simulate(model,hw,best['placement'],.8,'prescribed',record=True)
        save(OUT/'visualized_replay.json',best)
        render(model,best)
    else:
        saved=json.loads((OUT/'free_cube_replay.json').read_bytes())
        print(json.dumps(saved['sensitivity'],indent=2))


if __name__=='__main__':main()
