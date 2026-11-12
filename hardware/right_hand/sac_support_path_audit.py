"""Offline sampled path and hold audit; never exports motor commands."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_support_separation import SupportSeparation
ROOT=Path(__file__).resolve().parent

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);model,manifest=build(env)
    sep=SupportSeparation(model);data=mujoco.MjData(model)
    audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    q=np.r_[audit['joint_angles_sim_rad'],audit['cube_center_m'],audit['cube_quat_wxyz']]
    support={model.geom(n).id for n in ('support_glass','support_wood')}
    def hits():
        return [dict(a=model.geom(c.geom1).name,b=model.geom(c.geom2).name,penetration_mm=float(-c.dist*1000))
                for c in data.contact if c.dist < -1e-5]
    def support_hits():return [c for c in hits() if c['a'].startswith('support_') or c['b'].startswith('support_')]
    def lower_bound():return 1000*min(sep.lower_bound(data,g,o) for g in sep.vertices for o in support)
    original=q.copy();candidate=q.copy();candidate[[0,4,8]]=np.maximum(candidate[[0,4,8]],.65)
    images=[];poses={};opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0
    with mujoco.Renderer(model,height=540,width=720) as renderer:
        for name,pose in [('Original recorded pose',original),('Offline candidate roots >= 0.65 rad',candidate)]:
            mujoco.mj_resetData(model,data);data.qpos[:]=pose;data.ctrl[:]=pose[:16];mujoco.mj_forward(model,data)
            initial=hits();gap=lower_bound();max_drift=0.;max_pen=0.;min_clear=gap;trace=[]
            for i in range(round(2/model.opt.timestep)):
                mujoco.mj_step(model,data)
                drift=1000*np.linalg.norm(data.qpos[-7:-4]-pose[-7:-4]);max_drift=max(max_drift,float(drift))
                contacts=support_hits();max_pen=max(max_pen,max([c['penetration_mm'] for c in contacts]or[0]))
                min_clear=min(min_clear,lower_bound())
                if i%5==0:trace.append(data.qpos.tolist())
            poses[name]=dict(initial_qpos=pose.tolist(),initial_contacts=initial,initial_clearance_lower_bound_mm=gap,
                             hold_2s=dict(max_cube_drift_mm=max_drift,max_support_penetration_mm=max_pen,
                                          min_clearance_lower_bound_mm=min_clear,final_qpos=data.qpos.tolist(),trace=trace))
            data.qpos[:]=pose;mujoco.mj_forward(model,data)
            for azimuth,elevation in [(105,-25),(90,-80)]:
                camera=mujoco.MjvCamera();camera.lookat[:]=[.08,0,-.025];camera.distance=.36;camera.azimuth=azimuth;camera.elevation=elevation
                renderer.update_scene(data,camera=camera,scene_option=opt)
                im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,720,45),fill='black')
                draw.text((10,8),'OFFLINE MODEL | '+name,fill='white')
                draw.text((10,25),f'Motor-housing clearance lower bound: {gap:.2f} mm',fill='white');images.append(im)
    sheet=Image.new('RGB',(1440,1080))
    for i,im in enumerate(images):sheet.paste(im,((i%2)*720,(i//2)*540))
    sheet.save(ROOT/'sac_support_pose_comparison.jpg')
    teacher=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text());paths=[]
    for row in teacher['results']:
        count=0;max_pen=0.;clipped_count=0;clipped_pen=0.;minimum=float('inf')
        for rec in row['trace']:
            data.qpos[:]=rec['qpos'];mujoco.mj_forward(model,data);cs=support_hits()
            count+=bool(cs);max_pen=max(max_pen,max([c['penetration_mm'] for c in cs]or[0]))
            data.qpos[[0,4,8]]=np.maximum(data.qpos[[0,4,8]],.65);mujoco.mj_forward(model,data);cs=support_hits()
            clipped_count+=bool(cs);clipped_pen=max(clipped_pen,max([c['penetration_mm'] for c in cs]or[0]));minimum=min(minimum,lower_bound())
        paths.append(dict(offset=row['offset_index'],seed=row['seed'],frames=len(row['trace']),original_contact_frames=count,
                          original_max_support_penetration_mm=max_pen,modified_contact_frames=clipped_count,
                          modified_max_support_penetration_mm=clipped_pen,modified_min_clearance_lower_bound_mm=minimum))
    result=dict(status='offline_geometry_and_native_hold_only',motor_commands_sent=0,model=manifest,poses=poses,teacher_paths=paths,
                caveat='Root-clamped saved paths are kinematic collision checks, not replayed policy success. Candidate is not an authorized physical target. Self-contact and model calibration remain separate limitations.')
    (ROOT/'sac_support_path_audit_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(poses={k:{kk:vv for kk,vv in v.items() if kk!='initial_qpos' and kk!='hold_2s'}|{'hold':{kk:vv for kk,vv in v['hold_2s'].items() if kk not in ('trace','final_qpos')}} for k,v in poses.items()},paths=paths),indent=2))

if __name__=='__main__':main()
