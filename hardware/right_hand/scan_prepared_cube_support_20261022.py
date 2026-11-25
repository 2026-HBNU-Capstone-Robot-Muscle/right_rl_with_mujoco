"""Offline settling near the newly reported palm-to-root tipping placement.

Current hardware angles are fixed only in the first geometric settling probe;
dynamic holding is independently simulated afterward. Neither probe is a
measurement of the hidden cube support surface.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim,metadata
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def main():
    source=OUT/'placement_review.json';review=json.loads(source.read_text(encoding='utf-8'))
    hypotheses=review['historical_bridge_diagnostic']['hypotheses']
    seed=next(h for h in hypotheses if h['ippe_indices']==[0,0,0])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
    physics=set_cube_physics(m,side_m=.055,mass_kg=.100)
    q=ticks_to_sim(review['placement_position_ticks']);ctrl=ticks_to_sim(review['placement_goal_ticks'])
    d=mujoco.MjData(m);d.qpos[:16]=q;mujoco.mj_forward(m,d)
    palm=m.body('palm').id;Rp=d.xmat[palm].reshape(3,3).copy();origin=d.xpos[palm].copy()
    cube=m.geom('cube').id;gravity_up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    Rseed=np.array(seed['cube_rotation_world']);quat=np.empty(4);mujoco.mju_mat2Quat(quat,Rseed.ravel())
    base=np.array(seed['cube_center_world_m']);base_local=Rp.T@(base-origin)
    rows=[]
    def contacts(data):
        result=[]
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            result.append(dict(geom=m.geom(other).name,depth_mm=max(0,-float(c.dist)*1000)))
        return result
    def up_error(rotation):
        green=-rotation[:,1]
        return float(np.degrees(np.arccos(np.clip(green@gravity_up,-1,1))))
    for dx in (-15,-10,-5,0,5,10,15):
        for dy in (-10,-5,0,5,10):
            # Start above the uncertain surface; do not settle an initially
            # penetrating cube and call the ejected result a measured reset.
            center=base+Rp@np.array([dx,dy,0])/1000+gravity_up*.015
            data=mujoco.MjData(m);data.qpos[:16]=q;data.qpos[-7:-4]=center;data.qpos[-4:]=quat;data.ctrl[:]=ctrl
            mujoco.mj_forward(m,data);initial=contacts(data)
            peak_pen=0.
            for k in range(round(1.5/m.opt.timestep)):
                # This is expressly the kinematic placement probe. Forces from
                # a perfect fixed hand are not motor performance or SAC return.
                data.qpos[:16]=q;data.qvel[:16]=0;mujoco.mj_forward(m,data)
                mujoco.mj_step(m,data)
                peak_pen=max(peak_pen,max([c['depth_mm'] for c in contacts(data)]+[0]))
            final_q=data.qpos.copy();final_contacts=contacts(data)
            Rfinal=np.empty(9);mujoco.mju_quat2Mat(Rfinal,data.qpos[-4:]);Rfinal=Rfinal.reshape(3,3)
            local=Rp.T@(data.qpos[-7:-4]-origin)
            vel=float(np.linalg.norm(data.qvel[-6:-3]))
            row=dict(offset_palm_mm=[dx,dy,0],initial_cube_contact_geoms=sorted({c['geom'] for c in initial}),
                initial_penetration_mm=max([c['depth_mm'] for c in initial]+[0]),
                peak_cube_penetration_mm=peak_pen,final_cube_contact_geoms=sorted({c['geom'] for c in final_contacts}),
                green_up_error_deg=up_error(Rfinal),change_from_seed_deg=float(np.degrees(2*np.arccos(np.clip(abs(quat@data.qpos[-4:]),0,1)))),
                final_cube_center_palm_mm=(local*1000).tolist(),final_cube_qpos=final_q[-7:].tolist(),
                linear_speed_m_s=vel,held_by_hand=any(c['geom'].startswith(('palm_','if_','mf_','rf_','th_')) for c in final_contacts),
                qpos=final_q.tolist())
            rows.append(row)
    candidates=[r for r in rows if r['initial_penetration_mm']<=.01 and r['held_by_hand'] and r['linear_speed_m_s']<.01]
    dynamic=[]
    # Independent dynamic hold from a subset of settled candidates; retains
    # motor lag and normal native constraints, with no state rewriting.
    chosen=sorted(candidates,key=lambda r:np.linalg.norm(r['offset_palm_mm']))[:7]
    for row in chosen:
        data=mujoco.MjData(m);data.qpos[:]=row['qpos'];data.qvel[:]=0;data.ctrl[:]=ctrl;mujoco.mj_forward(m,data)
        cube_origin=data.qpos[-7:-4].copy();err=0;depth=0
        for _ in range(round(1/m.opt.timestep)):
            mujoco.mj_step(m,data);err=max(err,float(np.max(abs(data.qpos[:16]-ctrl))))
            depth=max(depth,max([c['depth_mm'] for c in contacts(data)]+[0]))
        Rfinal=np.empty(9);mujoco.mju_quat2Mat(Rfinal,data.qpos[-4:])
        dynamic.append(dict(offset_palm_mm=row['offset_palm_mm'],max_tracking_rad=err,
            drift_mm=float(np.linalg.norm(data.qpos[-7:-4]-cube_origin)*1000),
            peak_cube_penetration_mm=depth,final_cube_contact_geoms=sorted({c['geom'] for c in contacts(data)}),
            green_up_error_deg=up_error(Rfinal.reshape(3,3)),qpos=data.qpos.tolist()))
    report=dict(status='offline_reported_tipping_support_scan',mapping=metadata(),model_manifest=manifest,cube_physics=physics,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),seed_ippe_indices=[0,0,0],
        seed_palm_center_mm=(base_local*1000).tolist(),seed_up_error_relative_gravity_deg=up_error(Rseed),
        user_observation='Tried placing on palm beside ID40; cube tipped into the recessed root region.',
        seed_registration_verified=False,user_observation_is_not_a_measured_3d_pose=True,
        kinematic_settling_seconds=1.5,dynamic_hold_seconds=1,rows=rows,dynamic_holds=dynamic,
        usable_kinematic_candidates=len(candidates),hardware_commands_sent=0,hardware_execution_allowed=False,
        limitations=['Current hand model does not include compliant cables and shell clearances are approximate.',
                     'Historical marker relation and selected branch are provisional, not online registration.',
                     'Initial placement height is a synthetic drop probe, not the person\'s observed release height.',
                     'Kinematic probe fixes hand joints; dynamic hold is reported separately.'])
    (OUT/'support_settling_scan.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(samples=len(rows),usable_kinematic=len(candidates),
        seed_palm_center_mm=report['seed_palm_center_mm'],seed_green_up_error=report['seed_up_error_relative_gravity_deg'],
        centered=[{k:r[k] for k in ('offset_palm_mm','final_cube_contact_geoms','green_up_error_deg','linear_speed_m_s','final_cube_center_palm_mm')} for r in rows if r['offset_palm_mm'][1]==0],
        dynamic=[{k:v for k,v in r.items() if k!='qpos'} for r in dynamic]),indent=2),flush=True)
    # Render the newly simulated settling choices, not an asserted fitted cube.
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_glass','support_wood')}
    for gi in range(m.ngeom):
        if int(m.geom_bodyid[gi]) not in hand and gi not in supports and int(m.geom_bodyid[gi])!=int(m.geom_bodyid[cube]):m.geom_group[gi]=5
    opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
    selected=[r for r in rows if r['offset_palm_mm'] in ([-10,0,0],[0,0,0],[10,0,0])]
    sheet=Image.new('RGB',(1920,480),'white')
    with mujoco.Renderer(m,height=480,width=640) as ren:
        for i,row in enumerate(selected):
            data=mujoco.MjData(m);data.qpos[:]=row['qpos'];mujoco.mj_forward(m,data)
            view=Rp@np.array([.4,-1,-.8]);view/=np.linalg.norm(view)
            up=-Rp[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
            cam=mujoco.MjvCamera();cam.lookat[:]=origin+Rp@np.array([-.02,-.025,-.04]);cam.distance=.42
            ren.update_scene(data,camera=cam,scene_option=opt)
            for eye in ren.scene.camera:eye.pos[:]=cam.lookat+view*.42;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(ren.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,55),fill='black')
            draw.text((8,6),f"Synthetic settle: palm offset {row['offset_palm_mm']}mm",fill='white')
            draw.text((8,27),f"Green normal vs gravity {row['green_up_error_deg']:.1f}deg | OFFLINE ONLY",fill='white');sheet.paste(im,(i*640,0))
    sheet.save(OUT/'support_settling_candidates.jpg')


if __name__=='__main__':main()
