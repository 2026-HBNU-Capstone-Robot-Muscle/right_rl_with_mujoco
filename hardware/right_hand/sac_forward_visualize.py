"""Render saved offline trajectories and audit proximal housing swept bounds.

No hardware imports. Bounds are CAD geometry, NOT measured wooden clearance.
"""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from mujoco_playground import registry
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
COLORS=[(1,1,1),(1,.85,0),(.9,.04,.04),(1,.38,0),(.05,.2,1),(.05,.75,.1)]
NORMALS=np.array([[0,0,1],[0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]])

def main():
    p=argparse.ArgumentParser(); p.add_argument('--input',default='sac_stage0_teacher_v2_result.json')
    args=p.parse_args(); source=ROOT/args.input
    report=json.loads(source.read_text()); audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg); model=env.mj_model
    set_cube_physics(model,mass_kg=.100)
    out=ROOT/(source.stem+'_visual'); out.mkdir(exist_ok=True)
    data=mujoco.MjData(model); camera=mujoco.MjvCamera()
    camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:]=[.09,0,.01]; camera.distance=.32; camera.azimuth=130; camera.elevation=-55
    option=mujoco.MjvOption(); option.geomgroup[2]=0; option.geomgroup[3]=1
    cube=model.geom('cube').id; model.geom_rgba[cube]=[.2,.2,.2,1]
    palm=model.body('palm').id
    proximal=[]
    for gid in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[gid])).name
        if body in ('if_bs','if_px','mf_bs','mf_px','rf_bs','rf_px') and model.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH:
            proximal.append(gid)
    summaries=[]
    for idx,row in enumerate(report['results']):
        bounds={}; traces=row['trace']
        for record in traces:
            data.qpos[:]=record['qpos']; mujoco.mj_forward(model,data)
            Rp=data.xmat[palm].reshape(3,3); pp=data.xpos[palm]
            for gid in proximal:
                mesh=model.geom_dataid[gid]; adr=model.mesh_vertadr[mesh]; count=model.mesh_vertnum[mesh]
                points=model.mesh_vert[adr:adr+count]@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
                points=(points-pp)@Rp
                key=model.geom(gid).name
                lo,hi=points.min(0)*1000,points.max(0)*1000
                if key not in bounds: bounds[key]={'start_min_mm':lo.tolist(),'start_max_mm':hi.tolist(),'swept_min_mm':lo.tolist(),'swept_max_mm':hi.tolist()}
                else:
                    bounds[key]['swept_min_mm']=np.minimum(bounds[key]['swept_min_mm'],lo).tolist()
                    bounds[key]['swept_max_mm']=np.maximum(bounds[key]['swept_max_mm'],hi).tolist()
        summaries.append(dict(offset_index=row['offset_index'],seed=row['seed'],success=row['success_step'] is not None,bounds_palm_frame=bounds))
    chosen=[]
    for outcome in (True,False):
        found=next(((i,r) for i,r in enumerate(report['results']) if (r['success_step'] is not None)==outcome),None)
        if found: chosen.append(found)
    with mujoco.Renderer(model,height=540,width=720) as renderer:
        for idx,row in chosen:
            frames=[]; trace=row['trace']; indices=sorted(set(range(0,len(trace),3))|{len(trace)-1})
            for t in indices:
                rec=trace[t]; data.qpos[:]=rec['qpos']; mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera,scene_option=option)
                R=data.geom_xmat[cube].reshape(3,3); center=data.geom_xpos[cube]
                for normal,color in zip(NORMALS,COLORS):
                    basis=np.eye(3)[np.argmin(abs(normal))]; u=np.cross(basis,normal); v=np.cross(normal,u)
                    mat=R@np.stack([u,v,normal],axis=1)
                    scene=renderer.scene; geom=scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_BOX,np.array([.024,.024,.00015]),
                                       center+R@normal*.0274,mat.flatten(),np.array([*color,1],dtype=np.float32))
                    scene.ngeom+=1
                im=Image.fromarray(renderer.render()); draw=ImageDraw.Draw(im)
                text=f"OFFLINE ONLY | offset {row['offset_index']} seed {row['seed']} | step {rec['step']}"
                draw.rectangle((0,0,720,42),fill='black'); draw.text((12,8),text,fill='white')
                if t: draw.text((12,24),f"tip {rec['tip_deg']:.1f} deg | target error {rec['target_error_deg']:.1f} deg",fill='white')
                frames.append(im)
            label='success' if row['success_step'] is not None else 'failure'
            frames[0].save(out/f'{label}.gif',save_all=True,append_images=frames[1:],duration=120,loop=0)
            picks=[frames[int(x)] for x in np.linspace(0,len(frames)-1,4)]
            sheet=Image.new('RGB',(1440,1080))
            for i,im in enumerate(picks): sheet.paste(im,((i%2)*720,(i//2)*540))
            sheet.save(out/f'{label}_stages.jpg')
    payload=dict(status='offline_CAD_swept_bounds_not_wood_clearance',source=source.name,
                 geometry_frame='palm body local XYZ in mm',trials=summaries,
                 force_limits=dict(joint_force_ranges=model.jnt_actfrcrange.tolist(),joint_force_limited=model.jnt_actfrclimited.tolist()),
                 caveat='Proximal visual CAD mesh swept bounds; no calibrated wood geometry, no measured real clearance. Rendered cube colors illustrate face IDs, not camera evidence.')
    (out/'housing_sweep.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(out)

if __name__=='__main__': main()
