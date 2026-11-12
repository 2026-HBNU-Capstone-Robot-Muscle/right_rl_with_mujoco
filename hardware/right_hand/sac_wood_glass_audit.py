"""Offline clearance and command replay against the measured external support."""
import json
import os
import sys
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_render_geometry import render
from sac_support_cad_reference import mesh_points
ROOT=Path(__file__).resolve().parent

def closest(model,data,geoms,obstacle):
    values=[]
    for gid in geoms:
        distance=mujoco.mj_geomDistance(model,data,gid,obstacle,.2,np.zeros(6))
        values.append((float(distance*1000),model.geom(gid).name))
    distance,name=min(values)
    return dict(distance_mm=distance,geom=name)

def surface_inside(model,data,geoms,obstacle):
    hits=[]; rot=data.geom_xmat[obstacle].reshape(3,3); center=data.geom_xpos[obstacle]; size=model.geom_size[obstacle]
    for gid in geoms:
        points=(mesh_points(model,data,gid)-center)@rot
        depths=size-np.abs(points); mask=np.all(depths>1e-5,axis=1)
        if mask.any():hits.append(dict(geom=model.geom(gid).name,vertices_inside=int(mask.sum()),
                                     max_distance_to_nearest_box_face_mm=float(np.min(depths[mask],axis=1).max()*1000)))
    return hits

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    baseline=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text())
    reports=[]
    # Nominal dimensions plus a narrow sensitivity range for approximate ruler readings.
    for projection,thickness,reference in ((34,6,3),(31,5,3),(37,7,3),(34,6,-25)):
        model,spec=build(env,projection,thickness,front_reference_mm=reference); data=mujoco.MjData(model)
        proximal=[]; primitives=[]
        for gid in range(model.ngeom):
            body=model.body(int(model.geom_bodyid[gid])).name
            if body.startswith(('if_','mf_','rf_','th_')):
                if model.geom_contype[gid]:primitives.append(gid)
                if body in ('if_bs','if_px','mf_bs','mf_px','rf_bs','rf_px') and model.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH:proximal.append(gid)
        obstacles=[model.geom('support_glass').id,model.geom('support_wood').id]
        rows=[]
        for trial in baseline['results']:
            if trial['success_step'] is None:continue
            records=[]
            for rec in trial['trace']:
                data.qpos[:]=rec['qpos']; data.ctrl[:]=rec['ctrl']; mujoco.mj_forward(model,data)
                distances={model.geom(g).name:dict(primitive=closest(model,data,primitives,g),
                                                   proximal_visual_hull=closest(model,data,proximal,g),
                                                   actual_mesh_vertices_inside=surface_inside(model,data,proximal,g)) for g in obstacles}
                contacts=[dict(geoms=[model.geom(c.geom1).name,model.geom(c.geom2).name],distance_mm=float(c.dist*1000))
                          for c in data.contact if (c.geom1 in obstacles or c.geom2 in obstacles) and c.dist<0]
                records.append(dict(step=rec['step'],distances=distances,native_negative_contacts=contacts))
            rows.append(dict(offset_index=trial['offset_index'],seed=trial['seed'],
                             initial=records[0],worst_primitive=min(records,key=lambda r:min(x['primitive']['distance_mm'] for x in r['distances'].values())),
                             worst_visual_hull=min(records,key=lambda r:min(x['proximal_visual_hull']['distance_mm'] for x in r['distances'].values())),trace=records))
        if projection==34:
            # Keep measured reset intact: inspect intersection rather than silently move the hand.
            q=np.array(baseline['results'][0]['trace'][0]['qpos'])
            render(model,q,ROOT/f'sac_support_start_reference_{reference}.png',True)
        reports.append(dict(model=spec,results=rows))
        print(json.dumps(dict(projection_mm=projection,glass_mm=thickness,front_reference_mm=reference,
                             initial=rows[0]['initial'],worst_primitive=rows[0]['worst_primitive']),indent=2),flush=True)
    payload=dict(status='offline_support_clearance_audit',motor_commands_sent=0,scenarios=reports,
                 caveat='Visual mesh distance uses convex hulls and can be conservative. Negative distance is model overlap, not measured real penetration. Initial overlap must be resolved before training with support.')
    (ROOT/'sac_wood_glass_audit_result.json').write_text(json.dumps(payload,indent=2)+'\n')

if __name__=='__main__':main()
