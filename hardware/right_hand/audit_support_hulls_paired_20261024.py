"""Exact CAD-triangle/obstacle-box overlap diagnostic at saved OFF encoders.

Compare the convex-hull proxy's penetration with whether the actual visual
triangles intersect each support box. This changes no model or controller.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'pose_pair_readonly_20261024_045449_430810'
r=json.loads((FOLDER/'result.json').read_bytes())
raw=np.array([next(v['position_ticks_signed'] for v in r['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
palm=m.body('palm').id;rotation=d.xmat[palm].reshape(3,3);origin=d.xpos[palm]


def clip(poly,axis,bound,sign):
    out=[]
    if not len(poly):return out
    previous=poly[-1];before=sign*(previous[axis]-bound)<=1e-8
    for point in poly:
        inside=sign*(point[axis]-bound)<=1e-8
        if inside!=before:
            t=(bound-previous[axis])/(point[axis]-previous[axis]);out.append(previous+t*(point-previous))
        if inside:out.append(point)
        previous=point;before=inside
    return out


records=[]
for g in range(m.ngeom):
    name=m.geom(g).name
    if not name.endswith('_support_proxy'):continue
    mid=int(m.geom_dataid[g]);va=int(m.mesh_vertadr[mid]);vn=int(m.mesh_vertnum[mid])
    fa=int(m.mesh_faceadr[mid]);fn=int(m.mesh_facenum[mid])
    world=m.mesh_vert[va:va+vn]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    local=(world-origin)@rotation*1000;triangles=local[m.mesh_face[fa:fa+fn]]
    for support,(lower,upper) in manifest['bounds_palm_mm'].items():
        lower=np.array(lower);upper=np.array(upper)
        mask=np.all(triangles.max(1)>=lower,axis=1)&np.all(triangles.min(1)<=upper,axis=1)
        clipped=[]
        for tri in triangles[mask]:
            poly=list(tri)
            for axis in range(3):
                poly=clip(poly,axis,lower[axis],-1);poly=clip(poly,axis,upper[axis],1)
                if not len(poly):break
            if len(poly)>=3:clipped.append(np.array(poly))
        overlaps=[-float(c.dist)*1000 for c in d.contact if c.dist<0 and set((int(c.geom1),int(c.geom2)))==set((g,m.geom(support).id))]
        record=dict(geom=name,support=support,proxy_overlap_mm=max(overlaps,default=0.),
            original_triangle_count=len(triangles),broad_phase_triangles=int(mask.sum()),
            intersecting_surface_triangles=len(clipped))
        if clipped:
            points=np.concatenate(clipped)
            record.update(surface_intersection_bounds_palm_mm=[points.min(0).tolist(),points.max(0).tolist()])
        records.append(record)
out=dict(status='exact_visual_surface_support_box_diagnostic',records=records,
    native_geometry_changed=False,hardware_execution_allowed=False,real_devices_opened=False,
    limitations=['Triangle clipping checks the existing provisional box positions, not measured real contact.',
        'A nonzero triangle intersection supports geometric overlap; zero surface crossings alone does not exclude full containment.'])
(FOLDER/'support_hull_audit.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
