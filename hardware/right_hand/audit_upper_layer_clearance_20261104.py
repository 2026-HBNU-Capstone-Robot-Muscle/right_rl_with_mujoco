"""Exact convex separating-axis checks for static upper-layer volumes.

This is saved CAD geometry only. Positive values prove separation; a negative
value is an overlap indication, not physical penetration or layer-turn success.
No geometry or dynamic model is changed.
"""
import json
import numpy as np,mujoco
from scipy.spatial import ConvexHull
from research_rotation_body_hold_20261104 import CP,OUT,restore

def local_mesh(m,d,g,cube):
    if int(m.geom_type[g])==7:
        mesh=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mesh]);v=np.asarray(m.mesh_vert[a:a+int(m.mesh_vertnum[mesh])],float)
    elif int(m.geom_type[g])==6:
        v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[g]
    else:return None
    world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    return (world-d.geom_xpos[cube])@d.geom_xmat[cube].reshape(3,3)*1000

def box_gap(v,center,half):
    hull=ConvexHull(v);edges=set()
    for tri in hull.simplices:
        for a,b in ((tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])):edges.add(tuple(sorted((int(a),int(b)))))
    directions=np.array([v[b]-v[a] for a,b in edges]);directions/=np.linalg.norm(directions,axis=1)[:,None]
    cross=np.cross(np.eye(3)[:,None,:],directions[None,:,:]).reshape(-1,3);norm=np.linalg.norm(cross,axis=1);cross=cross[norm>1e-8]/norm[norm>1e-8,None]
    axes=np.vstack([np.eye(3),hull.equations[:,:3],cross]);proj=(v-center)@axes.T;radius=np.abs(axes)@half
    return float(max(np.max(proj.min(0)-radius),np.max(-radius-proj.max(0))))

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));cube=m.geom('cube').id
    paths=[('before_lowering',CP/'closest_state.npz')]+[(f'thumb_lower_{x:g}',OUT/f'turn_0_lower_{x:g}_normal_10/end_state.npz') for x in (8,14)]
    paths += [(f'thumb_lower_{x}',OUT/f'translation_case_{case:02d}/end_state.npz') for x,case in ((4,9),(12,10),(16,11))]
    rows=[]
    for name,path in paths:
        d=restore(m,np.load(path));geoms=[]
        for tip in ('mf_tip','th_tip'):
            g=m.geom(tip).id;v=local_mesh(m,d,g,cube)
            gap=box_gap(v,np.array([0.,-55/3.,0.]),np.array([27.5,55/6.,27.5]))
            # A circumscribing square for the full 90deg upper-layer sweep is
            # conservative; it can reject clearance but cannot prove collision.
            conservative=box_gap(v,np.array([0.,-55/3.,0.]),np.array([27.5*np.sqrt(2),55/6.,27.5*np.sqrt(2)]))
            geoms.append(dict(geom=tip,static_upper_layer_separation_axis_gap_mm=gap,conservative_swept_square_gap_mm=conservative))
        rows.append(dict(state=name,geometries=geoms,upper_layer_static_separated=all(v['static_upper_layer_separation_axis_gap_mm']>0 for v in geoms),upper_layer_conservative_sweep_separated=all(v['conservative_swept_square_gap_mm']>0 for v in geoms)))
    report=dict(status='postturn_upper_layer_CAD_volume_audit',cases=rows,rigid_cube_dynamic_test_does_not_verify_layer_rotation=True,physical_clearance_verified=False,devices_opened=False)
    (OUT/'upper_layer_clearance_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
