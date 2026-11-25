"""Read CAD reference geometry for the measured wood/glass stack; no hardware."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent

def mesh_points(model,data,gid):
    mid=model.geom_dataid[gid]; start=model.mesh_vertadr[mid]; count=model.mesh_vertnum[mid]
    return model.mesh_vert[start:start+count]@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg); model=env.mj_model
    data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    palm=model.body('palm').id; R=data.xmat[palm].reshape(3,3); origin=data.xpos[palm]
    geoms={}
    for gid in range(model.ngeom):
        if model.geom_bodyid[gid]!=palm or model.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        points=(mesh_points(model,data,gid)-origin)@R
        # Axis-aligned CAD surfaces: common vertex coordinates identify planes.
        planes=[]
        for axis in range(3):
            vals,counts=np.unique(np.round(points[:,axis]*1000,1),return_counts=True)
            order=np.argsort(counts)[-8:][::-1]
            planes.append([[float(vals[i]),int(counts[i])] for i in order])
        mid=model.geom_dataid[gid]; fa=model.mesh_faceadr[mid]; fn=model.mesh_facenum[mid]
        tri=points[model.mesh_face[fa:fa+fn]]*1000
        cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); area=np.linalg.norm(cross,axis=1)/2
        normals=cross/np.maximum(2*area[:,None],1e-12); centroid=tri.mean(1)
        area_planes=[]
        for axis in range(3):
            selected=np.abs(normals[:,axis])>.995
            vals=np.round(centroid[selected,axis],1); weights=area[selected]
            totals={float(v):float(weights[vals==v].sum()) for v in np.unique(vals)}
            area_planes.append(sorted(totals.items(),key=lambda kv:-kv[1])[:12])
        geoms[model.geom(gid).name]=dict(min_mm=(points.min(0)*1000).tolist(),max_mm=(points.max(0)*1000).tolist(),common_planes=planes,area_planes_mm2=area_planes)
    joints=[]
    for jid in range(16):
        joints.append(dict(name=model.joint(jid).name,anchor_palm_mm=((data.xanchor[jid]-origin)@R*1000).tolist()))
    result=dict(palm_origin_world_m=origin.tolist(),palm_rotation_world=R.tolist(),geoms=geoms,joints=joints,
                source_xml=env.xml_path)
    (ROOT/'sac_support_cad_reference_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
