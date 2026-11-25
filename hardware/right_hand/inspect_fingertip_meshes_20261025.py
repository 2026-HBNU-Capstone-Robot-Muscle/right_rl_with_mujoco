"""Inspect installed native collision/visual fingertip geometry, no devices."""
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from original_horizontal_scene_20261025 import Scene,OUT


def main():
    scene=Scene();m=scene.m;d=scene.d
    d.qpos[:16]=scene.q0;d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
    rows=[]
    for finger in ('if','mf','rf','th'):
        body=m.body(finger+'_ds').id;geoms=[]
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g])!=body:continue
            row=dict(name=m.geom(g).name,kind=int(m.geom_type[g]),group=int(m.geom_group[g]),
                contype=int(m.geom_contype[g]),conaffinity=int(m.geom_conaffinity[g]),
                local_pos=m.geom_pos[g].tolist(),local_quat=m.geom_quat[g].tolist(),size=m.geom_size[g].tolist())
            if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                mesh=int(m.geom_dataid[g]);start=int(m.mesh_vertadr[mesh]);count=int(m.mesh_vertnum[mesh])
                vertices=m.mesh_vert[start:start+count]
                # Include mesh compiler centering and geom transform exactly.
                bodyworld=d.xmat[body].reshape(3,3);geomworld=d.geom_xmat[g].reshape(3,3)
                local=((vertices@geomworld.T+d.geom_xpos[g])-d.xpos[body])@bodyworld
                row.update(mesh=m.mesh(mesh).name,mesh_vertex_count=count,
                    visual_bounds_in_body_mm=[(local.min(0)*1000).tolist(),(local.max(0)*1000).tolist()])
            geoms.append(row)
        rows.append(dict(body=finger+'_ds',geoms=geoms))
    (OUT/'fingertip_collision_visual_inventory.json').write_text(json.dumps(dict(rows=rows,
        model_source='installed MuJoCo Playground / Menagerie asset data',
        inspection_only=True,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print(json.dumps(rows,indent=2),flush=True)


if __name__=='__main__':main()
