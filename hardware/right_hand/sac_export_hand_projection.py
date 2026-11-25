"""Export model geometry at measured joints for an offline camera overlay."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    run=json.loads((ROOT/'leap_sac_start_capture_result.json').read_text())
    sample=[s for s in run['samples'] if s['phase']=='stage_2'][-1]
    mapping=np.r_[np.arange(5,17),np.arange(1,5)]
    joints=np.asarray(sample['actual'])[mapping-1]*2*np.pi/4096-np.pi
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    model,_=build(registry.load('LeapCubeFaceUp',config=cfg))
    data=mujoco.MjData(model);data.qpos[:16]=joints
    data.qpos[-7:-4]=[1,1,1];data.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(model,data)
    meshes=[]
    for g in range(model.ngeom):
        name=model.geom(g).name
        if not name.endswith('_visual') or model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        index=model.geom_dataid[g];first=model.mesh_vertadr[index];count=model.mesh_vertnum[index]
        vertices=model.mesh_vert[first:first+count]
        world=vertices@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g]
        meshes.append(dict(name=name,vertices_world_m=world.tolist()))
    report=dict(source_folder=run['vision_recording']['folder'],sample_phase=sample['phase'],
                sample_t=sample['t'],joints_sim_rad=joints.tolist(),meshes=meshes,
                joint_anchors=[dict(motor_id=int(mapping[i]),world_m=data.xanchor[i].tolist()) for i in range(16)],
                caveat='Nearest final-hold sample, not exactly synchronized image. Convex projected hulls are visual approximations.')
    (ROOT/'sac_hand_projection_geometry.json').write_text(json.dumps(report)+'\n')
    print('Exported',len(meshes),'visual meshes and 16 joint anchors; no device access.')

if __name__=='__main__':main()
