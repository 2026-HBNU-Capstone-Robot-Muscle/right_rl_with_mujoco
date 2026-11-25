"""Export current encoder-based CAD landmarks for saved-image alignment only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,metadata
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703'
def main():
    r=json.loads((ROOT/'quarter_support_photo_20261025_221703_805528/result.json').read_bytes());raw=np.array(r['placement_position_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'photo_alignment_scene.xml');d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    geoms=[]
    for i in range(m.ngeom):
        if int(m.geom_bodyid[i]) not in hand or int(m.geom_group[i]) not in (2,3):continue
        item=dict(name=m.geom(i).name,body=m.body(int(m.geom_bodyid[i])).name,group=int(m.geom_group[i]),type=int(m.geom_type[i]),
            center=d.geom_xpos[i].tolist(),rotation=d.geom_xmat[i].reshape(3,3).tolist(),size=m.geom_size[i].tolist())
        if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_BOX:
            local=np.array(list(itertools.product((-1,1),repeat=3)))*m.geom_size[i];points=local@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
            item['corners_world']=points.tolist()
        geoms.append(item)
    axes=[]
    for name,id in zip(metadata()['sim_joint_names'],metadata()['motor_ids_in_sim_order']):
        j=m.joint(name).id;axes.append(dict(name=name,motor_id=id,world_anchor=d.xanchor[j].tolist(),world_axis=d.xaxis[j].tolist()))
    result=dict(raw_ticks=raw.tolist(),model=manifest,geoms=geoms,motor_axes=axes,palm_world_position=d.xpos[m.body('palm').id].tolist(),
        palm_world_rotation=d.xmat[m.body('palm').id].reshape(3,3).tolist(),physical_camera_registration_verified=False,hardware_commands_sent=0)
    (OUT/'cad_geometry.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PALM',result['palm_world_position']);print('BOXES',json.dumps([g for g in geoms if g['body'] in ('if_px','mf_px','th_mp') and g['group']==3]));print('AXES',json.dumps(axes))
if __name__=='__main__':main()
