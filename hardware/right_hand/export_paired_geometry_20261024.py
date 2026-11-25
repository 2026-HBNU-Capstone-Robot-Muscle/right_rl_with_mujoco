"""Export native FK at paired OFF encoders for projection diagnostics. No devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'pose_pair_readonly_20261024_045449_430810'
blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob)
assert r['status']=='photo_and_readings_saved' and r['pair_check']['maximum_drift_ticks']<=3
assert r['servo_write_commands_sent']==0 and r['torque_changes_requested']==0
raw=np.array([next(v['position_ticks_signed'] for v in r['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
meshes=[]
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand or m.geom_group[g]>=3:continue
    if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
    index=int(m.geom_dataid[g]);start=int(m.mesh_vertadr[index]);count=int(m.mesh_vertnum[index])
    vertices=m.mesh_vert[start:start+count].copy()
    try:vertices=vertices[ConvexHull(vertices).vertices]
    except Exception:pass
    world=vertices@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    meshes.append(dict(geom=m.geom(g).name,body=m.body(m.geom_bodyid[g]).name,vertices_world_m=world.tolist()))
contacts=[]
for c in d.contact:
    if c.dist>=0:continue
    a,b=int(c.geom1),int(c.geom2)
    if int(m.geom_bodyid[a]) not in hand and int(m.geom_bodyid[b]) not in hand:continue
    contacts.append(dict(pair=sorted((m.geom(a).name,m.geom(b).name)),overlap_mm=-float(c.dist)*1000,
        position_world_m=c.pos.tolist()))
record=dict(source=str(FOLDER.name),source_sha256=hashlib.sha256(blob).hexdigest(),raw_ticks=raw.tolist(),
    mapping=list(SIM_TO_MOTOR_IDS),meshes=meshes,
    joints=[dict(name=m.joint(int(j)).name,motor_id=int(SIM_TO_MOTOR_IDS[i]),world_m=d.xanchor[j].tolist())
            for i,j in enumerate(m.actuator_trnid[:,0])],
    palm_position_world_m=d.xpos[palm].tolist(),palm_rotation_world=d.xmat[palm].reshape(3,3).tolist(),
    middle_base_position_world_m=d.xpos[m.body('mf_bs').id].tolist(),
    contacts=contacts,support_manifest=manifest,physical_calibration_verified=False,
    hardware_execution_allowed=False,real_devices_opened=False)
(FOLDER/'native_geometry.json').write_text(json.dumps(record),encoding='utf-8')
print(json.dumps(dict(meshes=len(meshes),contacts=contacts,hardware_execution_allowed=False),indent=2))
