"""Offline comparison of full joint-chain FK with official LEAP URDF.

Downloads public geometry only; never opens a serial port or changes hardware.
"""
from pathlib import Path
import hashlib,json,urllib.request,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
from mujoco_playground import registry

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'backoff_video_review_20261016'
URL='https://raw.githubusercontent.com/leap-hand/LEAP_Hand_Sim/master/assets/leap_hand/robot.urdf'
p=OUT/'official_robot.urdf'
if not p.exists():
    with urllib.request.urlopen(URL,timeout=30) as response:p.write_bytes(response.read())
urdf=ET.fromstring(p.read_bytes())
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
cad_names=[1,0,2,3,5,4,6,7,9,8,10,11,12,13,14,15]
cad_to_index={str(j):i for i,j in enumerate(cad_names)}
ORDER=np.array(list(range(4,16))+list(range(4)))
hw=json.loads((ROOT/'leap_id12_backoff_empty_20261016_144922_807163.json').read_text())

def vector(element,key,default='0 0 0'):
    return np.fromstring(element.get(key,default),sep=' ')

def fk(q):
    poses={'palm_lower':np.eye(4)};joints={};pending=list(urdf.findall('joint'))
    while pending:
        changed=False
        for j in pending[:]:
            parent=j.find('parent').get('link')
            if parent not in poses:continue
            origin=j.find('origin');transform=np.eye(4)
            transform[:3,3]=vector(origin,'xyz')
            transform[:3,:3]=Rotation.from_euler('xyz',vector(origin,'rpy')).as_matrix()
            transform=poses[parent]@transform
            index=cad_to_index[j.get('name')];axis=vector(j.find('axis'),'xyz')
            joints[index]=dict(anchor=transform[:3,3].copy(),axis=transform[:3,:3]@axis)
            turn=np.eye(4);turn[:3,:3]=Rotation.from_rotvec(axis*q[index]).as_matrix()
            poses[j.find('child').get('link')]=transform@turn
            pending.remove(j);changed=True
        if not changed:raise RuntimeError('Unresolved URDF chain')
    return joints

results=[]
for label,raw in [('zero',None),('hold3290',hw['approach_hold']['actual_end_median']),('hold3130',hw['return_hold']['actual_end_median'])]:
    q=np.zeros(16) if raw is None else np.asarray(raw)[ORDER]*2*np.pi/4096-np.pi
    expected=fk(q);d=mujoco.MjData(m);d.qpos[:16]=q;d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,d);palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    rows=[]
    for i in range(16):
        j=int(m.actuator_trnid[i,0]);anchor=R.T@(d.xanchor[j]-o);axis=R.T@d.xaxis[j]
        rows.append(dict(motor_id=int(ORDER[i]+1),joint=m.joint(j).name,cad_joint=cad_names[i],
                         anchor_error_mm=float(np.linalg.norm(anchor-expected[i]['anchor'])*1000),
                         axis_error_deg=float(np.degrees(np.arccos(np.clip(axis@expected[i]['axis'],-1,1)))),
                         model_anchor_palm_m=anchor.tolist(),official_anchor_palm_m=expected[i]['anchor'].tolist()))
    row=dict(pose=label,max_anchor_error_mm=max(r['anchor_error_mm'] for r in rows),max_axis_error_deg=max(r['axis_error_deg'] for r in rows),joints=rows)
    results.append(row)
    print(json.dumps({k:v for k,v in row.items() if k!='joints'}),flush=True)
report=dict(source_url=URL,source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),model=str(env.xml_path),results=results,
            hardware_commands_sent=0,caveat='Agreement checks source kinematics only, not actual assembly zeros, ID assignment, tip shape or physical clearance.')
(OUT/'official_chain_fk_audit.json').write_text(json.dumps(report,indent=2))
