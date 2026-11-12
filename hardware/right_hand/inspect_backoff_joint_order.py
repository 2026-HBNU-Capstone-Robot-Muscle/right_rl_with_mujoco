"""Offline actuator/qpos mapping audit. Does not import hardware controllers."""
import json
from pathlib import Path
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m=env.mj_model
rows=[]
for a in range(m.nu):
    j=int(m.actuator_trnid[a,0])
    rows.append(dict(actuator_index=a,actuator_name=m.actuator(a).name,joint=m.joint(j).name,
                     qpos_address=int(m.jnt_qposadr[j]),joint_axis=m.jnt_axis[j].tolist(),
                     range=m.jnt_range[j].tolist()))
report=dict(rows=rows,direct_qpos_assignment_valid=all(r['qpos_address']==r['actuator_index'] for r in rows),
            model_xml=str(env.xml_path),qpos0=m.qpos0[:16].tolist(),
            body_mounts=[dict(name=m.body(i).name,parent=m.body(int(m.body_parentid[i])).name,
                              position=m.body_pos[i].tolist(),quaternion=m.body_quat[i].tolist())
                         for i in range(m.nbody) if m.body(i).name in ('palm','th_mp','th_bs','th_px','th_ds','mf_bs','mf_px','mf_md','mf_ds')])
(ROOT/'backoff_video_review_20261016'/'joint_order_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
