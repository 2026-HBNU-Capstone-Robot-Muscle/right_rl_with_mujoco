"""Offline fixture geometry at actually measured empty-hand joint angles."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_support_separation import SupportSeparation
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m,_=build(registry.load('LeapCubeFaceUp',config=cfg));d=mujoco.MjData(m)
sep=SupportSeparation(m);support={m.geom(n).id for n in ('support_glass','support_wood')}
reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text());reference=np.asarray(reset['candidates'][0]['qpos'])
mapping=np.array([5,6,7,8,9,10,11,12,13,14,15,16,1,2,3,4])-1;rows=[]
for path in sorted(ROOT.glob('leap_sac_start_empty_20*.json')):
    r=json.loads(path.read_text(encoding='utf-8'))
    if r.get('status')!='empty_check_complete':continue
    last=r['samples'][-1];angles=np.asarray(last['actual'])[mapping]*(2*np.pi/4096)-np.pi
    d.qpos[:]=reference;d.qpos[:16]=angles;d.qpos[-7:-4]=[1,1,1];mujoco.mj_forward(m,d)
    contacts=[dict(pair=[m.geom(c.geom1).name,m.geom(c.geom2).name],depth_mm=-float(c.dist)*1000) for c in d.contact if c.dist<0 and (c.geom1 in support or c.geom2 in support)]
    gap=min(sep.lower_bound(d,g,s) for g in sep.vertices for s in support)*1000
    rows.append(dict(file=path.name,actual_angles_sim_rad=angles.tolist(),max_joint_difference_from_simulated_settled_start_rad=float(np.max(abs(angles-reference[:16]))),
                     housing_clearance_lower_bound_mm=gap,modeled_support_contacts=contacts))
report=dict(status='offline_measured_joint_pose_geometry_audit',motor_commands_sent=0,results=rows,
            caveat='Measured joints with assumed CAD/support registration. Does not replace observed real clearance or establish a loaded cube pose.')
(ROOT/'sac_measured_start_geometry_audit_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps([{k:v for k,v in row.items() if k!='actual_angles_sim_rad'} for row in rows],indent=2))
