"""Offline model contacts at measured/commanded stopping poses, no hardware."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
ROOT=Path(__file__).resolve().parent
ORDER=np.array(list(range(4,16))+list(range(4)))
TICK=2*np.pi/4096

def main():
    hw=json.loads((ROOT/'leap_sac_path_empty_result.json').read_text())
    row=[r for r in hw['samples'] if r['phase']=='reachable_empty_path'][-1]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    m=env.mj_model;cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    base=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'][0]['qpos']
    results=[]
    for source in ('actual','goal'):
        d=mujoco.MjData(m);d.qpos[:]=base;d.qpos[:16]=np.array(row[source])[ORDER]*TICK-np.pi
        d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        contacts=[]
        for c in d.contact:
            if c.dist<0:
                contacts.append(dict(a=m.geom(c.geom1).name,b=m.geom(c.geom2).name,
                    body_a=m.body(m.geom_bodyid[c.geom1]).name,body_b=m.body(m.geom_bodyid[c.geom2]).name,
                    penetration_mm=-float(c.dist)*1000))
        results.append(dict(source=source,contacts=contacts))
    jid=int(m.actuator_trnid[7,0])
    out=dict(hardware_timestamp=hw['timestamp'],id12_joint=m.joint(jid).name,
        actuator_joint_map=[dict(index=i,joint=m.joint(int(m.actuator_trnid[i,0])).name,
                                qpos_address=int(m.jnt_qposadr[int(m.actuator_trnid[i,0])]),
                                kp=float(m.actuator_gainprm[i,0]),bias=m.actuator_biasprm[i,:3].tolist()) for i in range(16)],
        simulated_joint_range_rad=m.jnt_range[jid].tolist(),results=results,
        physical_contact_verified=False,motor_commands_sent=0,
        caveat='Static contact reconstruction only. Model has no cable geometry; neither physical contact nor clearance is established.')
    (ROOT/'sac_id12_contact_snapshot_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out),flush=True)

if __name__=='__main__':main()
