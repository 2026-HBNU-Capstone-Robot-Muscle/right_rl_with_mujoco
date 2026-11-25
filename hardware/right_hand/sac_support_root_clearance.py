"""Find modeled root-angle clearance without moving the real hand."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);model,manifest=build(env)
    audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    data=mujoco.MjData(model);data.qpos[:16]=audit['joint_angles_sim_rad']
    data.qpos[-7:-4]=audit['cube_center_m'];data.qpos[-4:]=audit['cube_quat_wxyz'];data.ctrl[:]=data.qpos[:16]
    original=data.qpos.copy();obstacles=[model.geom('support_glass').id,model.geom('support_wood').id]
    def gap(prefix):
        ids=[model.geom(prefix+s+'_visual_support_proxy').id for s in ('_bs','_px')]
        return min(float(mujoco.mj_geomDistance(model,data,g,o,.2,np.zeros(6))) for g in ids for o in obstacles)
    rows=[];candidate=original.copy()
    for index,prefix in ((0,'if'),(4,'mf'),(8,'rf')):
        lower,upper=np.asarray(env._safe_lowers)[index],np.asarray(env._safe_uppers)[index]
        q0=float(original[index]); samples=[]
        for q in np.linspace(lower,upper,241):
            data.qpos[:]=original;data.qpos[index]=q;mujoco.mj_forward(model,data)
            samples.append(dict(q_rad=float(q),clearance_mm=1000*gap(prefix)))
        data.qpos[:]=original;mujoco.mj_forward(model,data)
        valid=[r for r in samples if r['clearance_mm']>=2.]
        nearest=min(valid,key=lambda r:abs(r['q_rad']-q0)) if valid else None
        if nearest:candidate[index]=nearest['q_rad']
        rows.append(dict(joint=model.joint(index).name,hardware_motor_id={0:5,4:9,8:13}[index],
                         initial_q_rad=q0,initial_clearance_mm=1000*gap(prefix),nearest_clear=nearest,samples=samples))
    data.qpos[:]=candidate;data.ctrl[:]=candidate[:16];mujoco.mj_forward(model,data)
    support_contacts=[dict(geoms=[model.geom(c.geom1).name,model.geom(c.geom2).name],distance_mm=float(c.dist*1000))
                      for c in data.contact if (c.geom1 in obstacles or c.geom2 in obstacles) and c.dist<0]
    initial_cube=data.qpos[-7:-4].copy();initial_q=data.qpos.copy()
    for _ in range(round(.3/model.opt.timestep)):mujoco.mj_step(model,data)
    result=dict(status='offline_support_clearance_candidate_NOT_hardware_command',motor_commands_sent=0,model=manifest,
                joint_scans=rows,candidate_qpos=initial_q.tolist(),candidate_initial_support_contacts=support_contacts,
                dynamic_hold_cube_drift_mm=float(1000*np.linalg.norm(data.qpos[-7:-4]-initial_cube)),
                dynamic_hold_joint_drift_rad=float(np.max(abs(data.qpos[:16]-initial_q[:16]))),
                caveat='Root angles found against conservative CAD convex housing proxies. Not a verified real posture or collision-free trajectory. Other contacts and cube support require evaluation.')
    (ROOT/'sac_support_root_clearance_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('joint_scans','candidate_qpos')},indent=2))
    print(json.dumps([{k:v for k,v in r.items() if k!='samples'} for r in rows],indent=2))

if __name__=='__main__':main()
