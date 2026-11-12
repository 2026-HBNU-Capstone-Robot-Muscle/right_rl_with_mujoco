"""Audit the archived placement pose before directional SAC training; no hardware."""
import hashlib
import json
import sys
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import contact_geoms, probe

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'work'/'Leap_Test'/'deploy'))
from leap_hw import hw_to_sim, SIM_JOINT_NAMES


def main():
    files=dict(trial='leap_sequence_flip_20261010_045958_782877.json',
               registration='sac_cube_rim_register_1010_stage0_result.json',
               gravity='table_tilt_20261010_053815_726640/tilt_result.json')
    sources={key:json.loads((ROOT/value).read_text(encoding='utf-8')) for key,value in files.items()}
    trial,reg,tilt=(sources[key] for key in ('trial','registration','gravity'))
    samples=[r for r in trial['samples'] if r['phase']=='place_cube']
    sample=samples[-1]
    joints=hw_to_sim(np.asarray(sample['actual'])*2*np.pi/4096)
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    model=env.mj_model
    set_cube_physics(model,mass_kg=.100)
    model.opt.gravity[:]=tilt['gravity_in_sim_world_m_s2_candidate']
    limits=np.stack((np.asarray(env._safe_lowers),np.asarray(env._safe_uppers)),axis=1)
    violations=[dict(joint=name,q_rad=float(q),limits_rad=lim.tolist())
                for name,q,lim in zip(SIM_JOINT_NAMES,joints,limits) if not lim[0]<=q<=lim[1]]
    cube=model.geom('cube').id
    center=np.asarray(reg['cube_center_world_m'])
    quat=np.asarray(reg['cube_quat_world_candidate'])
    offsets=[[0,0,0],[2.5,0,0],[5,0,0],[2.5,-2.5,0],[2.5,2.5,0],
             [2.5,0,2.5],[2.5,0,-2.5]]
    rows=[]
    for offset in offsets:
        data=mujoco.MjData(model)
        data.qpos[:16]=joints; data.qpos[-7:-4]=center+np.asarray(offset)/1000
        data.qpos[-4:]=quat; data.ctrl[:]=joints
        mujoco.mj_forward(model,data)
        kinematic=probe(model,data.qpos.copy(),cube)
        before=data.qpos.copy()
        for _ in range(round(.3/model.opt.timestep)):
            mujoco.mj_step(model,data)
        contacts=contact_geoms(model,data,cube)
        rows.append(dict(offset_mm=offset,kinematic_fixed_joints=kinematic,
                         dynamic_hold_drift_mm=float(1000*np.linalg.norm(data.qpos[-7:-4]-before[-7:-4])),
                         dynamic_joint_max_drift_rad=float(np.max(np.abs(data.qpos[:16]-before[:16]))),
                         dynamic_final_contacts=sorted({name for _,name in contacts})))
    manifest=dict(status='offline_stage0_placement_reset_audit',motor_commands_sent=0,
                  start_phase='place_cube_after_stage_0',sources=files,
                  sha256={k:hashlib.sha256((ROOT/v).read_bytes()).hexdigest() for k,v in files.items()},
                  source_sample_t=sample['t'],joint_angles_sim_rad=joints.tolist(),
                  cube_center_m=center.tolist(),cube_quat_wxyz=quat.tolist(),
                  gravity_m_s2=model.opt.gravity.tolist(),joint_limit_violations=violations,
                  zero_action_initial_clip_max_rad=float(np.max(np.abs(np.clip(joints,limits[:,0],limits[:,1])-joints))),
                  default_action_scale=float(cfg.action_scale),default_ema_alpha=float(cfg.ema_alpha),
                  actuator_force_ranges=model.actuator_forcerange.tolist(),candidates=rows,
                  caveat='Same archived placement session; approximate camera-to-model registration. External wood not calibrated or included. No real-hand policy validation.')
    (ROOT/'sac_stage0_reset_audit_result.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__': main()
