"""Pinned, audited initial cube-placement pose; experimental simulation only."""
import hashlib
import json
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
from mujoco import mjx
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent


class Stage0StartFaceUp:
    def __init__(self,env,offset_index=None):
        self.env=env
        path=ROOT/'sac_stage0_reset_audit_result.json'
        report=json.loads(path.read_text())
        for key,name in report['sources'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=report['sha256'][key]:
                raise RuntimeError(f'Reset audit source changed: {name}')
        if report['joint_limit_violations']:
            raise RuntimeError('Stage-0 reset violates configured joint limits')
        self.reset_manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        self.joints=jp.asarray(report['joint_angles_sim_rad'])
        self.cube_center=jp.asarray(report['cube_center_m'])
        self.cube_quat=jp.asarray(report['cube_quat_wxyz'])
        candidates=[row for row in report['candidates']
                    if row['kinematic_fixed_joints']['initial_geoms']
                    and row['kinematic_fixed_joints']['penetration_mm']<=2
                    and row['dynamic_hold_drift_mm']<=10
                    and row['dynamic_final_contacts']]
        self.offsets_m=jp.asarray(np.array([row['offset_mm'] for row in candidates])/1000)
        if not candidates or (offset_index is not None and not 0<=offset_index<len(candidates)):
            raise ValueError('No valid candidate or invalid fixed offset index')
        self.offset_index=offset_index
        self.friction_scale=1.
        model=env.mj_model
        set_cube_physics(model,mass_kg=.100)
        model.opt.gravity[:]=report['gravity_m_s2']
        env._mjx_model=mjx.put_model(model,impl='jax')
        env._cube_mass=model.body_subtreemass[model.body('cube').id]

    def __getattr__(self,name): return getattr(self.env,name)

    def reset(self,rng):
        rng,pick=jax.random.split(rng)
        index=(jax.random.randint(pick,(),0,len(self.offsets_m)) if self.offset_index is None
               else jp.asarray(self.offset_index,dtype=jp.int32))
        return self.reset_at(rng,index)

    def reset_at(self,rng,index):
        state=self.env.reset(rng)
        qpos=state.data.qpos.at[:16].set(self.joints)
        qpos=qpos.at[-7:-4].set(self.cube_center+self.offsets_m[index])
        qpos=qpos.at[-4:].set(self.cube_quat)
        data=state.data.replace(qpos=qpos,qvel=jp.zeros_like(state.data.qvel),ctrl=self.joints)
        data=mjx.forward(self.env.mjx_model,data)
        info=dict(state.info)
        info['stage0_offset_index']=index
        info['motor_targets']=data.ctrl
        info['start_cube_pos']=data.qpos[-7:-4]
        for key in ('qpos_error_history','cube_pos_error_history','cube_ori_error_history'):
            info[key]=jp.zeros_like(info[key])
        err=self.env._cube_orientation_error(data,info)
        info['prev_ori_error']=err; info['best_ori_error']=err
        obs=self.env._get_obs(data,info)
        return state.replace(data=data,obs=obs,info=info)

    def step(self,state,action): return self.env.step(state,action)
