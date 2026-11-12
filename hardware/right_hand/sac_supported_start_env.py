"""Experimental support-aware SAC environment. No live controller imports."""
import hashlib
import json
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
from mujoco import mjx
from sac_wood_glass_model import build
from sac_forward_task_env import ForwardFirstFlip
ROOT=Path(__file__).resolve().parent

class SupportedStart:
    dynamics_version='support_contacts_v2_all_pairs_128'
    def __init__(self,env,diagnostic_cube_side_mm=None):
        self.env=env;path=ROOT/'sac_supported_reset_result.json';report=json.loads(path.read_text())
        if not report['all_passed']:raise RuntimeError('Native supported reset validation failed')
        for name,digest in report['sources_sha256'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Reset source changed: '+name)
        self.reset_manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        model,self.support_manifest=build(env)
        self.diagnostic_cube_physics=None
        if diagnostic_cube_side_mm is not None:
            if not np.isfinite(diagnostic_cube_side_mm) or not 40<=diagnostic_cube_side_mm<=80:
                raise ValueError('Diagnostic cube side must be finite and within 40..80mm')
            from sac_cube_model import set_cube_physics
            self.diagnostic_cube_physics=set_cube_physics(model,side_m=diagnostic_cube_side_mm/1000,mass_kg=.100)
        # Large support boxes dominate the default bounding-sphere ranking.
        # The original 12-pair broadphase budget silently removes actual hand
        # contacts. Test all eligible pairs before retaining deepest contacts.
        model.numeric('max_geom_pairs').data[:]=-1
        model.numeric('max_contact_points').data[:]=128
        self.root_floor=float(report['root_command_floor_rad']);model.actuator_ctrlrange[[0,4,8],0]=self.root_floor
        env._mj_model=model;env._post_init()
        env._safe_lowers=np.array(env._safe_lowers);env._safe_lowers[[0,4,8]]=np.maximum(env._safe_lowers[[0,4,8]],self.root_floor)
        env._mjx_model=mjx.put_model(model,impl='jax')
        self.initial_q=jp.asarray([r['qpos'] for r in report['candidates']]);self.initial_ctrl=jp.asarray([r['ctrl'] for r in report['candidates']])
        self.offsets_m=jp.asarray([r['offset_mm'] for r in report['candidates']])/1000
        self.friction_scale=1.;self.support_ids=jp.asarray([model.geom(n).id for n in ('support_glass','support_wood')])

    def __getattr__(self,name):return getattr(self.env,name)

    def reset(self,rng):
        rng,key=jax.random.split(rng);return self.reset_at(rng,jax.random.randint(key,(),0,len(self.offsets_m)))

    def reset_at(self,rng,index):
        state=self.env.reset(rng);ctrl=self.initial_ctrl[index]
        data=state.data.replace(qpos=self.initial_q[index],qvel=jp.zeros_like(state.data.qvel),ctrl=ctrl)
        data=mjx.forward(self.env.mjx_model,data);info=dict(state.info)
        info['stage0_offset_index']=index;info['motor_targets']=ctrl;info['start_cube_pos']=data.qpos[-7:-4]
        for key in ('qpos_error_history','cube_pos_error_history','cube_ori_error_history'):info[key]=jp.zeros_like(info[key])
        err=self.env._cube_orientation_error(data,info);info['prev_ori_error']=err;info['best_ori_error']=err
        return state.replace(data=data,info=info,obs=self.env._get_obs(data,info))

    def step(self,state,action):return self.env.step(state,action)


class SupportedForward(ForwardFirstFlip):
    required_hold_steps=3
    def _initialize(self,state):
        state=super()._initialize(state);metrics=dict(state.metrics)
        metrics['support/max_penetration_mm']=jp.asarray(0.);metrics['support/fault']=jp.asarray(0.)
        metrics['support/contact_buffer_full']=jp.asarray(0.)
        return state.replace(metrics=metrics)

    def step(self,state,action):
        state=super().step(state,action)
        original_success=state.metrics['forward/success']
        achieved=(state.info['forward_goal_hold']>=self.required_hold_steps).astype(float)
        physical=(state.data.qpos[-5]<-.05)|~jp.all(jp.isfinite(state.data.qpos))|~jp.all(jp.isfinite(state.data.qvel))
        state=state.replace(done=jp.maximum(achieved,physical.astype(float)),
                            reward=state.reward+20*(achieved-original_success))
        contacts=state.data.contact
        mask=jp.any(contacts.geom[...,None]==self.env.support_ids,axis=(-1,-2))
        penetration=jp.max(jp.where(mask,jp.maximum(-contacts.dist,0),0),initial=0.)
        fault=penetration>.0005
        # If every retained slot is penetrating, omitted contacts are unknown.
        # End the rollout instead of optimizing against a saturated contact set.
        saturated=jp.all(contacts.dist<=0)
        fault=fault|saturated
        metrics=dict(state.metrics);metrics['support/max_penetration_mm']=penetration*1000;metrics['support/fault']=fault.astype(float)
        metrics['support/contact_buffer_full']=saturated.astype(float)
        metrics['forward/success']=jp.where(fault,0.,achieved)
        metrics['reward/success']=jp.where(fault,0.,achieved)
        info=dict(state.info);info['forward_goal_hold']=jp.where(fault,0,info['forward_goal_hold'])
        return state.replace(info=info,metrics=metrics,done=jp.maximum(state.done,fault.astype(float)),reward=jp.where(fault,-20.,state.reward))
