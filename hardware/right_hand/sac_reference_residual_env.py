"""SAC position corrections around an offline candidate motion; not replay=SAC."""
import hashlib
import json
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
ROOT=Path(__file__).resolve().parent

class ReferenceResidual:
    def __init__(self,env,reference,scale=.05):
        self.env=env;path=ROOT/reference;report=json.loads(path.read_text())
        if report.get('control_period_seconds')!=float(env.dt):raise RuntimeError('Reference control period mismatch')
        if not any(r['success'] for r in report['validation']):raise RuntimeError('No successful native reference')
        if env._config.ema_alpha!=1 or env._config.action_delay_steps!=0:raise RuntimeError('Reference inverse requires no EMA/delay')
        self.reference_sha256=hashlib.sha256(path.read_bytes()).hexdigest();self.scale=scale
        resets=json.loads((ROOT/'sac_supported_reset_result.json').read_text());base=np.asarray(resets['candidates'][0]['ctrl'])
        delta=np.asarray(report['validation'][0]['delta']).reshape(-1,16)
        self.base=jp.asarray(base);self.targets=jp.asarray(np.clip(base+delta,env._safe_lowers,env._safe_uppers))
        self.move_seconds=len(delta)*1.2;self.env.required_hold_steps=23

    def __getattr__(self,name):return getattr(self.env,name)

    @property
    def observation_size(self):return jax.tree_util.tree_map(lambda x:x.shape,jax.eval_shape(self.reset,jax.random.PRNGKey(0)).obs)

    def _observe(self,state):
        elapsed=state.info['reference_step']*self.dt
        phase=jp.stack([jp.minimum(elapsed/self.move_seconds,1.),(elapsed>=self.move_seconds).astype(float)])
        return state.replace(obs={k:jp.concatenate([v,phase]) for k,v in state.obs.items()})

    def _initialize(self,state):
        info=dict(state.info);info['reference_step']=jp.asarray(0,dtype=jp.int32)
        return self._observe(state.replace(info=info))

    def reset(self,rng):return self._initialize(self.env.reset(rng))
    def reset_at(self,rng,index):return self._initialize(self.env.reset_at(rng,index))

    def desired_target(self,step):
        elapsed=(step+1)*self.dt;phase=jp.minimum(jp.floor(elapsed/1.2).astype(jp.int32),len(self.targets)-1)
        previous=jp.where(phase==0,self.base,self.targets[jp.maximum(phase-1,0)])
        fraction=jp.clip((elapsed-phase*1.2)/1.2,0.,1.)
        return previous+(self.targets[phase]-previous)*fraction

    def step(self,state,action):
        count=state.info['reference_step']
        if 'steps' in state.info:count=jp.where(state.info['steps']==0,0,count)
        desired=jp.clip(self.desired_target(count)+self.scale*jp.clip(action,-1,1),self._safe_lowers,self._safe_uppers)
        executed=jp.clip((desired-state.data.ctrl)/self._config.action_scale,-1,1)
        # Base environment rebuilds the observation, so phase is appended once.
        next_state=self.env.step(state,executed);info=dict(next_state.info);info['reference_step']=count+1
        return self._observe(next_state.replace(info=info))
