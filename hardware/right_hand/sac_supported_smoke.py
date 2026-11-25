import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import json
import time
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart,SupportedForward
ROOT=Path(__file__).resolve().parent
started=time.monotonic()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';cfg.chain_goals=False;cfg.face_onehot_obs=False
env=SupportedForward(SupportedStart(registry.load('LeapCubeFaceUp',config=cfg)))
print('Model built; compiling reset and step',flush=True)
reset=jax.jit(env.reset_at);step=jax.jit(env.step);rows=[]
for i in range(len(env.offsets_m)):
    state=reset(jax.random.PRNGKey(0),jp.asarray(i));start=np.asarray(state.data.qpos[-7:-4]);max_drift=0.;max_pen=0.;fault=False
    initial_pen=float(jp.max(jp.maximum(-state.data.contact.dist,0),initial=0))*1000
    for t in range(40):
        state=step(state,jp.zeros(env.action_size));max_drift=max(max_drift,float(np.linalg.norm(np.asarray(state.data.qpos[-7:-4])-start))*1000)
        max_pen=max(max_pen,float(state.metrics['support/max_penetration_mm']));fault=fault or bool(state.metrics['support/fault'])
        if bool(state.done):break
    row=dict(offset_index=i,steps=t+1,initial_penetration_mm=initial_pen,max_cube_drift_mm=max_drift,max_support_penetration_mm=max_pen,
             support_fault=fault,done=bool(state.done),finite=bool(jp.all(jp.isfinite(state.data.qpos))),final_qpos=np.asarray(state.data.qpos).tolist())
    rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='final_qpos'}),flush=True)
report=dict(status='offline_supported_MJX_zero_action_smoke',motor_commands_sent=0,elapsed_seconds=time.monotonic()-started,
            dynamics_version=env.dynamics_version,
            reset_manifest_sha256=env.reset_manifest_sha256,results=rows,
            passed=all(r['finite'] and not r['done'] and r['initial_penetration_mm']<=2 and r['max_cube_drift_mm']<=10 for r in rows))
(ROOT/'sac_supported_smoke_result.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASSED',report['passed'],'elapsed',report['elapsed_seconds'],flush=True)
