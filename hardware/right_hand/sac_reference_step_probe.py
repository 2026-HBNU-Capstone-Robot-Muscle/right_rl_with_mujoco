"""Native/MJX one-substep dynamics comparison; no hardware."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import json
from pathlib import Path
import numpy as np
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart,SupportedForward
from sac_reference_residual_env import ReferenceResidual
ROOT=Path(__file__).resolve().parent

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';cfg.chain_goals=False;cfg.face_onehot_obs=False
    env=ReferenceResidual(SupportedForward(SupportedStart(registry.load('LeapCubeFaceUp',config=cfg))),'sac_supported_motion_search_20hz_result.json')
    model=env.mj_model;state=jax.jit(env.reset_at)(jax.random.PRNGKey(0),jp.asarray(0))
    native=mujoco.MjData(model);native.qpos[:]=np.asarray(state.data.qpos);native.ctrl[:]=np.asarray(state.data.ctrl);mujoco.mj_forward(model,native)
    data=mjx.put_data(model,native);step=jax.jit(mjx.step);rows=[]
    print('Compiling one physics step',flush=True)
    for k in range(20):
        ctrl=np.asarray(env.desired_target(jp.asarray(k//5)))
        native.ctrl[:]=ctrl;mujoco.mj_step(model,native)
        data=step(env.mjx_model,data.replace(ctrl=jp.asarray(ctrl)))
        error=np.asarray(data.qpos)-native.qpos
        row=dict(step=k+1,joint_error=error[:16].tolist(),native_qpos=native.qpos.tolist(),mjx_qpos=np.asarray(data.qpos).tolist(),
                 native_force=native.actuator_force.tolist(),mjx_force=np.asarray(data.actuator_force).tolist(),
                 native_qacc=native.qacc.tolist(),mjx_qacc=np.asarray(data.qacc).tolist(),
                 native_contacts=[dict(pair=[model.geom(c.geom1).name,model.geom(c.geom2).name],dist=float(c.dist)) for c in native.contact],
                 mjx_contacts=[dict(pair=[model.geom(int(g)).name for g in geom],dist=float(dist)) for geom,dist in zip(np.asarray(data.contact.geom),np.asarray(data.contact.dist)) if dist<.001])
        rows.append(row);print(json.dumps(dict(step=k+1,max_joint_error=float(np.max(np.abs(error[:16]))))),flush=True)
    report=dict(n_substeps=env.n_substeps,model_timestep=model.opt.timestep,rows=rows)
    (ROOT/'sac_reference_step_probe_result.json').write_text(json.dumps(report,indent=2)+'\n')
    for i in (0,4,19):
        row=rows[i]
        print(json.dumps(dict(step=row['step'],joint_error=row['joint_error'],native_force=row['native_force'],mjx_force=row['mjx_force'],
                              native_contacts=row['native_contacts'],mjx_contacts=row['mjx_contacts'])),flush=True)
if __name__=='__main__':main()
