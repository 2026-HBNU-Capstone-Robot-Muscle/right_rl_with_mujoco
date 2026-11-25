"""Compare saved MJX reference replay to native geometry, without hardware."""
import json
from pathlib import Path
import numpy as np
import mujoco
import inspect
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    ref=json.loads((ROOT/'sac_supported_motion_search_20hz_result.json').read_text())
    replay=json.loads((ROOT/'sac_supported_motion_search_20hz_result_mjx_eval.json').read_text())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);model,_=build(env)
    support={model.geom(n).id for n in ('support_glass','support_wood')}
    native={r['step']//5:r for r in ref['validation'][0]['trace']}
    rows=[]
    for row in replay['results'][0]['trace']:
        d=mujoco.MjData(model);d.qpos[:]=row['qpos'];d.ctrl[:]=row['ctrl'];mujoco.mj_forward(model,d)
        contacts=[dict(pair=[model.geom(c.geom1).name,model.geom(c.geom2).name],depth_mm=-float(c.dist)*1000)
                  for c in d.contact if c.geom1 in support or c.geom2 in support]
        r=dict(step=row['step'],native_contacts=contacts,mjx_depth_mm=row.get('support_penetration_mm'))
        if row['step'] in native:
            other=native[row['step']];r['max_ctrl_difference']=float(np.max(np.abs(np.array(row['ctrl'])-other['ctrl'])))
            r['max_joint_difference']=float(np.max(np.abs(np.array(row['qpos'])[:16]-np.array(other['qpos'])[:16])))
        rows.append(r)
    # Detect Python environment initialization changing the freshly built model.
    cfg2=registry.get_default_config('LeapCubeFaceUp');cfg2.impl='jax'
    from sac_supported_start_env import SupportedStart
    wrapped=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg2));other=wrapped.mj_model
    changed={}
    for name in dir(model):
        a=getattr(model,name);b=getattr(other,name,None)
        if isinstance(a,np.ndarray) and isinstance(b,np.ndarray) and a.shape==b.shape and np.issubdtype(a.dtype,np.number):
            if not np.allclose(a,b,equal_nan=True):changed[name]=dict(native=a.tolist(),environment=b.tolist())
    for name in dir(model.opt):
        if name.startswith('_'):continue
        a=getattr(model.opt,name);b=getattr(other.opt,name)
        if isinstance(a,(int,float,np.ndarray)) and not np.allclose(a,b,equal_nan=True):changed['opt.'+name]=dict(native=np.asarray(a).tolist(),environment=np.asarray(b).tolist())
    result=dict(rows=rows,model_changes_after_initialization=changed,motor_commands_sent=0)
    (ROOT/'sac_reference_engine_audit_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
