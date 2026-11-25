"""Audit actual contact candidate pairs after MJX model conversion."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import json
from pathlib import Path
import numpy as np
import mujoco
import jax
from mujoco import mjx
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg));m=env.mj_model
r=json.loads((ROOT/'sac_supported_reset_result.json').read_text());d=mujoco.MjData(m);d.qpos[:]=r['candidates'][0]['qpos'];d.ctrl[:]=r['candidates'][0]['ctrl'];mujoco.mj_forward(m,d)
j=jax.jit(mjx.forward)(env.mjx_model,mjx.put_data(m,d));geom=np.asarray(j.contact.geom);dist=np.asarray(j.contact.dist)
rows=[]
for c in d.contact:
    ids=[int(c.geom1),int(c.geom2)];idx=np.flatnonzero(np.all(np.sort(geom,axis=-1)==sorted(ids),axis=-1))
    rows.append(dict(pair=[m.geom(g).name for g in ids],native_dist=float(c.dist),mjx_dist=dist[idx].tolist(),candidate_count=len(idx)))
result=dict(native_contact_pairs=rows,mjx_candidate_slots=len(geom),disabled_flags=int(m.opt.disableflags),
            thumb_geoms=[dict(name=m.geom(i).name,body=int(m.geom_bodyid[i]),type=int(m.geom_type[i]),contype=int(m.geom_contype[i]),affinity=int(m.geom_conaffinity[i])) for i in range(m.ngeom) if m.geom(i).name=='th_tip' or 'palm_collision' in m.geom(i).name or 'mount_collision' in m.geom(i).name])
(ROOT/'sac_collision_pair_probe_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
