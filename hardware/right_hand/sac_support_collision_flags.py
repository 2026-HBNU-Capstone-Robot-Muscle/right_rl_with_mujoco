import json
from pathlib import Path
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);model,_=build(env)
data=mujoco.MjData(model)
source=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text())
data.qpos[:]=source['results'][0]['trace'][0]['qpos'];mujoco.mj_forward(model,data)
print('flags',model.opt.disableflags,model.opt.enableflags,'pairs',model.npair,'contacts',data.ncon)
for name in ('if_bs_collision_1','if_bs_visual','cube','support_glass','support_wood'):
    gid=model.geom(name).id
    print(name,'body',model.geom_bodyid[gid],'mask',model.geom_contype[gid],model.geom_conaffinity[gid], 'size',model.geom_size[gid],'margin',model.geom_margin[gid],model.geom_gap[gid])
print('exclude',model.exclude_signature.tolist())
print([(model.geom(c.geom1).name,model.geom(c.geom2).name,c.dist) for c in data.contact])
print(Path(env.xml_path).read_text()[:4000])
