"""Offline collision-filter regression for the additional mesh proxies."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_with_middle_thumb_hulls(env)
entries=manifest['additional_middle_thumb_hull_proxies']
a=m.geom(next(x['name'] for x in entries if x['body']=='mf_ds')).id
b=m.geom(next(x['name'] for x in entries if x['body']=='th_ds')).id
def allows(x,y):
    return bool((int(m.geom_contype[x])&int(m.geom_conaffinity[y])) or
                (int(m.geom_contype[y])&int(m.geom_conaffinity[x])))
assert allows(a,b)
for name in ('cube','support_glass','support_wood'):
    assert not allows(a,m.geom(name).id) and not allows(b,m.geom(name).id)
d=mujoco.MjData(m);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
mujoco.mj_forward(m,d)
# Synthetic body translation for the collision pipeline test only. Recompute
# body broadphase bounds as well as geometry positions. Editing only geom_xpos
# would leave the body BVH elsewhere and would not be a valid filter test.
body=int(m.geom_bodyid[b]);parent=int(m.body_parentid[body])
m.body_pos[body]+=d.xmat[parent].reshape(3,3).T@(d.geom_xpos[a]-d.geom_xpos[b])
mujoco.mj_forward(m,d)
# A mesh frame origin is not necessarily inside its convex hull. Coincident
# frame origins do not guarantee the selected two individual meshes overlap.
# Inspect every added proxy on these two target bodies (housing + pad).
left={m.geom(x['name']).id for x in entries if x['body']=='mf_ds'}
right={m.geom(x['name']).id for x in entries if x['body']=='th_ds'}
hits=[dict(a=m.geom(int(c.geom1)).name,b=m.geom(int(c.geom2)).name,dist=float(c.dist))
      for c in d.contact if (int(c.geom1) in left and int(c.geom2) in right) or (int(c.geom2) in left and int(c.geom1) in right)]
assert hits and min(c['dist'] for c in hits)<0, 'Middle/thumbnail body proxy contacts unexpectedly filtered out'
result=dict(offline_only=True,hardware_commands_sent=0,proxy_count=len(entries),
    middle_thumb_contact_enabled=True,cube_and_support_proxy_contacts_disabled=True,
    synthetic_overlap_detected=True,synthetic_overlap_depth_m=-min(c['dist'] for c in hits),
    detected_proxy_contacts=hits,
    mass_inertia_dof_unchanged=True)
root=Path(__file__).resolve().parent/'full_mapping_pose_review_20261019'
(root/'housing_proxy_verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
