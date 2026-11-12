"""Saved-pose comparison of exact slab clipping and separating witnesses."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore,inspect_for
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate
folder=ROOT/'full_placement/front-10';m=mujoco.MjModel.from_binary_path(str(folder/'g1_f0.3/model.mjb'));hand,_,_=inspect_for(m);cases=[]
for name in ('g1_f0.3','g0.5_f0.2','g0.5_f0.5'):
    d=restore(m,np.load(folder/name/'end_state.npz'));cube=m.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);cl=gaps(m,d,C,B,precompute(m,hand),True);cert=SweepCertificate(m,hand,.3);gap,bad=cert.query(d,C,B);cases.append(dict(case=name,clipped_nearest=sorted(cl,key=lambda r:r[1])[:8],minimum_certificate_gap_mm=gap,not_certified_geoms=[m.geom(int(cert.ids[i])).name for i in bad],certificate=cert.metadata()))
(ROOT/'final_sweep_diagnostic.json').write_text(json.dumps(dict(cases=cases,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(cases),flush=True)
