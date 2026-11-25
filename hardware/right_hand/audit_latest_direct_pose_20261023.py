"""Audit new saved-image hypotheses in the native hand model; no devices/export."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    blob=(FOLDER/'direct_reference_cube_pose.json').read_bytes();reference=json.loads(blob)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id
    d.qpos[:16]=ticks_to_sim(reference['measured_motor_ticks'])
    hypotheses=[]
    for h in reference['hypotheses']:
        R=np.array(h['cube_rotation_world']);center=np.array(h['cube_center_world_m'])
        quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
        d.qpos[-7:]=np.r_[center,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
        gaps={name:float(mujoco.mj_geomDistance(m,d,cube,m.geom(name).id,1,None))*1000
              for name in ('if_tip','mf_tip','rf_tip','th_tip')}
        contacts=[]
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)):
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                contacts.append(dict(other=m.geom(other).name,overlap_mm=max(0.,-float(c.dist)*1000)))
        hypotheses.append(dict(**h,tip_gaps_mm=gaps,cube_contacts=contacts))
        print('BRANCH',h['palm_ippe_branch'],'GAPS',gaps,'CONTACTS',contacts,flush=True)
    tips=[]
    signs=np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)])
    for name in ('if_tip','mf_tip','rf_tip','th_tip'):
        g=m.geom(name).id
        tips.append(dict(name=name,center_world_m=d.geom_xpos[g].tolist(),
            box_vertices_world_m=((signs*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]).tolist()))
    report=dict(source=FOLDER.name,reference_sha256=hashlib.sha256(blob).hexdigest(),status='native_direct_palm_cube_geometry_diagnostic',
        reference_source=reference['palm_reference_source'],hypotheses=hypotheses,
        joint_anchors_world_m={m.joint(i).name:d.xanchor[i].tolist() for i in range(16)},
        tips=tips,model_manifest=manifest,registration_verified=False,
        hardware_execution_allowed=False,motor_commands_sent=0,
        limitation='Manual image borders and provisional palm mounting. Apparent overlaps/distances are model diagnostics, not measured contact or executable closure.')
    (FOLDER/'direct_pose_geometry_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
