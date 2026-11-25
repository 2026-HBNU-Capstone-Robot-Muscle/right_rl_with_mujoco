"""Plan alternative cube placements for the current grip; never fit observations.

The output is a desired placement feasibility study, not a corrected estimate
of where the user's cube was. No joint target or runtime transform is exported.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    blob=(FOLDER/'direct_reference_cube_pose.json').read_bytes();reference=json.loads(blob)
    trial=json.loads((FOLDER/'result.json').read_bytes())
    h=min(reference['hypotheses'],key=lambda x:x['marker_error_px'])
    center=np.array(h['cube_center_world_m']);basis=np.array(h['cube_rotation_world'])
    q=ticks_to_sim(reference['measured_motor_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id
    tips=[m.geom(name).id for name in ('if_tip','mf_tip','th_tip')]
    limits=np.r_[[12.]*3,[8.]*3]
    def pose(x):
        R=Rotation.from_rotvec(x[3:]*np.pi/180).as_matrix()@basis
        point=center+x[:3]/1000
        d.qpos[:16]=q;d.qpos[-7:]=np.r_[point,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
        return point,R
    def measure(x):
        point,R=pose(x)
        gaps=np.array([mujoco.mj_geomDistance(m,d,cube,g,1,None)*1000 for g in tips])
        depth=np.zeros(m.ngeom);contacts=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube in (a,b):
                other=b if a==cube else a;overlap=max(0.,-float(c.dist)*1000)
                depth[other]=max(depth[other],overlap)
                contacts.append(dict(other=m.geom(other).name,overlap_mm=overlap))
        return gaps,depth,point,R,contacts
    def residual(x):
        gaps,depth,_,_,_=measure(x)
        return np.r_[(gaps-.1)*2,np.maximum(depth-.8,0)*8,x/limits*.08]
    rng=np.random.default_rng(20261023);rows=[]
    for i in range(12):
        x=np.zeros(6) if i==0 else rng.uniform(-.45,.45,6)*limits
        fit=least_squares(residual,x,bounds=(-limits,limits),diff_step=1e-3,max_nfev=150)
        gaps,depth,point,R,contacts=measure(fit.x)
        row=dict(index=i,cost=float(fit.cost),desired_translation_world_mm=fit.x[:3].tolist(),
            desired_rotation_vector_world_deg=fit.x[3:].tolist(),desired_cube_center_world_m=point.tolist(),
            desired_cube_rotation_world=R.tolist(),tip_gaps_mm=gaps.tolist(),
            max_overlap_mm=float(max(depth)),contacts=contacts,
            geometric_three_tip_feasible=bool(max(abs(gaps))<.8 and max(depth)<1.5))
        rows.append(row)
        print('PLACEMENT',i,'delta',np.round(fit.x,2).tolist(),'gaps',np.round(gaps,2).tolist(),
              'overlap',round(max(depth),2),'feasible',row['geometric_three_tip_feasible'],flush=True)
    rows.sort(key=lambda r:r['cost'])
    output=dict(status='desired_cube_placement_feasibility_not_observation_fit',source=FOLDER.name,reference_sha256=hashlib.sha256(blob).hexdigest(),
        rows=rows,model_manifest=manifest,registration_verified=False,joint_goals_changed=False,
        runtime_transform_changed=False,hardware_execution_allowed=False,motor_commands_sent=0,
        conclusion='Desired placements only. Open-hand stability, approach dynamics and placement uncertainty still need independent checks; never overwrite the measured pose with these targets.')
    (FOLDER/'desired_placement_search.json').write_text(json.dumps(output,indent=2),encoding='utf-8')

if __name__=='__main__':main()
