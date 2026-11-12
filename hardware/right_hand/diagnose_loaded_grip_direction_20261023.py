"""Local contact geometry directions, analysis only; never a motor target file."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
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
FOLDER=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'


def main():
    reg=json.loads((FOLDER/'full_rim_constrained_cube_registration.json').read_bytes())
    f=reg['rows'][0];h=min(f['hypotheses'],key=lambda x:x['error_px'])
    trial=json.loads((FOLDER/'result.json').read_bytes())
    sample=min(trial['samples'],key=lambda x:abs(x['t']-(f['camera_read_monotonic']-f['enabled_at'])))
    raw=np.array(sample['actual']);center=np.array(h['cube_center_world_m'])
    R=np.array(h['cube_rotation_world']);quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id
    names=('if_tip','mf_tip','rf_tip','th_tip')
    def measure(q):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=np.r_[center,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
        gaps={n:float(mujoco.mj_geomDistance(m,d,cube,m.geom(n).id,1,None))*1000 for n in names}
        return gaps
    baseline=measure(raw);rows=[]
    for ident in (5,6,7,8,9,10,11,12,1,2,3,4):
        moved=raw.copy();moved[ident-1]+=8
        value=measure(moved)
        rows.append(dict(motor_id=ident,gap_delta_per_plus8ticks_mm={n:value[n]-baseline[n] for n in names}))
    out=dict(status='conditional_contact_gradient_diagnostic',source=FOLDER.name,measured_motor_ticks=raw.tolist(),
        cube_center_world_m=center.tolist(),cube_rotation_world=R.tolist(),baseline_tip_gaps_mm=baseline,
        perturbations=rows,registration_verified=False,hardware_execution_allowed=False,hardware_commands_sent=0,
        limitations=['Rim and hand mounting uncertainties are not calibrated.',
                     'Negative gap is model overlap, not an instruction to push or a measured penetration.'])
    (FOLDER/'conditional_grip_directions.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print('BASE',baseline,flush=True)
    for row in rows:print(row['motor_id'],{n:round(v,3) for n,v in row['gap_delta_per_plus8ticks_mm'].items()},flush=True)


if __name__=='__main__':main()
