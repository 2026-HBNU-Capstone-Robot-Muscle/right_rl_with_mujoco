"""Check whether provisional cube poses reproduce the saved fixed-hand hold.

Hand coordinates are clamped to the measured held pose to isolate object/contact
geometry. This is state-estimation analysis, never an actuator/path certificate.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import itertools
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
    blob=(FOLDER/'direct_reference_cube_pose.json').read_bytes();ref=json.loads(blob)
    h=min(ref['hypotheses'],key=lambda x:x['marker_error_px'])
    center=np.array(h['cube_center_world_m']);R=np.array(h['cube_rotation_world'])
    orientation=Rotation.from_matrix(R);quat=orientation.as_quat()[[3,0,1,2]]
    q=ticks_to_sim(ref['measured_motor_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    m.dof_armature[:16]=1e6
    cube=m.geom('cube').id;up=-np.array(m.opt.gravity);up/=np.linalg.norm(up)
    supports={m.geom(n).id for n in ('support_glass','support_wood')}
    rows=[]
    for xyz in itertools.product((-3.,0.,3.),repeat=3):
        d=mujoco.MjData(m);d.qpos[:16]=q
        start=np.r_[center+R@np.array(xyz)/1000,quat];d.qpos[-7:]=start;d.ctrl[:]=q
        mujoco.mj_forward(m,d);external=False;max_pen=0.;tail=[]
        for k in range(round(4/m.opt.timestep)):
            d.qpos[:16]=q;d.qvel[:16]=0;mujoco.mj_step(m,d)
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube in (a,b):
                    max_pen=max(max_pen,max(0.,-float(c.dist)*1000))
                    external |= (b if a==cube else a) in supports
            if k>=round(3/m.opt.timestep) and k%25==0:tail.append(d.qpos[-7:].copy())
        final=d.qpos[-7:].copy();rot=Rotation.from_quat(final[3:][[1,2,3,0]])
        shift=float(np.linalg.norm(final[:3]-start[:3])*1000)
        angle=float((rot*orientation.inv()).magnitude()*180/np.pi)
        jitter=float(np.max(np.ptp(np.array(tail)[:,:3],axis=0))*1000)
        finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
        stable=finite and not external and shift<=5 and angle<=5 and jitter<=.2
        row=dict(offset_cube_axes_mm=list(xyz),shift_mm=shift,angle_change_deg=angle,
            final_second_center_span_mm=jitter,external_support_contact=bool(external),
            max_cube_overlap_mm=max_pen,finite=finite,compatible_with_near_fixed_hold=stable,
            initial_cube_qpos=start.tolist(),final_cube_qpos=final.tolist())
        rows.append(row)
        print('HOLD',xyz,'shift/angle/jitter',round(shift,2),round(angle,2),round(jitter,3),'compatible',stable,flush=True)
    result=dict(status='provisional_observed_hold_consistency_probe',source=FOLDER.name,
        reference_sha256=hashlib.sha256(blob).hexdigest(),case_count=len(rows),
        compatible_cases=sum(row['compatible_with_near_fixed_hold'] for row in rows),
        tested_hold_seconds=4,source_hold_capture_seconds=75.906,
        hand_clamped_to_measured_pose=True,model_manifest=manifest,rows=rows,
        full_model_or_pose_calibration_verified=False,hardware_execution_allowed=False,motor_commands_sent=0,
        limitations=['4s local contact consistency, not reproduction of the entire 76s record.',
            'The hand is fixed at measured angles. This does not certify motor tracking, current, gains or a new motion.',
            'Initial cube poses come from manual image borders/provisional mounting. A compatible case does not establish the actual pose.',
            'No measured pose or runtime transform is overwritten by settled poses.'])
    (FOLDER/'observed_hold_consistency.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('COMPATIBLE',result['compatible_cases'],'/',len(rows),flush=True)

if __name__=='__main__':main()
