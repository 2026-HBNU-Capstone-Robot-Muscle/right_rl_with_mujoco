"""Reject contradictory image-to-model registration; no dynamic or real action."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,sys
from pathlib import Path
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    source=json.loads((OUT/('conditional_raw_frame_start_pose.json' if '--raw-frame' in sys.argv else 'conditional_actual_start_pose.json')).read_bytes());actual=json.loads((ROOT/'loaded_repeat_20261026_170932_163649/result.json').read_bytes())['placement_position_ticks'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'conditional_saved_start_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    results=[]
    for pose in source['candidates']:
        if not pose['consistent_with_known_start']:continue
        d.qpos[:16]=ticks_to_sim(actual);d.qpos[-7:-4]=pose['cube_center_world_m'];xyzw=Rotation.from_matrix(pose['R_cube_world']).as_quat();d.qpos[-4:]=xyzw[[3,0,1,2]];d.qvel[:]=0;mujoco.mj_forward(m,d);contacts=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) not in hand:continue
            contacts.append(dict(geom=m.geom(other).name,depth_mm=max(0.,-float(c.dist)*1000)))
        results.append(dict(center_palm_mm=pose['center_in_palm_mm'],max_cube_hand_overlap_mm=max((c['depth_mm'] for c in contacts),default=0.),contacts=contacts,metric_registration_accepted=False))
    report=dict(status='saved_image_metric_registration_consistency_check',source_anchor_error_px=source['anchor_reprojection_px'],candidates=results,model=manifest,
        image_consistency_not_absolute_hand_calibration=True,hand_marker_location_and_single_face_depth_not_independently_verified=True,
        cannot_distinguish_pose_error_from_collision_geometry_error=True,simulation_started_from_this_pose=False,hardware_execution_allowed=False,real_devices_opened=False)
    name='conditional_raw_frame_model_consistency.json' if '--raw-frame' in sys.argv else 'conditional_start_model_consistency.json'
    (OUT/name).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='model'},indent=2),flush=True)
if __name__=='__main__':main()
