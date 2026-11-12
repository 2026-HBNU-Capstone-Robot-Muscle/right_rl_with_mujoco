"""Check every saved-photo pose branch against historical encoder geometry."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_review'

def main():
    registration=json.loads((OUT/'whole_face_rim_registration.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'rim_consistency_CAD.xml');set_cube_physics(m,.055,.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;output=[]
    for capture,filename in zip(registration['captures'],['leap_sequence_flip_20261011_051350_713486.json','leap_sequence_flip_20261011_051518_433595.json']):
        trial=json.loads((ROOT/filename).read_bytes());row=[s for s in trial['samples'] if s['phase']=='place_cube'][-1]
        q=ticks_to_sim(row['actual']);results=[]
        for pose in capture['all_planar_pose_combinations']:
            d.qpos[:16]=q;d.qpos[-7:]=np.r_[pose['center_world_m'],Rotation.from_matrix(pose['rotation_world']).as_quat()[[3,0,1,2]]]
            mujoco.mj_forward(m,d);contacts=[]
            for c in d.contact:
                if cube in (int(c.geom1),int(c.geom2)) and c.dist<0:
                    contacts.append(dict(pair=[m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name],model_overlap_mm=-float(c.dist)*1000))
            maximum=max([c['model_overlap_mm'] for c in contacts]+[0.])
            results.append({**pose,'maximum_cube_model_overlap_mm':maximum,'cube_overlaps':contacts,
                'geometric_case_evaluable':maximum<=.5})
        good=[r for r in results if r['geometric_case_evaluable'] and r['initial_green_up_error_deg']<20]
        print('RIM_CONSISTENCY',capture['folder'],'valid_green_up',len(good),'minimum_overlap',min(r['maximum_cube_model_overlap_mm'] for r in results),
            'best_image_case_overlap',results[0]['maximum_cube_model_overlap_mm'],flush=True)
        output.append(dict(folder=capture['folder'],historical_encoder_source=filename,candidates=results,
            evaluable_green_up_case_count=len(good)))
    (OUT/'whole_face_rim_geometry_consistency.json').write_text(json.dumps(dict(captures=output,
        historical_support_geometry_not_recreated=True,hand_cube_geometric_consistency_only=True,model=manifest,
        raw_inputs_unchanged=True,hardware_commands_sent=0,geometry_calibration_verified=False),indent=2),encoding='utf-8')

if __name__=='__main__':main()
