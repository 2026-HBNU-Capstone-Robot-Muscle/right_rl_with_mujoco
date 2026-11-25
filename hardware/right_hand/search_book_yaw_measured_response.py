"""Offline candidate search using the latest measured held-hand response.

The current trial provides baseline encoders. Cube placement and friction are
still explicit geometry hypotheses; neither a joint zero nor camera registration
is fitted to a desired rotation result. No motor/camera device is opened.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from scipy.spatial.transform import Rotation
from book_support_model_20261024 import build_book
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim
import search_book_contact_yaw as search

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_small_yaw_trial_20261024_215736_594865'
OUT=ROOT/'right_hand_rotation_20261024/book_yaw_measured_response';OUT.mkdir(exist_ok=True)


def main():
    blob=(FOLDER/'result.json').read_bytes();trial=json.loads(blob)
    samples=[s for s in trial['samples'] if s['phase']=='stage_4_book_grasp_hold'][-20:]
    actual=np.median([s['actual'] for s in samples],axis=0);base=np.array(samples[-1]['goal'])
    source=dict(grasp_goal_ticks=base.tolist(),actual_baseline_ticks=actual.tolist(),
        source=FOLDER.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(blob).hexdigest(),
        measured_phase='stage_4_book_grasp_hold',source_samples=20,hardware_execution_allowed=False)
    normalized=OUT/'measured_baseline_source.json';normalized.write_text(json.dumps(source,indent=2),encoding='utf-8')
    old=json.loads((ROOT/'right_hand_rotation_20261024/book_grasp_yaw_diagnostic/relative_candidate_replay.json').read_bytes())
    c0=np.array(old['selected_placement_hypothesis']['center_m']);R=np.array(old['cube_rotation_world_hypothesis'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_book(env,OUT/'geometry_search.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]
    palms=[i for i in range(m.ngeom) if m.geom(i).name.startswith('palm_collision')]
    placements=[]
    for dx in range(-6,7,2):
        for dy in range(-6,7,2):
            for dz in range(-4,9,2):
                center=c0+R@np.array([dx,dy,dz])*.001
                d.qpos[:16]=ticks_to_sim(actual);d.qpos[-7:]=np.r_[center,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
                overlap=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                if overlap>.15:continue
                gaps=[float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for g in tips]
                palm=min(float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for g in palms)
                placements.append(dict(center_m=center.tolist(),offset_mm=[dx,dy,dz],initial_overlap_mm=overlap,
                    tip_gaps_mm=gaps,palm_gap_mm=palm,geometric_score=float(max(gaps)+.1*np.linalg.norm([dx,dy,dz]))))
    if not placements:raise ValueError('No low-overlap initial placement hypothesis')
    selected=min(placements,key=lambda p:p['geometric_score'])
    prior=dict(measured_grasp_ticks=actual.tolist(),selected_placement_hypothesis=selected,
        cube_rotation_world_hypothesis=R.tolist(),placement_hypotheses=placements,
        selection_by_initial_geometry_only=True,physical_marker_registration_verified=False,
        hardware_execution_allowed=False)
    (OUT/'relative_candidate_replay.json').write_text(json.dumps(prior,indent=2),encoding='utf-8')
    print('CURRENT_MEASURED_BASELINE',actual.tolist(),flush=True)
    print('GEOMETRY_HYPOTHESIS',json.dumps(selected),flush=True)
    search.SOURCE=normalized;search.OUT=OUT
    search.main()


if __name__=='__main__':main()
