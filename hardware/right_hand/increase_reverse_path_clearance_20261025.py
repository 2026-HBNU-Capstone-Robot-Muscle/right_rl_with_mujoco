"""Increase separation in proposed goals, preserving CAD, bounds and raw data."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,argparse
from pathlib import Path
import numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from collision_projection_20261025 import CollisionProjection
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--observed-pairs-only',action='store_true');args=parser.parse_args()
    p=OUT/'CAD_adapted_forward_reference_path_v2.json';blob=p.read_bytes();old=json.loads(blob)
    source=np.array(old['dense_goal_ticks'])[::-1].copy();reference=ticks_to_sim(source)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'reverse_clearance_CAD.xml')
    palms=[m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.startswith('palm_collision_')]
    pairs=[(name,tip) for name in palms for tip in ('th_tip','rf_tip')]+[('rf_tip','th_tip')]
    if args.observed_pairs_only:pairs=[('rf_tip','th_tip'),('palm_collision_6','th_tip'),('palm_collision_7','th_tip'),('palm_collision_4','rf_tip')]
    projector=CollisionProjection(m,margin_m=.0015,guard_pairs=pairs);path=[];maximum_deviation=0.
    for i,q in enumerate(reference):
        result,review=projector.project(q,q,bound_rad=.15)
        if review['maximum_overlap_mm']>.0001:raise RuntimeError('Projection did not remove contact')
        path.append(projector.quantize_goals(result));maximum_deviation=max(maximum_deviation,review['maximum_reference_deviation_rad'])
        if i%200==0:print('REVERSE_CLEARANCE',i,'delta_deg',round(float(np.degrees(maximum_deviation)),2),flush=True)
    dense=[path[0]]
    for a,b in zip(path[:-1],path[1:]):
        n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        dense.extend(np.rint(a+(b-a)*k/n).astype(int) for k in range(1,n+1))
    qq=ticks_to_sim(np.array(dense));maximum=max(projector.maximum_overlap(q) for q in qq)*1000
    minimum=min(float(np.min(projector.distances(q,projector.guard_pairs))) for q in qq)*1000
    result=dict(source_path_sha256=hashlib.sha256(blob).hexdigest(),source_order_reversed=True,
        dense_goal_ticks=np.array(dense).tolist(),maximum_dense_model_overlap_mm=maximum,
        desired_guard_pair_clearance_mm=1.5,minimum_dense_guard_pair_clearance_mm=minimum,
        desired_guard_clearance_satisfied=minimum>=1.4,
        guard_pairs=[list(p) for p in pairs],maximum_reference_deviation_rad=maximum_deviation,
        full_static_path_clear=maximum<.0001,
        all_quantized_goals_within_native_bounds=bool(np.all(qq>=projector.lo-1e-10) and np.all(qq<=projector.hi+1e-10)),
        maximum_goal_step_ticks=int(np.max(abs(np.diff(np.array(dense),axis=0)))),nominal_seconds=(len(dense)-1)*.05,
        source_start_changed=not np.array_equal(dense[0],source[0]),model=manifest,
        contact_geometry_changed=False,native_ranges_changed=False,archived_motor_records_changed=False,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/('reverse_original_clearance_path_v2.json' if args.observed_pairs_only else 'reverse_original_clearance_path_v1.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('REVERSE_CLEARANCE_COMPLETE','clear',result['full_static_path_clear'],'minimum_guard_mm',minimum,'goals',len(dense),flush=True)

if __name__=='__main__':main()
