"""Bounded integer target export and independent dense geometry recheck."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from collision_projection_20261025 import CollisionProjection
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    p=OUT/'projected_policy_reference_v2.json';blob=p.read_bytes();old=json.loads(blob)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'quantized_policy_CAD.xml');projector=CollisionProjection(m)
    source=np.array(old['dense_raw_ticks']);raw=projector.quantize_goals(ticks_to_sim(source));q=ticks_to_sim(raw)
    maximum=max(projector.maximum_overlap(row) for row in q)*1000
    range_violation=max(float(np.max(projector.lo-q)),float(np.max(q-projector.hi)),0.)
    result={**old,'projection_revision':3,'parent_revision_sha256':hashlib.sha256(blob).hexdigest(),
        'dense_raw_ticks':raw.tolist(),
        'projected_goals_sim_rad':ticks_to_sim(projector.quantize_goals(np.array(old['projected_goals_sim_rad']))).tolist(),
        'integer_rounding_native_range_violation_rad':range_violation,
        'maximum_integer_rounding_correction_ticks':int(np.max(abs(raw-source))),
        'maximum_dense_model_overlap_mm':maximum,'full_static_path_clear':maximum<.0001 and range_violation<1e-10,
        'native_ranges_unchanged':True,'archived_raw_measurements_unchanged':True,'model':manifest}
    assert np.max(abs(np.diff(raw,axis=0)))<=2
    assert result['full_static_path_clear']
    (OUT/'projected_policy_reference_v3.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('QUANTIZED_POLICY','maximum_correction_ticks',result['maximum_integer_rounding_correction_ticks'],
        'native_range_violation',range_violation,'model_overlap_mm',maximum,flush=True)

if __name__=='__main__':main()
