"""Check distance-query evidence against actual contacts; no devices."""
from pathlib import Path
import json
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import restore

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
FOLDER = ROOT / 'right_hand_rotation_20261109/ring_third_support/after_transfer'
m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
z = np.load(ROOT / 'right_hand_rotation_20261109/index_thumb_transfer/cp1042_together/end.npz', allow_pickle=False)
d = restore(m, z)
cube = m.geom('cube').id
report = {'mujoco_version': mujoco.__version__, 'enableflags': int(m.opt.enableflags), 'disableflags': int(m.opt.disableflags), 'states': []}
proposals = json.loads((FOLDER / 'review.json').read_bytes())
for label, q in [('baseline', None)] + [(str(item['fit_index']), item['ring_qpos']) for item in proposals['fits'][:3]]:
    d = restore(m, z)
    if q is not None:
        d.qpos[8:12] = q
        mujoco.mj_forward(m, d)
    geoms = []
    for name in ('cube', 'rf_tip', 'rf_ds_collision_1', 'rf_md_collision_1', 'rf_px_collision'):
        g = m.geom(name).id
        points = np.zeros(6)
        distance = None if g == cube else float(mujoco.mj_geomDistance(m, d, cube, g, .25, points))
        cs = [{'dist': float(c.dist), 'pos': c.pos.tolist()} for c in d.contact if {int(c.geom1), int(c.geom2)} == {cube, g}]
        geoms.append({'name': name, 'geom_type': int(m.geom_type[g]), 'contype': int(m.geom_contype[g]),
                      'conaffinity': int(m.geom_conaffinity[g]), 'size': m.geom_size[g].tolist(),
                      'position': d.geom_xpos[g].tolist(), 'distance_m': distance,
                      'points': points.tolist(), 'actual_contacts': cs})
    report['states'].append({'label': label, 'geoms': geoms})
(FOLDER / 'distance_query_audit.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
print(json.dumps(report))
