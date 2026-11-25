"""Inspect the stopped model configuration and independently bound self gaps."""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from upper_layer_cylinder_clearance_20261105 import precompute
from research_rotation_body_hold_20261104 import inspect_for

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit'
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
hand, _, check = inspect_for(m)
cache = {g: (vertices, edges) for g, vertices, edges in precompute(m, hand)}
cases = []
for directory in ('reused_reference_controller', 'reused_reference_projected_controller'):
    for path in sorted((OUT / directory).glob('*.npz')):
        with np.load(path, allow_pickle=False) as z:
            d = mujoco.MjData(m)
            d.qpos[:] = z['qpos'][-1]
            d.ctrl[:] = np.asarray(z['goals'][-1] - 2048) * 2 * np.pi / 4096
            mujoco.mj_forward(m, d)
        supports, own = check(d)
        contacts = []
        for c in d.contact:
            a, b = int(c.geom1), int(c.geom2)
            if a not in cache or b not in cache or c.dist >= 0:
                continue
            va, ea = cache[a]
            vb, eb = cache[b]
            A = va @ d.geom_xmat[a].reshape(3, 3).T + d.geom_xpos[a]
            B = vb @ d.geom_xmat[b].reshape(3, 3).T + d.geom_xpos[b]
            axes = np.vstack([ConvexHull(A).equations[:, :3], ConvexHull(B).equations[:, :3],
                              np.eye(3), (A.mean(0) - B.mean(0))[None]])
            axes /= np.maximum(np.linalg.norm(axes, axis=1)[:, None], 1e-20)
            aa = A @ axes.T
            bb = B @ axes.T
            gap = float(np.max(np.maximum(aa.min(0) - bb.max(0), bb.min(0) - aa.max(0))) * 1000)
            contacts.append({'pair': [m.geom(a).name, m.geom(b).name], 'native_contact_mm': float(c.dist)*1000,
                             'independent_separating_plane_gap_lower_bound_mm': gap,
                             'separation_proved': gap > 1e-6})
        row = {'path': str(path.relative_to(ROOT)), 'guard_self_pairs': own,
               'support_pairs': supports, 'negative_self_contacts': contacts}
        cases.append(row)
        print(json.dumps(row), flush=True)
(OUT / 'contact_blocker_audit.json').write_text(json.dumps({'cases': cases, 'devices_opened': 0}, indent=2) + '\n', encoding='utf-8')
