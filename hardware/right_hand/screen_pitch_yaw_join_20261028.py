"""Offline hand-route search between two demonstrated primitives.

No device APIs. A geometric pass is not a loaded-transfer certificate.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import hashlib
import itertools
import json
import time
from pathlib import Path

import mujoco
import numpy as np
from mujoco_playground import registry
from calibrated_quarter_plan_20261028 import LOADED_PATH
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261028/two_axis_connection/hand_join'
PITCH = ROOT / 'right_hand_rotation_20261028/two_axis_connection/saved_pitch/loaded_pitch_path.npz'
SCENE = ROOT / 'right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/scene_-10.xml'
GROUPS = {'thumb': (0, 1, 2, 3), 'index': (4, 5, 6, 7),
          'middle': (8, 9, 10, 11), 'ring': (12, 13, 14, 15)}

def interpolate(a, b):
    n = max(1, int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1, n+1)]).astype(int)

def candidate(start, end, order):
    if order == ('together',):
        return interpolate(start, end)
    rows = []
    previous = start.copy()
    for name in order:
        target = previous.copy()
        ids = list(GROUPS[name])
        target[ids] = end[ids]
        rows.extend(interpolate(previous, target))
        previous = target
    return np.asarray(rows)

def contact_reader(m):
    hand = {m.body('palm').id}
    for body in range(m.nbody):
        if int(m.body_parentid[body]) in hand:
            hand.add(body)
    h = np.isin(m.geom_bodyid, list(hand))
    support = np.array([(m.geom(i).name or '').startswith(('support_', 'leap_mount_collision'))
                        or m.geom(i).name == 'floor' for i in range(m.ngeom)])
    def inspect(d):
        self_mm = support_mm = 0.
        worst_self = worst_support = None
        for index in np.flatnonzero(d.contact.dist < 0):
            c = d.contact[index]
            a, b = int(c.geom1), int(c.geom2)
            depth = -float(c.dist)*1000
            if h[a] and h[b] and depth > self_mm:
                self_mm = depth
                worst_self = '|'.join(sorted((m.geom(a).name, m.geom(b).name)))
            if ((h[a] and support[b]) or (h[b] and support[a])) and depth > support_mm:
                support_mm = depth
                worst_support = '|'.join(sorted((m.geom(a).name, m.geom(b).name)))
        return self_mm, support_mm, worst_self, worst_support
    return inspect

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    start = np.load(PITCH)['goals'][-1]
    end = np.load(LOADED_PATH)['start']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    m, manifest = build(env, OUT/'scene.xml', book_projection_mm=-10.)
    correct_model(m, start)
    d = mujoco.MjData(m)
    d.qpos[-7:] = [0, 0, 2, 1, 0, 0, 0]
    lo, hi = limits()
    inspect = contact_reader(m)
    began = time.perf_counter()
    results = []
    for order in [('together',), *itertools.permutations(GROUPS)]:
        goals = candidate(start, end, order)
        values = np.vstack([start, goals])
        assert max(abs(np.diff(values, axis=0)).ravel()) <= 2
        assert np.all(values >= lo) and np.all(values <= hi)
        self_mm = support_mm = 0.
        stop = None
        first = None
        for i, q in enumerate(values):
            d.qpos[:16] = ticks_to_sim(q)
            mujoco.mj_forward(m, d)
            sp, su, wp, wu = inspect(d)
            self_mm, support_mm = max(self_mm, sp), max(support_mm, su)
            if sp > 1e-6 or su > .15:
                stop = 'hand_self_contact' if sp > 1e-6 else 'hand_support_contact'
                first = dict(index=i, worst_pair=wp if sp > 1e-6 else wu,
                             depth_mm=sp if sp > 1e-6 else su)
                break
        row = dict(order=list(order), goals=len(goals), nominal_seconds=len(goals)*.075,
                   passed=stop is None, stop=stop, first_failure=first,
                   max_self_mm=self_mm, max_support_mm=support_mm)
        if row['passed']:
            name = '_'.join(order)+'.npz'
            np.savez_compressed(OUT/name, start=start, goals=goals, sample_period_s=.075)
            row['asset'] = name
        results.append(row)
        print(json.dumps(row), flush=True)
    report = dict(status='pitch_end_to_yaw_open_static_route_search', cases=results,
                  geometric_passes=sum(r['passed'] for r in results),
                  source_pitch_sha256=hashlib.sha256(PITCH.read_bytes()).hexdigest(),
                  source_yaw_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),
                  model=manifest,
                  cube_disabled_for_hand_clearance=True, loaded_transfer_verified=False,
                  native_limits_preserved=True, max_goal_step_ticks=2,
                  hardware_execution_allowed=False, devices_opened=False,
                  elapsed_seconds=time.perf_counter()-began)
    (OUT/'review.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('SUMMARY', report['geometric_passes'], 'of', len(results), flush=True)

if __name__ == '__main__':
    main()
