"""Offline measured-pose contact audit; does not access hardware or change goals."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from box_clearance import clearance

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'backoff_video_review_20261016'
trial = json.loads((ROOT / 'leap_id12_backoff_empty_20261016_144922_807163.json').read_text())
current = json.loads((OUT / 'readonly_193938_source.json').read_text())
order = np.array(list(range(4, 16)) + list(range(4)))
cfg = registry.get_default_config('LeapCubeFaceUp')
cfg.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=cfg)
m = env.mj_model
original_disableflags = int(m.opt.disableflags)
# Keep native CCD enabled explicitly. It was already enabled in the source
# model; this alone does NOT fix the observed zero-distance artifacts.
m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
hand = {m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:
        hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:
        m.geom_contype[g] = m.geom_conaffinity[g] = 0

def label(g):
    return {'geom': m.geom(g).name, 'body': m.body(int(m.geom_bodyid[g])).name}

rows = []
# One representative per phase suffices to diagnose distance-query artifacts.
representatives = [next(s for s in reversed(trial['samples']) if s['phase'] == phase)
                   for phase in dict.fromkeys(s['phase'] for s in trial['samples'])]
distance_disagreements = []
for sample in representatives + [{'phase': 'current_OFF', 't': None,
        'actual': [r['position_ticks_signed'] for r in sorted(current['motors'], key=lambda r: r['id'])]}]:
    d = mujoco.MjData(m)
    d.qpos[:16] = np.asarray(sample['actual'])[order] * 2 * np.pi / 4096 - np.pi
    d.qpos[-7:-4] = [0, 0, 2]
    d.qpos[-4:] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    contacts = []
    for c in d.contact:
        a, b = label(c.geom1), label(c.geom2)
        if (a['body'].startswith('mf_') or b['body'].startswith('mf_')):
            contacts.append({'a': a, 'b': b, 'distance_mm': float(c.dist * 1000)})
    # Compare the entire last link, not just fingertip spheres. Adjacent link
    # overlap and collision exclusions must not be mistaken for external contact.
    last_body = int(m.geom_bodyid[m.geom('mf_tip').id])
    last = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == last_body
            and (m.geom_contype[g] or m.geom_conaffinity[g])]
    nearest = {}
    for other in range(m.ngeom):
        body = int(m.geom_bodyid[other])
        name = m.body(body).name
        if body not in hand or body == last_body or not (m.geom_contype[other] or m.geom_conaffinity[other]):
            continue
        pair_distances = []
        for g in last:
            engine = float(mujoco.mj_geomDistance(m, d, g, other, 1.0, np.zeros(6)))
            if int(m.geom_type[g]) == 6 and int(m.geom_type[other]) == 6:
                dist, overlap = clearance(d.geom_xpos[g], d.geom_xmat[g].reshape(3,3), m.geom_size[g],
                                          d.geom_xpos[other], d.geom_xmat[other].reshape(3,3), m.geom_size[other])
                if abs(max(0.,engine)-dist) > 1e-5:
                    distance_disagreements.append(dict(phase=sample['phase'], a=label(g), b=label(other),
                        engine_mm=engine*1000, independent_mm=dist*1000, sat_overlap=overlap,
                        box_a=[d.geom_xpos[g].tolist(),d.geom_xmat[g].reshape(3,3).tolist(),m.geom_size[g].tolist()],
                        box_b=[d.geom_xpos[other].tolist(),d.geom_xmat[other].reshape(3,3).tolist(),m.geom_size[other].tolist()]))
            else:
                raise ValueError('Non-box collision geometry requires an independently validated distance implementation')
            pair_distances.append(dist)
        dist = min(pair_distances)
        nearest[name] = min(nearest.get(name, float('inf')), dist * 1000)
    rows.append({'phase': sample['phase'], 't': sample['t'], 'middle_contacts': contacts,
                 'last_link_nearest_bodies_mm': dict(sorted(nearest.items(), key=lambda x: x[1])[:8])})

summary = []
for phase in dict.fromkeys(r['phase'] for r in rows):
    group = [r for r in rows if r['phase'] == phase]
    pairs = {}
    for r in group:
        for c in r['middle_contacts']:
            key = c['a']['body'] + ' / ' + c['b']['body']
            pairs[key] = min(pairs.get(key, float('inf')), c['distance_mm'])
    summary.append({'phase': phase, 'middle_contact_pairs_min_mm': pairs,
                    'last_sample_nearest': group[-1]['last_link_nearest_bodies_mm']})
metadata = {'disableflags': int(m.opt.disableflags), 'enableflags': int(m.opt.enableflags),
            'disabled_flag_names': [name for name in dir(mujoco.mjtDisableBit) if name.startswith('mjDSBL_') and int(getattr(mujoco.mjtDisableBit,name)) & int(m.opt.disableflags)],
            'nativeccd_disabled_bit': int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD),
            'original_disableflags': original_disableflags,
            'mujoco_version': mujoco.__version__,
            'excluded_body_pairs': [], 'middle_geometries': [], 'explicit_pairs': int(m.npair)}
for signature in m.exclude_signature:
    a, b = int(signature) >> 16, int(signature) & 65535
    metadata['excluded_body_pairs'].append([m.body(a).name, m.body(b).name])
for g in range(m.ngeom):
    if m.body(int(m.geom_bodyid[g])).name.startswith('mf_'):
        metadata['middle_geometries'].append(dict(**label(g), type=int(m.geom_type[g]),
            contype=int(m.geom_contype[g]), conaffinity=int(m.geom_conaffinity[g])))
report = {'summary': summary, 'samples': rows, 'distance_disagreements': distance_disagreements,
          'model_collision_metadata': metadata, 'hardware_commands_sent': 0,
          'meaning': 'Static geometry only. Contact mesh, exclusions, and physical correspondence remain provisional.'}
(OUT / 'middle_contact_audit.json').write_text(json.dumps(report, indent=2))
print(json.dumps(summary, indent=2))
print(json.dumps({'mujoco_version': metadata['mujoco_version'],
                  'disabled_flag_names': metadata['disabled_flag_names'],
                  'distance_disagreement_count': len(distance_disagreements),
                  'report': str(OUT / 'middle_contact_audit.json')}, indent=2))
