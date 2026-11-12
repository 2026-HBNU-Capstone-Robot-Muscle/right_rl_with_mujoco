"""Check whether archived stage-4 posture makes hand-to-hand contacts in MuJoCo."""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from mujoco_playground import registry

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def name(model, kind, index):
    return mujoco.mj_id2name(model, kind, int(index))


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    run = json.loads((ROOT / 'leap_sequence_flip_20261011_051518_433595.json').read_text())
    rows = [r for r in run['samples'] if r['phase'] == 'stage_4']
    end = rows[-1]
    report = dict(status='offline_mechanical_stop_collision_probe', motor_commands_sent=0,
                  observed_motor_id=12, phase='stage_4', postures={})
    for label, ticks in [('actual', end['actual']), ('goal', end['goal'])]:
        data = mujoco.MjData(model)
        data.qpos[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
        data.qpos[-7:-4] = np.asarray([0, 0, 2])  # cube far from the hand
        data.qpos[-4:] = np.asarray([1, 0, 0, 0])
        mujoco.mj_forward(model, data)
        hand_contacts = []
        for contact in data.contact:
            a = name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1])
            b = name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2])
            if a.startswith(('if_', 'mf_', 'rf_', 'th_', 'palm')) and b.startswith(
                    ('if_', 'mf_', 'rf_', 'th_', 'palm')):
                hand_contacts.append(dict(bodies=[a, b],
                                          geoms=[name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                                                 name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)],
                                          penetration_mm=max(0., float(-contact.dist * 1000))))
        middle_vs_support = []
        for gm in range(model.ngeom):
            bm = name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gm])
            if not bm.startswith('mf_'):
                continue
            for go in range(model.ngeom):
                bo = name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[go])
                if bo != 'palm' and not bo.startswith('mf_'):
                    continue
                if bo == bm:
                    continue
                from_to = np.zeros(6)
                distance = mujoco.mj_geomDistance(model, data, gm, go, .1, from_to)
                if np.isfinite(distance):
                    middle_vs_support.append(dict(bodies=[bm, bo],
                                                  geoms=[name(model, mujoco.mjtObj.mjOBJ_GEOM, gm),
                                                         name(model, mujoco.mjtObj.mjOBJ_GEOM, go)],
                                                  distance_mm=float(distance * 1000),
                                                  collider_enabled=bool(model.geom_contype[gm] and
                                                                        model.geom_contype[go])))
        report['postures'][label] = dict(all_hand_contacts=hand_contacts,
                                         middle_finger_contacts=[c for c in hand_contacts
                                                                 if any(x.startswith('mf_') for x in c['bodies'])],
                                         nearest_middle_support_geoms=sorted(
                                             middle_vs_support, key=lambda r: r['distance_mm'])[:10],
                                         nearest_middle_palm_collision_geoms=sorted(
                                             [r for r in middle_vs_support
                                              if r['bodies'][1] == 'palm' and r['collider_enabled']],
                                             key=lambda r: r['distance_mm'])[:5],
                                         nearest_palm_by_middle_segment={
                                             segment: min((r for r in middle_vs_support
                                                           if r['bodies'] == [segment, 'palm']
                                                           and r['collider_enabled']),
                                                          key=lambda r: r['distance_mm'], default=None)
                                             for segment in ('mf_bs', 'mf_px', 'mf_md', 'mf_ds')})
    report['caveat'] = ('No contact here does not prove real clearance: CAD collision geometry may omit the '
                        'physical stop, and self-collision pairs may be filtered. A contact here also does '
                        'not prove it is the same physical lip.')
    (ROOT / 'sac_stop_collision_probe_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({**{k: v for k, v in report.items() if k != 'postures'},
                      'contacts_by_posture': {k: dict(all=len(v['all_hand_contacts']),
                                                     middle=v['middle_finger_contacts'],
                                                     nearest_middle_palm_colliders=v['nearest_middle_palm_collision_geoms'],
                                                     by_segment=v['nearest_palm_by_middle_segment'])
                                              for k, v in report['postures'].items()}}, indent=2))


if __name__ == '__main__':
    main()
