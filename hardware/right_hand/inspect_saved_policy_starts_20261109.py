"""Check archived policy starts against the current unchanged collision model."""
from pathlib import Path
import hashlib
import json
import numpy as np
import mujoco
from hand_motor_mapping import sim_to_ticks
from research_rotation_body_hold_20261104 import inspect_for, native

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/training_start_transfer'
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'


def inspect(m, d, hand, check):
    supports, own = check(d)
    cube = m.geom('cube').id
    contacts = []
    for contact in d.contact:
        if cube in (int(contact.geom1), int(contact.geom2)):
            other = int(contact.geom2) if int(contact.geom1) == cube else int(contact.geom1)
            contacts.append({'geom': m.geom(other).name, 'distance_mm': float(contact.dist) * 1000,
                             'hand_geom': int(m.geom_bodyid[other]) in hand})
    return {'self_pairs': own, 'support_pairs': supports, 'cube_contacts': contacts,
            'maximum_cube_penetration_mm': max((max(0., -c['distance_mm']) for c in contacts if c['hand_geom']), default=0.)}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
    hand, _, check = inspect_for(m)
    low, high, _, _ = native(m)
    rows = []
    for name in ('sac_supported_reset_result.json', 'sac_cube55_resets.json'):
        source = json.loads((ROOT / name).read_bytes())
        for index, candidate in enumerate(source['candidates']):
            d = mujoco.MjData(m)
            d.qpos[:] = candidate['qpos']
            d.ctrl[:] = candidate['ctrl']
            mujoco.mj_forward(m, d)
            static = inspect(m, d, hand, check)
            raw = sim_to_ticks(d.ctrl)
            accepted = bool(not static['self_pairs'] and
                            max(static['support_pairs'].values(), default=0.) <= .15 and
                            static['maximum_cube_penetration_mm'] <= 1 and
                            np.all(raw >= low) and np.all(raw <= high))
            row = {'source': name, 'index': index, 'source_old_passed': candidate['passed'],
                   'current_model_static': static, 'current_native_range_valid': bool(np.all(raw >= low) and np.all(raw <= high)),
                   'current_model_initial_admission': accepted}
            if accepted:
                filename = name.removesuffix('.json') + f'_{index}.npz'
                np.savez_compressed(OUT / filename, qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl,
                                    qacc_warmstart=d.qacc_warmstart, act=d.act, time=d.time,
                                    up=-m.opt.gravity / np.linalg.norm(m.opt.gravity))
                row['state_file'] = filename
            rows.append(row)
            print(json.dumps(row), flush=True)
    report = {'status': 'archived_policy_start_transfer_inspected',
              'model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest(), 'cases': rows,
              'initial_admitted': sum(r['current_model_initial_admission'] for r in rows),
              'static_check_not_full_dynamic_admission': True,
              'devices_opened': 0, 'hardware_execution_allowed': False}
    (OUT / 'review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
