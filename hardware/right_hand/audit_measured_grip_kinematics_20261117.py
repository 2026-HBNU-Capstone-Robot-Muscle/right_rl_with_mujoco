"""Offline FK of the measured physical grip; no invented cube pose or motion approval."""
from pathlib import Path
import hashlib, json
import numpy as np
import mujoco
from hand_motor_mapping import ticks_to_sim

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'right_hand_rotation_20261117/user_measured_grip/kinematics'
MODEL = ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
VIEW = ROOT/'measured_grip_view_20261117_175648_673729/result.json'
REFERENCE = ROOT/'right_hand_rotation_20261117/user_measured_grip/reference.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    observation = json.loads(VIEW.read_bytes())
    reference = json.loads(REFERENCE.read_bytes())
    actual = np.array(observation['after']['actual_ticks'], dtype=int)
    goal = np.array(reference['holding_goal_ticks_readonly'], dtype=int)
    if observation['after']['goal_ticks'] != goal.tolist():
        raise RuntimeError('Observation target differs from physical reference')
    m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
    d = mujoco.MjData(m)
    palm = m.body('palm').id
    tips = ('if_tip', 'mf_tip', 'rf_tip', 'th_tip')

    def points(ticks):
        d.qpos[:16] = ticks_to_sim(ticks)
        d.qvel[:] = 0
        d.ctrl[:] = ticks_to_sim(goal)
        mujoco.mj_forward(m, d)
        rotation = d.xmat[palm].reshape(3, 3)
        return {name: (d.geom_xpos[m.geom(name).id]-d.xpos[palm]) @ rotation * 1000 for name in tips}

    initial = points(actual)
    probes = []
    for role, motor_id in [('if_tip', 5), ('if_tip', 7), ('mf_tip', 9), ('mf_tip', 11),
                           ('rf_tip', 13), ('rf_tip', 15), ('th_tip', 1), ('th_tip', 2),
                           ('th_tip', 3), ('th_tip', 4)]:
        test = actual.copy()
        test[motor_id-1] -= 16
        changed = points(test)
        delta = changed[role]-initial[role]
        probes.append(dict(motor_id=motor_id, role=role, change_ticks=-16,
                           change_degrees=-16*360/4096,
                           tip_center_displacement_in_palm_mm=delta.tolist(),
                           tip_center_distance_mm=float(np.linalg.norm(delta))))

    points(actual)
    palm_rotation = d.xmat[palm].reshape(3, 3)
    geometry = []
    for gid in range(m.ngeom):
        name = m.geom(gid).name or ''
        body_name = m.body(int(m.geom_bodyid[gid])).name or ''
        if not (name.startswith('palm_collision_') or body_name.startswith(('if_', 'mf_', 'rf_', 'th_'))):
            continue
        geometry.append(dict(name=name, body=body_name, geometry_type=int(m.geom_type[gid]),
            center_palm_mm=((d.geom_xpos[gid]-d.xpos[palm]) @ palm_rotation*1000).tolist(),
            rotation_palm=(palm_rotation.T @ d.geom_xmat[gid].reshape(3,3)).tolist(),
            size_mm=(m.geom_size[gid]*1000).tolist(), mesh_id=int(m.geom_dataid[gid])))
    hand = {palm}
    for body in range(m.nbody):
        if int(m.body_parentid[body]) in hand:
            hand.add(body)
    self_contacts = []
    for contact in d.contact:
        a, b = int(contact.geom1), int(contact.geom2)
        if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
            self_contacts.append(dict(geometry=[m.geom(a).name, m.geom(b).name],
                                      signed_distance_mm=float(contact.dist*1000)))
    report = dict(kind='measured_grip_static_forward_kinematics_only',
        model_sha256=sha(MODEL), source_path=VIEW.relative_to(ROOT).as_posix(),
        source_sha256=sha(VIEW), reference_sha256=sha(REFERENCE), script_sha256=sha(Path(__file__)),
        actual_ticks=actual.tolist(), holding_goal_ticks=goal.tolist(),
        tip_centers_in_palm_frame_mm={k:v.tolist() for k,v in initial.items()},
        geometry_in_palm_frame=geometry,
        gravity_in_palm_frame=(m.opt.gravity @ palm_rotation).tolist(),
        single_joint_inward_fk_probes=probes, static_model_hand_contacts=self_contacts,
        cube_pose_assumed=False, cube_contact_locations_measured=False,
        dynamic_simulation=False, physical_motion_executed=False,
        executable_candidate=False, original_model_modified=False,
        interpretation='Tip center motion only; not contact-point travel or proof of cube support. '
                       'The measured grip remains the reference; no old model grip substituted.')
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(tip_centers=report['tip_centers_in_palm_frame_mm'],
                          probes=probes, static_hand_contacts=self_contacts), ensure_ascii=False))


if __name__ == '__main__':
    main()
