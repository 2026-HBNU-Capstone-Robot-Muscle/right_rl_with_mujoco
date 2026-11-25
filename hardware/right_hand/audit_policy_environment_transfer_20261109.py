"""Read-only model/observation contract comparison; no hardware or training."""
from pathlib import Path
import hashlib
import inspect
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/environment_transfer'
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'


def serial(value):
    if isinstance(value, (np.ndarray, np.generic)):
        return value.tolist()
    return str(value)


def describe(model):
    data = mujoco.MjData(model)
    start = json.loads((ROOT / 'sac_cube55_resets.json').read_text())['candidates'][0]
    data.qpos[:] = start['qpos']; data.ctrl[:] = start['ctrl']
    mujoco.mj_forward(model, data)
    palm = model.body('palm').id
    sensors = {}
    for name in ('palm_position', 'cube_position', 'cube_orientation'):
        try:
            sensor = model.sensor(name)
        except KeyError:
            continue
        sid = sensor.id
        adr, dim = int(model.sensor_adr[sid]), int(model.sensor_dim[sid])
        sensors[name] = {
            'objtype': int(model.sensor_objtype[sid]), 'objid': int(model.sensor_objid[sid]),
            'value': data.sensordata[adr:adr+dim].tolist(),
        }
    return {
        'nq': model.nq, 'nv': model.nv, 'nu': model.nu, 'ngeom': model.ngeom,
        'joint_names': [model.joint(i).name for i in range(model.njnt)],
        'actuator_joints': [model.joint(int(i)).name for i in model.actuator_trnid[:, 0]],
        'joint_axis': model.jnt_axis.tolist(),
        'joint_range': model.jnt_range.tolist(),
        'joint_actuator_force_limited': model.jnt_actfrclimited.tolist(),
        'joint_actuator_force_range': model.jnt_actfrcrange.tolist(),
        'actuator_ctrlrange': model.actuator_ctrlrange.tolist(),
        'actuator_forcerange': model.actuator_forcerange.tolist(),
        'actuator_gainprm': model.actuator_gainprm.tolist(),
        'actuator_biasprm': model.actuator_biasprm.tolist(),
        'dof_damping': model.dof_damping.tolist(),
        'dof_armature': model.dof_armature.tolist(),
        'gravity': model.opt.gravity.tolist(),
        'timestep': model.opt.timestep,
        'palm_origin': data.xpos[palm].tolist(),
        'palm_rotation': data.xmat[palm].reshape(3, 3).tolist(),
        'sensors_at_same_archived_state': sensors,
        'body_mass': {model.body(i).name: float(model.body_mass[i]) for i in range(model.nbody)},
        'geoms': {model.geom(i).name: {
            'body': model.body(int(model.geom_bodyid[i])).name,
            'type': int(model.geom_type[i]), 'size': model.geom_size[i].tolist(),
            'local_pos': model.geom_pos[i].tolist(), 'local_quat': model.geom_quat[i].tolist(),
            'friction': model.geom_friction[i].tolist(), 'solref': model.geom_solref[i].tolist(),
            'solimp': model.geom_solimp[i].tolist(),
            'contype': int(model.geom_contype[i]), 'conaffinity': int(model.geom_conaffinity[i]),
        } for i in range(model.ngeom)},
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'; cfg.chain_goals = False; cfg.face_onehot_obs = False
    base = registry.load('LeapCubeFaceUp', config=cfg)
    reset = json.loads((ROOT / 'sac_supported_reset_result.json').read_text())
    source_check = {name: {'expected': digest, 'current': hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}
                    for name, digest in reset['sources_sha256'].items()}
    mismatches = [name for name, row in source_check.items() if row['expected'] != row['current']]
    if any(name != 'wood_glass_support_measurements.json' for name in mismatches):
        raise RuntimeError('Unreconstructed model source changes: ' + repr(mismatches))
    # The old reset manifest records its dimensions. Reconstruct an isolated
    # research model explicitly; do not edit current measurements or bypass the
    # unchanged SupportedStart provenance checks.
    archived = reset['model']
    old, support_manifest = build(base, projection_mm=archived['projection_mm'],
                                  glass_mm=archived['glass_thickness_mm'],
                                  front_reference_mm=archived['fixed_front_palm_x_mm'],
                                  bottom_reference_mm=archived['fixed_bottom_palm_z_mm'])
    old.numeric('max_geom_pairs').data[:] = -1
    old.numeric('max_contact_points').data[:] = 128
    old.actuator_ctrlrange[[0, 4, 8], 0] = reset['root_command_floor_rad']
    base._mj_model = old
    base._post_init()
    new = mujoco.MjModel.from_binary_path('current.mjb', assets={'current.mjb': MODEL.read_bytes()})
    before, after = describe(old), describe(new)
    (OUT / 'training_model.json').write_text(json.dumps(before, indent=2), encoding='utf-8')
    (OUT / 'current_model.json').write_text(json.dumps(after, indent=2), encoding='utf-8')
    source = inspect.getsource(base.__class__)
    source_file = Path(inspect.getfile(base.__class__))
    (OUT / 'training_environment_source.py').write_text(source, encoding='utf-8')
    methods = {}
    for name in ('_get_obs', '_post_init', 'get_palm_position', 'get_cube_position',
                 'get_cube_orientation', '_cube_orientation_error'):
        methods[name] = [{'class': cls.__name__, 'source': inspect.getsource(cls.__dict__[name])}
                         for cls in type(base).__mro__ if name in cls.__dict__]
    (OUT / 'training_methods.json').write_text(json.dumps(methods, indent=2), encoding='utf-8')
    report = {
        'status': 'read_only_environment_contract_comparison',
        'training_model_is_explicit_archive_parameter_reconstruction': True,
        'archive_source_hash_check': source_check,
        'archive_source_hash_mismatches': mismatches,
        'reconstructed_support_manifest': support_manifest,
        'current_model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        'training_source_file': str(source_file),
        'training_source_sha256': hashlib.sha256(source_file.read_bytes()).hexdigest(),
        'training_config': cfg.to_dict(),
        'same_joint_order': before['joint_names'] == after['joint_names'],
        'same_actuator_joint_order': before['actuator_joints'] == after['actuator_joints'],
        'same_joint_axes': np.allclose(old.jnt_axis, new.jnt_axis),
        'same_palm_origin': np.allclose(before['palm_origin'], after['palm_origin']),
        'same_palm_rotation': np.allclose(before['palm_rotation'], after['palm_rotation']),
        'training_gravity': before['gravity'], 'current_gravity': after['gravity'],
        'training_timestep': old.opt.timestep, 'current_timestep': new.opt.timestep,
        'training_control_dt': base.dt,
        'training_force_limit': old.actuator_forcerange.tolist(),
        'current_force_limit': new.actuator_forcerange.tolist(),
        'training_joint_force_limit': old.jnt_actfrcrange.tolist(),
        'current_joint_force_limit': new.jnt_actfrcrange.tolist(),
        'training_gain': old.actuator_gainprm[:, 0].tolist(),
        'current_gain': new.actuator_gainprm[:, 0].tolist(),
        'training_damping': old.dof_damping.tolist(),
        'current_damping': new.dof_damping.tolist(),
        'training_palm_position_sensor': before['sensors_at_same_archived_state'].get('palm_position'),
        'current_palm_position_sensor': after['sensors_at_same_archived_state'].get('palm_position'),
        'training_palm_body_origin': before['palm_origin'],
        'current_palm_body_origin': after['palm_origin'],
        'common_geom_differences': {name: {key: {'training': value, 'current': after['geoms'][name][key]}
                                       for key, value in geom.items() if value != after['geoms'][name][key]}
                                    for name, geom in before['geoms'].items()
                                    if name in after['geoms'] and geom != after['geoms'][name]},
        'training_only_geoms': sorted(set(before['geoms']) - set(after['geoms'])),
        'current_only_geoms': sorted(set(after['geoms']) - set(before['geoms'])),
        'devices_opened': 0, 'new_training_executed': False, 'hardware_execution_allowed': False,
    }
    (OUT / 'review.json').write_text(json.dumps(report, indent=2, default=serial), encoding='utf-8')
    keys = ('same_joint_order', 'same_actuator_joint_order', 'same_joint_axes',
            'same_palm_origin', 'same_palm_rotation', 'training_gravity', 'current_gravity',
            'training_control_dt', 'training_timestep', 'current_timestep',
            'training_joint_force_limit', 'current_joint_force_limit',
            'training_gain', 'current_gain', 'training_damping', 'current_damping',
            'training_palm_position_sensor', 'current_palm_position_sensor',
            'training_palm_body_origin', 'current_palm_body_origin')
    print(json.dumps({key: report[key] for key in keys}), flush=True)
    print(json.dumps({'changed_common_geoms': list(report['common_geom_differences']),
                      'training_only_geoms': report['training_only_geoms'],
                      'current_only_geoms': report['current_only_geoms']}), flush=True)


if __name__ == '__main__':
    main()
