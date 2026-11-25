"""Replay the known seven-pose whole-cube flip in the offline contact model.

Uses the saved real stage-0 pose, measured-gravity candidate, 54.5 mm / 100 g
cube, and existing controller poses.  It never opens hardware devices.
"""

import json
import sys
import argparse
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def face_up(quat, gravity):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.asarray(quat))
    R = matrix.reshape(3, 3)
    up = -np.asarray(gravity) / np.linalg.norm(gravity)
    normals = np.asarray([[0, 0, 1], [0, 0, -1], [1, 0, 0],
                          [-1, 0, 0], [0, 1, 0], [0, -1, 0]])
    return int(np.argmax((R @ normals.T).T @ up))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--friction-scale', type=float, default=1.0)
    parser.add_argument('--first-only', action='store_true')
    parser.add_argument('--offset-mm', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--visual-finger-collision', action='store_true')
    parser.add_argument('--side-sign-mask', type=int, choices=range(8), default=0)
    parser.add_argument('--rotate-world-axis', choices=('x', 'y', 'z'))
    parser.add_argument('--rotate-deg', type=float, default=0.0)
    args = parser.parse_args()
    if args.friction_scale <= 0:
        parser.error('friction scale must be positive')
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    changed_visual = 0
    disabled_primitive = 0
    if args.visual_finger_collision:
        for gid in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                     int(model.geom_bodyid[gid]))
            if not name.startswith(('th_', 'if_', 'mf_', 'rf_')):
                continue
            if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH and model.geom_contype[gid] == 0:
                model.geom_contype[gid] = 1
                model.geom_conaffinity[gid] = 2
                changed_visual += 1
            elif model.geom_contype[gid] != 0:
                model.geom_contype[gid] = 0
                model.geom_conaffinity[gid] = 0
                disabled_primitive += 1
    model.geom_friction[:, 0] *= args.friction_scale
    tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                       'tilt_result.json').read_text())
    model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
    reg = json.loads((ROOT / 'sac_cube_rim_register_1010_stage0_result.json').read_text())
    search = json.loads((ROOT / 'sac_measured_gravity_local_search_result.json').read_text())
    saved = json.loads((ROOT / 'leap_support_capture_result.json').read_text())
    poses = json.loads((ROOT / 'sequence_flip_poses.json').read_text(encoding='utf-8'))['poses']
    actual = np.asarray([s for s in saved['samples'] if s['phase'] == 'place_cube'][-1]['actual'], float)
    side_signs = np.ones(16)
    for bit, joint in enumerate((1, 5, 9)):
        if args.side_sign_mask & (1 << bit):
            side_signs[joint] = -1
    start_joints = hw_to_sim(actual * 2 * np.pi / 4096) * side_signs
    reset = env.reset(jax.random.PRNGKey(0))
    gravity = model.opt.gravity.copy()
    cube_body = model.body('cube').id
    candidate_results = []
    candidates = ([dict(offset_mm=args.offset_mm)] if args.offset_mm is not None else
                  search['plausible'][:1] if args.first_only else search['plausible'])
    for candidate in candidates:
        offset = np.asarray(candidate['offset_mm']) / 1000
        cube_start = np.asarray(reg['cube_center_world_m']) + offset
        data = mujoco.MjData(model)
        data.qpos[:] = np.asarray(reset.data.qpos)
        data.qpos[:16] = start_joints
        data.qpos[-7:-4] = cube_start
        cube_quat = np.asarray(reg['cube_quat_world_candidate'], float)
        if args.rotate_world_axis:
            axis = np.eye(3)['xyz'.index(args.rotate_world_axis)]
            rotation = np.empty(4)
            mujoco.mju_axisAngle2Quat(rotation, axis, np.deg2rad(args.rotate_deg))
            rotated = np.empty(4)
            mujoco.mju_mulQuat(rotated, rotation, cube_quat)
            cube_quat = rotated
        data.qpos[-4:] = cube_quat
        data.ctrl[:] = start_joints
        mujoco.mj_forward(model, data)

        def status(stage, step):
            touch = []
            for c in data.contact:
                a, b = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                if a == cube_body and b != cube_body:
                    touch.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b))
                elif b == cube_body and a != cube_body:
                    touch.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, a))
            return dict(stage=stage, step=step, time_s=float(data.time),
                        cube_center_m=data.qpos[-7:-4].tolist(),
                        cube_quat_wxyz=data.qpos[-4:].tolist(),
                        cube_displacement_mm=float(1000 * np.linalg.norm(data.qpos[-7:-4] - cube_start)),
                        face_up=face_up(data.qpos[-4:], gravity), cube_contact_count=len(touch),
                        touching_bodies=sorted(set(touch)),
                        max_joint_tracking_error_rad=float(np.max(abs(data.qpos[:16] - data.ctrl[:16]))))

        rows = [status(0, 0)]
        contact_steps = {}
        previous = start_joints.copy()
        for stage, pose in enumerate(poses[1:], 1):
            target = hw_to_sim(np.asarray(pose['raw_ticks'], float) * 2 * np.pi / 4096) * side_signs
            finger_counts = []
            for command_step in range(1, 71):
                fraction = command_step / 70
                data.ctrl[:] = previous + fraction * (target - previous)
                for _ in range(3):
                    mujoco.mj_step(model, data)
                fingers = set()
                for c in data.contact:
                    a, b = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                    other = b if a == cube_body else a if b == cube_body else None
                    if other is not None:
                        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other)
                        prefix = name.split('_')[0]
                        if prefix in ('th', 'if', 'mf', 'rf'):
                            fingers.add(prefix)
                finger_counts.append(len(fingers))
                if command_step in (20, 40, 60, 70):
                    rows.append(status(stage, command_step))
            contact_steps[f'stage_{stage}'] = dict(
                steps=len(finger_counts),
                any_finger_steps=sum(n >= 1 for n in finger_counts),
                two_finger_steps=sum(n >= 2 for n in finger_counts),
                max_distinct_fingers=max(finger_counts))
            previous = target
        candidate_results.append(dict(offset_mm=candidate['offset_mm'],
                                      start_face=rows[0]['face_up'],
                                      final_face=rows[-1]['face_up'],
                                      face_changed=rows[0]['face_up'] != rows[-1]['face_up'],
                                      max_stage_end_tracking_error_rad=max(
                                          r['max_joint_tracking_error_rad'] for r in rows if r['step'] == 70),
                                      final_displacement_mm=rows[-1]['cube_displacement_mm'],
                                      contact_steps=contact_steps,
                                      stage_endpoints=[r for r in rows if r['step'] == 70 or r['stage'] == 0]))
    report = dict(status='offline_recorded_sequence_replay', motor_commands_sent=0,
                  friction_scale=args.friction_scale,
                  rotated_world_axis=args.rotate_world_axis,
                  rotated_degrees=args.rotate_deg,
                  selected_offset_mm=args.offset_mm,
                  visual_finger_collision=args.visual_finger_collision,
                  side_sign_mask=args.side_sign_mask,
                  enabled_visual_finger_geoms=changed_visual,
                  disabled_primitive_finger_geoms=disabled_primitive,
                  candidate_count=len(candidate_results),
                  face_changed_count=sum(r['face_changed'] for r in candidate_results),
                  candidate_results=candidate_results,
                  caveat='Nearby candidate starts and an approximate contact model. No camera occlusion or motor dynamics fitted. Physical flip outcome must be compared separately.')
    if args.side_sign_mask:
        filename = f'sac_sequence_replay_side_sign_{args.side_sign_mask}_result.json'
    elif args.visual_finger_collision:
        filename = 'sac_sequence_replay_visual_finger_result.json'
    elif args.offset_mm is not None:
        tag = '_'.join(f'{v:g}' for v in args.offset_mm)
        filename = f'sac_sequence_replay_offset_{tag}_result.json'
    elif args.rotate_world_axis:
        filename = f'sac_sequence_replay_rotate_{args.rotate_world_axis}_{args.rotate_deg:g}_result.json'
    else:
        filename = ('sac_sequence_replay_sim_result.json' if args.friction_scale == 1 and not args.first_only
                    else f'sac_sequence_replay_friction_{args.friction_scale:g}_result.json')
    (ROOT / filename).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps([dict(offset_mm=r['offset_mm'], face_changed=r['face_changed'],
                           final_face=r['final_face'], displacement_mm=r['final_displacement_mm'])
                      for r in candidate_results], indent=2))


if __name__ == '__main__':
    main()
