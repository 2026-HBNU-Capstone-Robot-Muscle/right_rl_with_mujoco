"""Offline MuJoCo replay driven by archived *measured* joint angles.

This replays the joint motion already observed on the real hand, including any
external-support obstruction. It does not model the wood itself, predict new
commands, or open a motor/camera device.
"""

import json
import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim

RUN = ROOT / 'leap_sequence_flip_20261011_051518_433595.json'
TICK_RAD = 2 * np.pi / 4096
GREEN = np.asarray([0., -1., 0.])
ORANGE = np.asarray([-1., 0., 0.])


def rotation(quat):
    values = np.empty(9)
    mujoco.mju_quat2Mat(values, np.asarray(quat))
    return values.reshape(3, 3)


def angle(a, b):
    d = np.asarray(a).T @ np.asarray(b)
    return float(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--offset-mm', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--stage2-same-session', action='store_true',
                        help='Initialize from October 11 same-run stage-2 cube and encoder pose')
    parser.add_argument('--friction-scale', type=float, default=1.0)
    args = parser.parse_args()
    if args.friction_scale <= 0:
        parser.error('friction scale must be positive')
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    model.geom_friction[:, 0] *= args.friction_scale
    tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                       'tilt_result.json').read_text())
    model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
    reg = json.loads((ROOT / ('sac_stage2_anchor_registration_result.json'
                              if args.stage2_same_session else
                              'sac_cube_rim_register_1010_stage0_result.json')).read_text())
    search = json.loads((ROOT / 'sac_measured_gravity_local_search_result.json').read_text())
    run = json.loads(RUN.read_text())
    rows = [r for r in run['samples'] if r['phase'] in
            {f'stage_{i}' for i in range(1, 7)}]
    rows.sort(key=lambda r: r['t'])
    t = np.asarray([r['t'] for r in rows], float)
    q = np.asarray([hw_to_sim(np.asarray(r['actual']) * TICK_RAD) for r in rows])
    # Repeat the same approximate candidate reset used by the command replay
    # so that the difference between replays is attributable to the trace.
    offset_mm = ([0, 0, 0] if args.stage2_same_session and args.offset_mm is None
                 else search['plausible'][0]['offset_mm'] if args.offset_mm is None
                 else args.offset_mm)
    offset = np.asarray(offset_mm) / 1000
    cube_start = np.asarray(reg['cube_center_world_m']) + offset
    data = mujoco.MjData(model)
    first_stage = 2 if args.stage2_same_session else 1
    initial_q = (q[np.flatnonzero(np.asarray([r['phase'] == 'stage_2' for r in rows]))[-1]]
                 if args.stage2_same_session else q[0])
    data.qpos[:16] = initial_q
    data.qpos[-7:-4] = cube_start
    data.qpos[-4:] = np.asarray(reg['cube_quat_world_candidate'])
    data.ctrl[:16] = initial_q
    mujoco.mj_forward(model, data)
    cube_body = model.body('cube').id
    initial_cube_contacts = [float(c.dist) for c in data.contact
                             if model.geom_bodyid[c.geom1] == cube_body
                             or model.geom_bodyid[c.geom2] == cube_body]
    hold = mujoco.MjData(model)
    hold.qpos[:] = data.qpos
    hold.ctrl[:] = data.ctrl
    mujoco.mj_forward(model, hold)
    for _ in range(30):
        mujoco.mj_step(model, hold)
    fixed_hold = dict(duration_s=30 * float(model.opt.timestep),
                      initial_cube_contacts=len(initial_cube_contacts),
                      initial_max_penetration_mm=float(-1000 * min(initial_cube_contacts, default=0)),
                      cube_center_drift_mm=float(1000 * np.linalg.norm(
                          hold.qpos[-7:-4] - data.qpos[-7:-4])))
    snapshots = {}
    contact_steps = {}
    previous_q = initial_q.copy()
    dt = float(model.opt.timestep)
    if args.stage2_same_session:
        snapshots['stage_2'] = dict(cube_center_m=data.qpos[-7:-4].tolist(),
                                    cube_quat_wxyz=data.qpos[-4:].tolist())
    for stage in range(first_stage if first_stage == 1 else 3, 7):
        stage_rows = [r for r in rows if r['phase'] == f'stage_{stage}']
        lo, hi = stage_rows[0]['t'], stage_rows[-1]['t']
        stage_finger_steps = 0
        stage_two_finger_steps = 0
        finger_step_counts = {name: 0 for name in ('th', 'if', 'mf', 'rf')}
        pair_step_counts = {}
        step_times = np.arange(lo, hi + dt / 2, dt)
        for step_time in step_times:
            desired = np.asarray([np.interp(step_time, t, q[:, j]) for j in range(16)])
            data.qpos[:16] = desired
            data.qvel[:16] = (desired - previous_q) / dt
            data.ctrl[:16] = desired
            mujoco.mj_forward(model, data)
            fingers = set()
            for c in data.contact:
                a, b = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                other = b if a == cube_body else a if b == cube_body else None
                if other is not None:
                    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(other))
                    prefix = body.split('_')[0]
                    if prefix in ('th', 'if', 'mf', 'rf'):
                        fingers.add(prefix)
            stage_finger_steps += bool(fingers)
            stage_two_finger_steps += len(fingers) >= 2
            for name in fingers:
                finger_step_counts[name] += 1
            if len(fingers) >= 2:
                pair = '+'.join(sorted(fingers))
                pair_step_counts[pair] = pair_step_counts.get(pair, 0) + 1
            mujoco.mj_step(model, data)
            previous_q = desired
        contact_steps[f'stage_{stage}'] = dict(sim_steps=len(step_times),
                                               any_finger_steps=int(stage_finger_steps),
                                               two_finger_steps=int(stage_two_finger_steps),
                                               finger_step_counts=finger_step_counts,
                                               multi_finger_set_counts=pair_step_counts)
        snapshots[f'stage_{stage}'] = dict(cube_center_m=data.qpos[-7:-4].tolist(),
                                          cube_quat_wxyz=data.qpos[-4:].tolist())
    a, b = snapshots['stage_2'], snapshots['stage_4']
    Ra, Rb = rotation(a['cube_quat_wxyz']), rotation(b['cube_quat_wxyz'])
    ga, gb = Ra @ GREEN, Rb @ GREEN
    oa, ob = Ra @ ORANGE, Rb @ ORANGE
    measure = dict(rotation_deg=angle(Ra, Rb),
                   green_normal_change_deg=float(np.degrees(np.arccos(
                       np.clip(ga @ gb, -1, 1)))),
                   green_tip_toward_initial_orange_front_deg=float(np.degrees(
                       np.arctan2(gb @ oa, gb @ ga))),
                   orange_front_turn_about_initial_green_axis_deg=float(np.degrees(
                       np.arctan2(np.cross(oa, ob) @ ga, oa @ ob))),
                   center_displacement_mm=float(1000 * np.linalg.norm(
                       np.asarray(b['cube_center_m']) - np.asarray(a['cube_center_m']))))
    report = dict(status='offline_recorded_encoder_replay', motor_commands_sent=0,
                  source_file=str(RUN), approximate_cube_start_m=cube_start.tolist(),
                  selected_offset_mm=list(offset_mm),
                  friction_scale=args.friction_scale,
                  stage2_same_session=args.stage2_same_session,
                  registration_file=('sac_stage2_anchor_registration_result.json'
                                     if args.stage2_same_session else
                                     'sac_cube_rim_register_1010_stage0_result.json'),
                  source_sample_count=len(rows), simulated_timestep_s=dt,
                  fixed_joint_hold_diagnostic=fixed_hold,
                  stage_2_to_4=measure, contact_steps=contact_steps,
                  snapshots=snapshots,
                  caveat='Kinematically imposed sparse real encoder trace; no external wood geom, exact cube reset, calibrated camera-to-model transform, or real contact forces. Diagnostic only.')
    filename = ('sac_recorded_encoder_replay_stage2_same_session_result.json'
                if args.stage2_same_session and args.offset_mm is None else
                'sac_recorded_encoder_replay_stage2_offset_' + '_'.join(
                    f'{x:g}' for x in offset_mm) + '_result.json'
                if args.stage2_same_session else
                'sac_recorded_encoder_replay_result.json' if args.offset_mm is None
                else 'sac_recorded_encoder_replay_offset_' + '_'.join(
                    f'{x:g}' for x in offset_mm) + '_result.json')
    if args.friction_scale != 1:
        filename = filename.replace('_result.json',
                                    f'_friction_{args.friction_scale:g}_result.json')
    (ROOT / filename).write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'snapshots'}, indent=2))


if __name__ == '__main__':
    main()
