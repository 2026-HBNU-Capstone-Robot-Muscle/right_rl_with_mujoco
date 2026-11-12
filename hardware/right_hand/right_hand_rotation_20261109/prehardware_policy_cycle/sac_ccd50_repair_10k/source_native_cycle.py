"""Native offline model guards and reproducible cycle assets; no hardware."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import hashlib
import json
import time
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor, angle
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg

OUT = Path(__file__).resolve().parent
CONT = ROOT / 'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff'
ACTIVE_MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BOUND = ('qpos', 'qvel', 'ctrl', 'act', 'qacc_warmstart')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, report):
    pending = path.with_suffix('.tmp')
    pending.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    pending.replace(path)


def assets(model_kind='active'):
    report = json.loads((CONT / 'review.json').read_text())
    source_model = ROOT / report['source_model']
    source_initial = ROOT / report['source_initial_state']
    if sha(source_model) != report['source_model_sha256'] or sha(source_initial) != report['source_initial_state_sha256']:
        raise RuntimeError('Stored continuous-path provenance changed')
    candidate = CONT / 'candidate.npz'
    if sha(candidate) != report['candidate_sha256']:
        raise RuntimeError('Stored continuous reference changed')
    if model_kind=='active':model_path=ACTIVE_MODEL
    elif model_kind=='archived':model_path=source_model
    elif model_kind=='active_ccd50':
        variant_folder=OUT/'next_cycle/handoff_refinement_20261110/collision_convergence'
        variant_manifest=json.loads((variant_folder/'manifest.json').read_text())
        model_path=OUT/variant_manifest['variant_model_path']
        if sha(ACTIVE_MODEL)!=variant_manifest['source_model_sha256'] or sha(model_path)!=variant_manifest['variant_model_sha256']:
            raise RuntimeError('Collision-convergence model provenance changed')
    else:raise ValueError('Unknown offline model kind: '+model_kind)
    model = mujoco.MjModel.from_binary_path('offline.mjb', assets={'offline.mjb': model_path.read_bytes()})
    initial = dict(np.load(source_initial, allow_pickle=False))
    reference = dict(np.load(candidate, allow_pickle=False))
    lo, hi, qlo, qhi = native(model)
    if np.any(reference['goals'] < lo) or np.any(reference['goals'] > hi):
        raise RuntimeError('Archived reference violates unchanged native command limits')
    if np.max(np.abs(np.diff(np.vstack([reference['start'], reference['goals']]), axis=0))) > 2:
        raise RuntimeError('Archived reference violates two-tick spacing')
    return model, initial, reference, report, {
        'model_kind': model_kind, 'model_path': str(model_path.relative_to(ROOT)),
        'model_sha256': sha(model_path), 'initial_path': str(source_initial.relative_to(ROOT)),
        'initial_sha256': sha(source_initial), 'reference_sha256': sha(candidate),
        'source_is_archived_model_trajectory_not_physical_policy_success': True,
        'max_goal_ticks': 2, 'control_seconds': .075,
        'original_model_and_native_guards_preserved': model_kind!='active_ccd50',
        'original_source_model_file_preserved':True,
        'native_geometry_limits_and_guard_thresholds_preserved':True,
        'numerical_CCD_iterations':int(model.opt.ccd_iterations),
    }


class Guard:
    def __init__(self, model):
        self.m = model
        self.hand, self.supports, self.contacts = inspect_for(model)
        self.cube = model.geom('cube').id
        self.lo, self.hi, self.qlo, self.qhi = native(model)
        self.worst = dict(self_mm=0., support_mm=0., cube_mm=0., tracking_rad=0., range_rad=0.)
        self.gap = self.longest_gap = 0.
        self.last = {}

    def check(self, data):
        support, own = self.contacts(data)
        cube_pen = 0.; inhand = external = False
        for c in data.contact:
            a, b = int(c.geom1), int(c.geom2)
            if self.cube not in (a, b):
                continue
            other = b if a == self.cube else a
            if int(self.m.geom_bodyid[other]) in self.hand:
                inhand |= c.dist < .0004
                cube_pen = max(cube_pen, -float(c.dist) * 1000)
            elif other in self.supports and c.dist < .0004:
                external = True
        self.gap = 0. if inhand else self.gap + self.m.opt.timestep
        self.longest_gap = max(self.longest_gap, self.gap)
        now = dict(self_mm=max(own.values(), default=0.), support_mm=max(support.values(), default=0.),
                   cube_mm=max(0., cube_pen), tracking_rad=float(np.max(np.abs(data.qpos[:16] - data.ctrl))),
                   range_rad=float(max(0., np.max(self.qlo-data.qpos[:16]), np.max(data.qpos[:16]-self.qhi))))
        for name, value in now.items():
            self.worst[name] = max(self.worst[name], value)
        reasons = [name for name, bound in (('self_mm', 1e-6), ('support_mm', .15),
                   ('cube_mm', 1.), ('tracking_rad', .18), ('range_rad', 1e-6)) if now[name] > bound]
        if external:
            reasons.append('external_cube_support')
        if self.longest_gap > .25:
            reasons.append('hand_contact_lost')
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            reasons.append('nonfinite_state')
        self.last = dict(now, reasons=reasons, self_pairs_mm=own, support_pairs_mm=support,
                         hand_gap_s=self.gap, external_cube_support=external)
        return reasons


def model_differences(current, source_path):
    old = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': source_path.read_bytes()})
    result = {}
    for name in ('jnt_range', 'actuator_ctrlrange', 'actuator_forcelimited', 'actuator_forcerange',
                 'jnt_actfrclimited', 'jnt_actfrcrange', 'actuator_gainprm', 'actuator_biasprm',
                 'geom_size', 'geom_type', 'geom_contype', 'geom_conaffinity', 'geom_friction',
                 'geom_solref', 'body_mass', 'body_pos', 'body_quat'):
        a, b = getattr(old, name), getattr(current, name)
        if a.shape != b.shape:
            result[name] = {'source_shape': list(a.shape), 'current_shape': list(b.shape)}
        elif not np.array_equal(a, b):
            result[name] = {'maximum_abs_difference': float(np.max(np.abs(a.astype(float)-b.astype(float)))),
                            'different_entries': int(np.count_nonzero(a != b))}
    result['gravity'] = {'source': old.opt.gravity.tolist(), 'current': current.opt.gravity.tolist()}
    return result


def baseline(model_kind='active', mode='zero'):
    model, initial, reference, archived, provenance = assets(model_kind)
    folder = OUT / f'baseline_{model_kind}_{mode}'
    folder.mkdir(exist_ok=True)
    data = restore(model, initial)
    guard = Guard(model)
    goals = reference['goals']
    B0 = initial['initial_cube_rotation']
    target = upright_yaw_target(B0, initial['up'], 90)
    q = np.empty(4); mujoco.mju_mat2Quat(q, target.ravel())
    if mode == 'saved_sac':
        from sac_reference_numpy import ReferencePolicy
        from probe_reference_policy_from_training_start_20261109 import observation_type
        policy = ReferencePolicy(); builder = observation_type()()
        for _ in range(3):
            build_observation(builder, model, data, data.ctrl, q)
    states = []; indices = []; times = []; full = {key: [] for key in BOUND}; traces = []
    def record(index):
        indices.append(index); times.append(data.time)
        for key in BOUND:
            full[key].append(getattr(data, key).copy())
    record(0)
    began = time.perf_counter(); stop = None; previous = motor(data.ctrl)
    for index, nominal in enumerate(goals):
        if mode == 'saved_sac':
            obs = build_observation(builder, model, data, data.ctrl, q)
            phase = [index / max(1, len(goals)-1), float(index >= len(goals)-134)]
            action = policy.action(np.r_[obs, phase])
            proposed = np.rint(nominal + .05 * action[np.argsort(np.array(SIM_TO_MOTOR_IDS))] / TICK).astype(int)
            goal = previous + np.clip(proposed-previous, -2, 2)
            if np.any(goal < guard.lo) or np.any(goal > guard.hi):
                stop = ['native_goal_range']; break
        else:
            goal = nominal
        prior_ctrl = data.ctrl.copy()
        data.ctrl[:] = ticks_to_sim(goal)
        if mode == 'saved_sac':
            builder.set_last_act(np.clip((data.ctrl-prior_ctrl)/.2, -1, 1))
        for _ in range(round(.075/model.opt.timestep)):
            mujoco.mj_step(model, data)
            stop = guard.check(data)
            if stop:
                break
        previous = goal.copy()
        if (index+1) % 50 == 0 or stop or index+1 == len(goals):
            record(index+1)
            B = data.xmat[model.body('cube').id].reshape(3,3)
            traces.append({'goals': index+1, 'simulated_seconds': float(data.time),
                           'yaw_deg': angle(B, B0, initial['up']),
                           'target_error_deg': orientation_error_deg(B, target),
                           'center': data.qpos[-7:-4].tolist()})
        if (index+1) % 250 == 0 or stop:
            print(json.dumps({'mode': mode, 'model': model_kind, 'goals': index+1,
                               'stop': stop, 'elapsed_seconds': time.perf_counter()-began}), flush=True)
        if stop:
            break
    np.savez_compressed(folder/'state_pool.npz', indices=indices, time=times, **full,
                        up=initial['up'], initial_cube_rotation=B0,
                        initial_cube_center=initial['initial_cube_center'])
    np.savez_compressed(folder/'end_state.npz', **{key:getattr(data,key) for key in BOUND},
                        time=data.time, up=initial['up'], initial_cube_rotation=B0,
                        initial_cube_center=initial['initial_cube_center'])
    B = data.xmat[model.body('cube').id].reshape(3,3)
    report = dict(status='offline_baseline_complete', mode=mode, provenance=provenance,
                  source_model_differences=model_differences(model, ROOT/archived['source_model']),
                  completed_goals=index+1, total_goals=len(goals), stop=stop,
                  guard=guard.last, worst=guard.worst, trace=traces,
                  final_yaw_deg=angle(B, B0, initial['up']), target_error_deg=orientation_error_deg(B,target),
                  upper_face_tilt_deg=face_up_error_deg(B,5,initial['up']),
                  elapsed_seconds=time.perf_counter()-began, devices_opened=0,
                  task_success_certified=False, hardware_execution_allowed=False,
                  saved_sac_is_transfer_test_with_new_reference=mode=='saved_sac')
    save_json(folder/'review.json', report)
    print(json.dumps({key:report[key] for key in ('status','mode','completed_goals','stop','final_yaw_deg','target_error_deg','elapsed_seconds')}), flush=True)
    return report


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(); p.add_argument('--model', choices=('active','archived'), default='active')
    p.add_argument('--mode', choices=('zero','saved_sac'), default='zero')
    a = p.parse_args(); baseline(a.model,a.mode)
