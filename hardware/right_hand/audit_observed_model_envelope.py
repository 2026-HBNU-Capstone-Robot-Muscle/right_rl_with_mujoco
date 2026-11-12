"""Offline sensitivity of the observed OFF pose to stock model assumptions.

No motor or camera API. This produces diagnostics, never a runtime certificate.
The joint-range variant includes an actually observed encoder neighborhood.
The contact thresholds are hypotheses; contact forces and meshes stay unchanged.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_empty_preparation_plan import offsets, FRONTS, MAX_INITIAL_SUPPORT_MM, ALLOWED_SUPPORT_PAIRS
from book_grasp_trial_plan import (BookGraspPlan, THUMB_RELEASE, SELF_PAIRS,
    MAX_INITIAL_THUMB_SELF_MM, THUMB_ESCAPE_GOALS, SUPPORT_ROOT_ESCAPE_GOALS,
    INITIAL_CONTACT_TRACKING_RAD, LOADED_PHASE, LIFT)
from book_support_model_20261024 import build_book
from screen_book_grasp_trial import cached_contacts
from initial_contact_escape import ContactEscape
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'book_settled_off_readonly_20261024_174424_563949/request.json'
OUT = ROOT/'right_hand_rotation_20261024/observed_model_envelope_audit'
PAIR = 'palm_collision_7|rf_tip'
TOLERANCES = (0., .05, .1, .25, .5)


def main():
    began = time.perf_counter()
    OUT.mkdir(exist_ok=True)
    source_bytes = SOURCE.read_bytes()
    request = json.loads(source_bytes)
    raw = np.array(request['off_state']['position'], dtype=int)
    limits = json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lower, upper = np.array(limits['lower_ticks']), np.array(limits['upper_ticks'])
    observed_lower = int(raw[3]-4)
    expanded_lower = lower.copy()
    expanded_lower[3] = observed_lower

    class DiagnosticPlan(BookGraspPlan):
        @staticmethod
        def validate_admission(values):
            a = np.asarray(values)
            if (a.shape != (16,) or not np.isfinite(a).all() or np.any(a != np.rint(a))
                    or np.any(a < expanded_lower) or np.any(a > upper)):
                raise ValueError('Outside diagnostic observed neighborhood')
            return a.astype(int)

        @staticmethod
        def validate_off_state(state):
            if any(np.any(state[k]) for k in ('torque', 'error', 'watchdog')):
                raise ValueError('Source is not an OFF observation')
            return DiagnosticPlan.validate_admission(state['position'])

    plan = DiagnosticPlan(SOURCE)
    paths = []
    for offset in offsets():
        start = raw+offset
        rows = plan.plan(start)
        samples = np.vstack([start]+[q for _, q in rows])
        paths.append(dict(offset=offset, rows=rows, angles=ticks_to_sim(samples),
            tip_count=sum(p == THUMB_RELEASE for p, _ in rows),
            lift_count=sum(p == LIFT for p, _ in rows),
            max_step=int(np.max(abs(np.diff(samples, axis=0)), initial=0))))
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    results = {str(t): dict(static=[], dynamic=[]) for t in TOLERANCES}
    raw_ring_max = 0.

    for front in FRONTS:
        m, _ = build_book(env, OUT/'diagnostic_scene.xml', book_projection_mm=front,
            book_ylo_mm=-500., book_yhi_mm=500., support_projection_mm=50.)
        j = m.joint('th_cmc').id
        act = m.actuator('th_cmc_act').id
        old_joint = m.jnt_range[j].copy()
        old_ctrl = m.actuator_ctrlrange[act].copy()
        m.jnt_range[j, 0] = ticks_to_sim(raw)[12] - 4*2*np.pi/4096
        m.actuator_ctrlrange[act, 0] = m.jnt_range[j, 0]
        assert np.isclose(m.jnt_range[j, 0], (observed_lower-2048)*2*np.pi/4096)
        m.geom_contype[m.geom('cube').id] = 0
        m.geom_conaffinity[m.geom('cube').id] = 0
        hand = {m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:
                hand.add(b)
        support = {m.geom(n).id for n in ('support_book', 'support_glass', 'support_wood')}
        inspect = cached_contacts(m, hand, support)
        joints = m.actuator_trnid[:, 0]
        lo = np.maximum(m.jnt_range[joints, 0], m.actuator_ctrlrange[:, 0])
        hi = np.minimum(m.jnt_range[joints, 1], m.actuator_ctrlrange[:, 1])

        def pose(d, angles):
            d.qpos[:16] = angles
            d.qpos[-7:-4] = [0, 0, 2]
            d.qpos[-4:] = [1, 0, 0, 0]
            mujoco.mj_forward(m, d)

        def excess(q):
            return float(max(0., np.max(lo-q), np.max(q-hi)))

        def filtered(pairs, tolerance):
            return {p: v for p, v in pairs.items() if p != PAIR or v > tolerance}

        def trackers(pairs, self_pairs, support_deadline, self_deadline, monotonic):
            return {t: (ContactEscape(pairs, ALLOWED_SUPPORT_PAIRS, MAX_INITIAL_SUPPORT_MM,
                        support_deadline, monotonic=monotonic),
                    ContactEscape(filtered(self_pairs, t), SELF_PAIRS, MAX_INITIAL_THUMB_SELF_MM,
                        self_deadline, monotonic=monotonic)) for t in TOLERANCES}

        def collect_checks(track, self_after, support_after, native_excess, max_step=0,
                           tracking=0., tracking_after=0., finite=True):
            checks = {}
            for t, (support_escape, self_escape) in track.items():
                sr, rr = support_escape.result(), self_escape.result()
                checks[t] = bool(finite and self_after[t] <= 1e-6 and support_after <= .15
                    and native_excess <= 1e-6 and max_step <= 2
                    and sr['self_escape_max_unapproved_overlap_mm'] <= 1e-6
                    and rr['self_escape_max_unapproved_overlap_mm'] <= 1e-6
                    and sr['self_escape_complete'] and rr['self_escape_complete']
                    and tracking <= INITIAL_CONTACT_TRACKING_RAD and tracking_after <= .12)
            return checks

        for path in paths:
            d = mujoco.MjData(m)
            pose(d, path['angles'][0])
            pairs, self_pairs = inspect(d)
            deadline = path['tip_count'] + min(path['lift_count'], SUPPORT_ROOT_ESCAPE_GOALS)
            track = trackers(pairs, self_pairs, deadline, min(path['tip_count'], THUMB_ESCAPE_GOALS), True)
            self_after = {t: 0. for t in TOLERANCES}
            support_after = 0.
            for index, ((phase, _), q) in enumerate(zip(path['rows'], path['angles'][1:]), 1):
                pose(d, q)
                pairs, self_pairs = inspect(d)
                raw_ring_max = max(raw_ring_max, self_pairs.get(PAIR, 0.))
                for t, (se, re) in track.items():
                    sf = filtered(self_pairs, t)
                    se.observe(pairs, index)
                    re.observe(sf, index)
                    if phase != THUMB_RELEASE:
                        self_after[t] = max(self_after[t], max(sf.values(), default=0.))
                if phase not in (THUMB_RELEASE, LIFT):
                    support_after = max(support_after, max(pairs.values(), default=0.))
            native_excess = float(max(0., np.max(lo-path['angles']), np.max(path['angles']-hi)))
            checks = collect_checks(track, self_after, support_after, native_excess, path['max_step'])
            for t in TOLERANCES:
                results[str(t)]['static'].append(dict(passed=checks[t], book_front_mm=front,
                    offset_ticks=path['offset'].tolist(), max_self_after_release_mm=self_after[t],
                    max_support_after_lift_mm=support_after, native_violation_rad=native_excess,
                    self_escape=track[t][1].result()))
        print(f'front {front}: 67 static variants complete', flush=True)

        path = paths[0]
        dynamics = [('initial_hold', path['angles'][0])]*10
        inserted = False
        for (phase, _), q in zip(path['rows'], path['angles'][1:]):
            if phase == LOADED_PHASE and not inserted:
                dynamics += [('placement_hold', dynamics[-1][1])]*40
                inserted = True
            dynamics.append((phase, q))
        dynamics += [('final_hold', path['angles'][-1])]*20
        gains, bias = m.actuator_gainprm.copy(), m.actuator_biasprm.copy()
        for scale in (.5, 1., 1.5):
            m.actuator_gainprm[:] = gains
            m.actuator_biasprm[:] = bias
            m.actuator_gainprm[:, 0] *= scale
            m.actuator_biasprm[:, 1] *= scale
            d = mujoco.MjData(m)
            pose(d, path['angles'][0])
            d.ctrl[:] = path['angles'][0]
            pairs, self_pairs = inspect(d)
            tip_s = path['tip_count']*.05
            track = trackers(pairs, self_pairs, .5+tip_s+min(path['lift_count']*.05, 3.), .5+tip_s, False)
            self_after = {t: 0. for t in TOLERANCES}
            support_after = native_excess = tracking = tracking_after = elapsed = 0.
            joint_excess = np.zeros(16)
            finite = True
            for phase, q in dynamics:
                d.ctrl[:] = q
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m, d)
                    elapsed += m.opt.timestep
                    if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()):
                        finite = False
                        break
                    pairs, self_pairs = inspect(d)
                    raw_ring_max = max(raw_ring_max, self_pairs.get(PAIR, 0.))
                    for t, (se, re) in track.items():
                        sf = filtered(self_pairs, t)
                        se.observe(pairs, elapsed)
                        re.observe(sf, elapsed)
                        if phase not in ('initial_hold', THUMB_RELEASE):
                            self_after[t] = max(self_after[t], max(sf.values(), default=0.))
                    tracking = max(tracking, float(max(abs(d.qpos[:16]-q))))
                    native_excess = max(native_excess, excess(d.qpos[:16]))
                    joint_excess = np.maximum(joint_excess,
                        np.maximum(np.maximum(lo-d.qpos[:16], d.qpos[:16]-hi), 0.))
                    if phase not in ('initial_hold', THUMB_RELEASE, LIFT):
                        support_after = max(support_after, max(pairs.values(), default=0.))
                        tracking_after = max(tracking_after, float(max(abs(d.qpos[:16]-q))))
                if not finite:
                    break
            checks = collect_checks(track, self_after, support_after, native_excess,
                tracking=tracking, tracking_after=tracking_after, finite=finite)
            # MuJoCo joint limits are soft constraints. Keep the strict result,
            # and separately report a sub-encoder-resolution residual analysis.
            # Commands/static poses must still remain entirely inside bounds.
            other_checks = collect_checks(track, self_after, support_after, 0.,
                tracking=tracking, tracking_after=tracking_after, finite=finite)
            for t in TOLERANCES:
                results[str(t)]['dynamic'].append(dict(passed=checks[t], book_front_mm=front,
                    passed_with_one_tick_soft_joint_residual=bool(other_checks[t]
                        and native_excess <= 2*np.pi/4096),
                    gain_scale=scale, tracking_rad=tracking, tracking_after_rad=tracking_after,
                    max_self_after_release_mm=self_after[t], native_violation_rad=native_excess,
                    joint_residual_ticks={str(SIM_TO_MOTOR_IDS[i]): float(v*4096/(2*np.pi))
                        for i, v in enumerate(joint_excess) if v > 1e-9},
                    max_support_after_lift_mm=support_after, self_escape=track[t][1].result()))
            print(f'front {front} gain {scale}: {[checks[t] for t in TOLERANCES]}', flush=True)
        m.jnt_range[j] = old_joint
        m.actuator_ctrlrange[act] = old_ctrl

    summary = {t: {k: dict(passed=sum(r['passed'] for r in rows), total=len(rows))
                   for k, rows in groups.items()} for t, groups in results.items()}
    for t, groups in results.items():
        summary[t]['dynamic_with_one_tick_soft_joint_residual'] = dict(
            passed=sum(r['passed_with_one_tick_soft_joint_residual'] for r in groups['dynamic']),
            total=len(groups['dynamic']))
    result = dict(source=str(SOURCE.relative_to(ROOT)), source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        observed_ticks=raw.tolist(), original_ID4_lower_ticks=float(lower[3]),
        diagnostic_ID4_lower_ticks=observed_lower, observed_below_original_degrees=float((lower[3]-raw[3])*360/4096),
        contact_tolerance_pair=PAIR, raw_max_ring_overlap_mm=raw_ring_max,
        summary=summary, cases=results, native_ranges_unchanged=False,
        joint_calibration_verified=False, contact_tolerance_calibrated=False,
        original_runtime_files_changed=False, collision_forces_and_meshes_unchanged=True,
        hardware_commands_sent=0, hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began,
        note='Sensitivity only: measured encoder existence does not establish a safe driven joint range. '
             'Contact thresholds are diagnostic hypotheses, not measured geometric accuracy.')
    (OUT/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(dict(summary=summary, raw_max_ring_overlap_mm=raw_ring_max,
        seconds=result['elapsed_seconds'])), flush=True)


if __name__ == '__main__':
    main()
