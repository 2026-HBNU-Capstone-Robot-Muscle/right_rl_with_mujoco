"""Full-hand preparation from a relaxed pose, distinct from the ID12 diagnostic.

Adds ID15 to bounded PIP recovery. Policy joint limits are never modified.
Each PIP recovery segment remains <=0.15rad inward at the existing 60mA.
At most two segments per PIP, <=0.21rad total including enable-shift allowance.
"""
import numpy as np
import hand12_posture_target_100 as runner

base, STOPS = runner.base, runner.STOPS
CAPS=runner.CAPS.copy()
CAPS[[7,15]]=80  # IDs8/16: explicit trial setting, never increased during a run.
CAPS.flags.writeable=False
PROFILE_VELOCITY=20
TICK = runner.TICK
PIP_SIM = (2, 6, 10)
PIP_HW = (6, 10, 14)
RECOVERY_SEGMENT_TICKS = int(np.floor(.15/TICK))-4
RECOVERY_TOTAL_RAD = .21


def plan(raw):
    raw = np.asarray(raw)
    if (raw.shape != (16,) or not np.isfinite(raw).all() or
            np.any(raw != np.rint(raw)) or np.any(raw < 0) or np.any(raw > 4095)):
        raise ValueError('Invalid single-turn encoder array; no movement')
    q = base.hw_to_sim(raw*TICK)
    lower = base.limits.SAFE_LOWERS.copy()
    lower[[2, 3, 6, 7, 10, 11]] = -.34
    lower[list(PIP_SIM)] = -.50
    bad = np.flatnonzero((q < lower) | (q > base.limits.SAFE_UPPERS))
    if len(bad):
        details = '; '.join(f'ID{int(base.SIM2ARR[i])+1}: {q[i]:+.4f}rad '
                            f'outside [{lower[i]:.3f}, {base.limits.SAFE_UPPERS[i]:.3f}]' for i in bad)
        raise ValueError('Outside preparation envelope: '+details)
    raw = raw.astype(int)
    staged = raw.copy()
    moves = []
    for i in PIP_HW:
        if raw[i]*TICK-base.OFFSET < -.32:
            target = min(int(np.ceil((base.OFFSET-.30)/TICK)),
                         int(raw[i])+RECOVERY_SEGMENT_TICKS)
            staged[i] = target
            moves.append((i, target))
            # A single segment cannot cover the low end of the already allowed
            # -.50rad start envelope. Continue once at the same current/speed,
            # retaining .02rad settling room above the original -.34 boundary.
            ready_raw = int(np.ceil((base.OFFSET-.34)/TICK))+int(np.ceil(.02/TICK))
            if target < ready_raw:
                target = min(int(np.ceil((base.OFFSET-.30)/TICK)), target+RECOVERY_SEGMENT_TICKS)
                staged[i] = target
                moves.append((i, target))
            if (int(staged[i])-int(raw[i])+4)*TICK > RECOVERY_TOTAL_RAD:
                raise ValueError(f'ID{i+1}: PIP recovery exceeds 0.21rad total; no motion')
    # Final planned and measured positions must pass the ORIGINAL envelope.
    goal = base.target_plan(staged)
    if np.max(np.abs(goal-raw))*TICK > 1.40:
        raise ValueError('Total staged travel exceeds 1.40rad; no motion')
    return moves, staged, goal


class Bus(runner.Bus):
    planner = staticmethod(plan)
    current_caps = CAPS


def settle_preload(bus, original, report, *, cancel, now, sleep, current_caps=CAPS, profile_velocity=PROFILE_VELOCITY):
    """Bounded OFF-only settling; the original reference and goals never move."""
    began=now()
    recent=[]
    report['preload_settle_limits']=dict(total_shift_ticks=4, goal_position_ticks=1, timeout_s=.5)
    while True:
        cancel()
        read_began=now()
        state=bus.snapshot()
        report['preload_samples'].append(runner.pack(state))
        if now()-read_began>.2 or now()-began>=.5:
            raise RuntimeError('OFF preload did not settle within read/time limits; no enable')
        # Retain all OFF/health/config checks and the one-tick goal/position
        # agreement. Check absolute drift against the ORIGINAL request below.
        current_goal=base.validate_preload(state,state['position'],expected_current_ma=current_caps,
                                          expected_profile_velocity=profile_velocity)
        for name in ('position','goal'):
            if np.any(np.abs(state[name]-original)>4):
                raise RuntimeError('OFF preload shift exceeds 4 ticks from original request; no enable')
        plan(state['position'])
        plan(current_goal)
        recent.append(state)
        recent=recent[-3:]
        if (len(recent)==3 and all(np.max(np.ptp(np.array([s[name] for s in recent]),axis=0))<=4
                                  for name in ('position','goal'))):
            report['accepted_preload_goal_raw']=current_goal.tolist()
            report['preload_shift_ticks']=(current_goal-original).tolist()
            return current_goal
        sleep(.02)


def run_trial(bus, initial, report, **kwargs):
    return runner.run_trial(bus, initial, report, planner=plan, full_start=True,
                            start_step_ticks=8, preload_check=settle_preload,
                            current_caps=CAPS, profile_velocity=PROFILE_VELOCITY, **kwargs)
