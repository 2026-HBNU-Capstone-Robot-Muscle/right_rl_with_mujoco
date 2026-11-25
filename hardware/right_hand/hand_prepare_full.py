"""Full XC330 profile, actual local wiring, stage-based preparation.

550mA follows LEAP's Python API; PID gains preserve the device's preflight values.
Recorded trials with the example's lower gains stopped at tracking plateaus.
The measured-current trip
650mA and preparation ramp are local choices, NOT manufacturer recommendations.
No hardware is opened at import. All changed RAM is restored after verified OFF.
"""
from functools import partial
import numpy as np
import hand_prepare_resume as prep

base, plan = prep.base, prep.plan
TOTAL_ON_SECONDS = None
CAPS = np.full(16,550,dtype=int)
STOPS = np.full(16,650,dtype=int)
PROFILE_VELOCITY = 60  # 13.74rpm motor profile; command ramp is <=28.125 deg/s.
PRESERVE_DEVICE_GAINS = True
for array in (CAPS,STOPS): array.flags.writeable=False


class RecoveringSteps:
    """Smooth start; halve on lag/load and restore after 0.4s of stable tracking."""
    def __init__(self, report, phase, *, caps=CAPS, max_step_ticks=16):
        self.maximum=max_step_ticks
        self.steps=np.full(16,min(4,max_step_ticks),dtype=int)
        self.caps=np.array(caps,copy=True)
        self.good_since=np.full(16,np.nan)
        self.report,self.phase=report,phase

    def update(self, error, current, stamp):
        error=np.asarray(error)
        current=np.asarray(current)
        current=np.abs(np.where(current>=32768,current-65536,current))
        next_steps=self.steps.copy()
        loaded=(current>=.9*self.caps)&(error>=.02)
        slow=(error>=.03)|loaded
        # One reduction per disturbance; stable recovery resets the hysteresis.
        next_steps[slow]=np.minimum(next_steps[slow],4)
        next_steps[error>=.04]=2
        good=(error<=.015)&(current<.8*self.caps)
        self.good_since[~good]=np.nan
        newly_good=good&np.isnan(self.good_since)
        self.good_since[newly_good]=stamp
        speedup=good&(stamp-self.good_since>=.4)&(next_steps<self.maximum)
        next_steps[speedup]=np.minimum(next_steps[speedup]*2,self.maximum)
        self.good_since[speedup]=stamp
        for i in np.flatnonzero(next_steps!=self.steps):
            reason='tracking_recovered' if next_steps[i]>self.steps[i] else 'load_or_tracking'
            self.report.setdefault('adaptive_step_events',[]).append(dict(
                phase=self.phase,t=float(stamp),motor_id=int(i)+1,
                step_ticks_before=int(self.steps[i]),step_ticks_after=int(next_steps[i]),reason=reason))
        self.steps=next_steps
        self.report.setdefault('final_step_ticks_by_phase',{})[self.phase]=self.steps.tolist()
        return self.steps.copy()


class Bus(prep.Bus):
    current_caps=CAPS
    read_timeout_recovery=True

    def snapshot(self):
        # Same contiguous read as legacy code, now also decode PID registers.
        return self.block(64,83,[('torque',64,1),('error',70,1),
            ('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),
            ('current_cap',102,2),('acceleration',108,4),('velocity',112,4),
            ('goal',116,4),('current',126,2),('position',132,4),
            ('voltage',144,2),('temperature',146,1)])


def run_trial(bus, initial, report, **kwargs):
    report.update(profile='full_xc330_550ma',operator_paced_contact=True,
                  gain_source='preserved_preflight_per_motor',
                  preparation_max_step_ticks=16,preparation_no_progress_limit_s=5,
                  current_trip_note='650mA measured input current; local trip, not official specification')
    return prep.runner.run_trial(bus,initial,report,planner=plan,full_start=True,
        start_step_ticks=16,recovery_step_ticks=8,adaptive_factory=RecoveringSteps,
        preload_check=partial(prep.settle_preload,current_caps=CAPS,profile_velocity=PROFILE_VELOCITY),
        current_caps=CAPS,measured_current_stops=STOPS,profile_velocity=PROFILE_VELOCITY,
        position_gains={name:initial.get(name) for name in ('p_gain','i_gain','d_gain')},
        total_on_seconds=None,preparation_limit_seconds=None,
        phase_timeouts=False,policy_step_ticks=32,**kwargs)
