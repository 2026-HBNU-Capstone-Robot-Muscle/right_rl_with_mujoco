"""Historical 150mA contact profile for regression/replay; active launchers use hand_prepare_full."""
from functools import partial
import hand_prepare_resume as prep

base, plan = prep.base, prep.plan
TOTAL_ON_SECONDS = 90
CAPS, STOPS = prep.CAPS.copy(), prep.STOPS.copy()
LOADED_IDS = (3, 8, 12, 16)
for motor in LOADED_IDS:
    CAPS[motor-1], STOPS[motor-1] = 150, 180
CAPS.flags.writeable = STOPS.flags.writeable = False


class Bus(prep.Bus):
    current_caps = CAPS


def run_trial(bus, initial, report, **kwargs):
    return prep.runner.run_trial(bus, initial, report, planner=plan, full_start=True,
        start_step_ticks=8, preload_check=partial(prep.settle_preload, current_caps=CAPS),
        current_caps=CAPS, measured_current_stops=STOPS,
        profile_velocity=prep.PROFILE_VELOCITY, total_on_seconds=TOTAL_ON_SECONDS, **kwargs)
