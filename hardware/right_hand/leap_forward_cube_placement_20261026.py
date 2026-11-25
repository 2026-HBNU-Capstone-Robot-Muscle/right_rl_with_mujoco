"""Operator starts one placement comparison; imports and default mode open no devices."""
import sys
from forward_cube_placement_20261026 import ForwardPlacementPlan, require_placement_review
from leap_loaded_quarter_finish_trial_20261025 import LoadedFinishBus
from loaded_quarter_finish_gate_20261025 import run_screen
from quarter_current_runtime_20261025 import main

if __name__ == '__main__':
    if '--execute' in sys.argv:
        try:
            require_placement_review()
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
            print('PLACEMENT REVIEW NOT READY:', str(exc), '. No devices opened.', flush=True)
            raise SystemExit(2)
    print('FIRST SPACE: EMPTY hand. Cube outside hand and camera.', flush=True)
    print('Only at PLACE: about5mm toward FINGERTIPS from last placement. GREEN UP, ORANGE toward WRIST/ID41.', flush=True)
    print('Same824 loaded goals. No extra thumb/root motion or force increase. REMOVE before THIRD SPACE/OFF.', flush=True)
    raise SystemExit(main(ForwardPlacementPlan, bus_type=LoadedFinishBus, screener=run_screen))
