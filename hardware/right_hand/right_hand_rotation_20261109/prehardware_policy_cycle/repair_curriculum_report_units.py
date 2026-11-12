"""Repair the initial-angle label in saved curriculum reports without retraining."""
import json
import numpy as np
from native_cycle import OUT, save_json, sha
from next_turn_env import NextTurn


def main():
    repairs = []
    for parent in sorted(OUT.glob('sac_next_*')):
        if not (parent / 'settings.json').exists():
            continue
        settings = json.loads((parent / 'settings.json').read_text())
        env = NextTurn(settings['curriculum_increment_deg'])
        _, obs = env._reset_one(np.array([0, 0], dtype=np.uint32))
        radians = float(obs[107])
        degrees = float(np.degrees(obs[107]))
        for path in sorted(parent.glob('evaluation_*/review.json')):
            r = json.loads(path.read_text())
            if 'initial_angle_unit_correction' in r:
                continue
            previous = r.get('initial_goal_error_deg')
            if previous is not None and abs(previous-radians) > 1e-6:
                raise AssertionError((str(path), previous, radians))
            correction = dict(original_mislabelled_value=previous,
                              original_value_was_radians=previous is not None,
                              corrected_initial_goal_error_deg=degrees,
                              method='Recompute identical deterministic source-state reset; no motion evaluation repeated',
                              evaluation_outcome_and_trace_unchanged=True)
            r['initial_goal_error_deg'] = degrees
            r['initial_angle_unit_correction'] = correction
            save_json(path, r)
            repairs.append(dict(path=str(path.relative_to(OUT)), **correction))
    save_json(OUT / 'curriculum_angle_unit_corrections.json', dict(repairs=repairs, devices_opened=0))
    print(json.dumps(repairs), flush=True)


if __name__ == '__main__':
    main()
