"""Summarize saved comparison evidence; not a hardware or task admission."""
from pathlib import Path
import json
import math
import numpy as np
from sac_forward_geometry import measure

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/environment_transfer'


def main():
    training = json.loads((OUT / 'training_model.json').read_text())
    current = json.loads((OUT / 'current_model.json').read_text())
    focus = {}
    for name in ('palm_collision_4', 'palm_collision_8', 'th_tip', 'rf_tip', 'cube'):
        focus[name] = {key: {'training': training['geoms'][name][key], 'current': current['geoms'][name][key]}
                       for key in training['geoms'][name]
                       if training['geoms'][name][key] != current['geoms'][name][key]}
    archives = []
    for path in (ROOT / 'sac_supported_motion_search_20hz_result_mjx_eval.json',
                 ROOT / 'sac_forward_support_reference_v1_20k/supported_batch_eval_sac_cube55_resets_cube55mm.json'):
        doc = json.loads(path.read_text())
        case = doc['results'][0]
        trace = case['trace']
        start = np.asarray(trace[0]['qpos'][-4:])
        prefix = []
        for row in trace[:16]:
            if 'qpos' not in row:
                continue
            tip, axis, error = measure(start, np.asarray(row['qpos'][-4:]))
            prefix.append({'step': row['step'], 'tip_deg': math.degrees(float(tip)),
                           'target_error_deg': math.degrees(float(error))})
        archives.append({'source': str(path.relative_to(ROOT)),
                         'first_case_summary': {key: value for key, value in case.items() if key != 'trace'},
                         'archived_prefix': prefix, 'old_record_not_new_evaluation': True})
    report = {'focus_model_differences': focus, 'historical_prefixes': archives,
              'provider_verification': json.loads((OUT / 'observation_provider_verification.json').read_text()),
              'devices_opened': 0, 'new_training_executed': False, 'hardware_execution_allowed': False}
    (OUT / 'findings.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
