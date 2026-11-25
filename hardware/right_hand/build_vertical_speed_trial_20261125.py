"""Build an isolated trial packet; never open a device or change original code."""
import hashlib
import json
from pathlib import Path

import numpy as np
import vertical_direct_grip_plan_20261118 as plan
from vertical_speed_retime_20261125 import select_rows

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/vertical_3x_separate_trial'


def main():
    BASE.mkdir(parents=True, exist_ok=True)
    previous = json.loads((BASE/'trial_manifest.json').read_bytes()) if (BASE/'trial_manifest.json').exists() else {}
    original = plan.VerticalDirectGrip()
    rows = original.rows(original.start)
    chosen, phases, indices, mandatory = select_rows(original.start, rows, original.labels)
    np.savez_compressed(BASE/'candidate.npz', start=original.start, goals=chosen,
                        phases=phases, source_indices=indices, mandatory=mandatory)
    sources = json.loads((ROOT/'right_hand_rotation_20261124/two_axis_cycle/operator_trial_manifest.json').read_bytes())['source_hashes']
    sources = dict(sources)
    for name in ['vertical_speed_retime_20261125.py', 'vertical_speed_trial_plan_20261125.py',
                 'leap_vertical_speed_trial_20261125.py', 'build_vertical_speed_trial_20261125.py',
                 'test_vertical_speed_trial_20261125.py']:
        sources[name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    manifest = dict(kind='isolated_vertical_speed_trial', source_hashes=sources,
        candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        baseline_candidate_sha256=original.manifest['candidate_sha256'],
        operator_authorized_maximum_step_ticks=previous.get('operator_authorized_maximum_step_ticks'),
        authorization_basis=previous.get('authorization_basis'),
        period_seconds=.075, proposed_maximum_step_ticks=6, factor=3,
        source_goals=len(rows), selected_goals=len(chosen),
        nominal_source_seconds=.075*len(rows), nominal_trial_seconds=.075*len(chosen),
        physical_execution_performed=False, original_source_files_modified=False,
        original_transport_preserved=True, goal_step_eligibility_is_explicit_experiment=True)
    (BASE/'trial_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in manifest.items() if k != 'source_hashes'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
