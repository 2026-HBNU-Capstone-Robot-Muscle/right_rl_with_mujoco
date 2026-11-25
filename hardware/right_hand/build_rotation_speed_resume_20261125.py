"""Build a fresh authorized experiment; preserve the canceled earlier trial."""
import hashlib
import json

import numpy as np

from rotation_speed_resume_plan_20261125 import ROOT, BASE, RotationSpeedTrial
from grip_pause_resume_10s_trial_20261125 import POLICY as RECOVERY_POLICY

NEW = ['rotation_speed_resume_plan_20261125.py', 'leap_rotation_speed_resume_20261125.py',
       'leap_horizontal_speed_resume_3x.cmd', 'leap_vertical_speed_resume_3x.cmd',
       'build_rotation_speed_resume_20261125.py', 'test_rotation_speed_resume_20261125.py',
       'grip_pause_resume_10s_trial_20261125.py',
       'speed_trial_state_observer_20261125.py', 'test_speed_trial_state_observer_20261125.py',
       'vertical_speed_retime_20261125.py', 'vertical_speed_trial_plan_20261125.py']


def main():
    sources = json.loads((ROOT/'right_hand_rotation_20261125/communication_pause_resume/manifest.json').read_bytes())['source_hashes']
    for name, expected in sources.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Existing1x/recovery source changed: '+name)
    sources.update({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in NEW})
    for axis in ('horizontal', 'vertical'):
        sequence = RotationSpeedTrial(axis, verify=False)
        rows = sequence.rows(sequence.start)
        folder = BASE/axis
        folder.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(folder/'candidate.npz', start=sequence.start, goals=rows,
            phases=np.asarray(sequence.labels), source_indices=sequence.indices,
            mandatory=sequence.mandatory, baseline_goals=sequence.original_rows)
        manifest = dict(kind='new_separate3x_with_current_READ_recovery', axis=axis,
            source_hashes=sources, candidate_sha256=hashlib.sha256((folder/'candidate.npz').read_bytes()).hexdigest(),
            baseline_candidate_sha256=sequence.reference.manifest['candidate_sha256'],
            operator_authorized_maximum_step_ticks=6,
            authorization_basis='2026-11-25 user requested a separate three-times-speed rotation experiment',
            canceled_old_trial_unchanged=True, original1x_and_middleware_unchanged=True,
            maximum_read_recovery_pause_s=RECOVERY_POLICY['maximum_extra_pause_s'],
            physical_execution_performed=False, **sequence.metadata())
        (folder/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps({k:v for k,v in manifest.items() if k not in
            ('source_hashes', 'selected_source_indices', 'retained_mandatory_indices')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
