"""Bind pause/resume without changing the original motion or v2 recovery files."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/communication_pause_resume'
NEW = ['grip_pause_resume_20261125.py', 'leap_vertical_direct_grip_resume_20261125.py',
       'leap_vertical_direct_grip_resume.cmd', 'build_grip_pause_resume_20261125.py',
       'test_grip_pause_resume_20261125.py']


def main():
    original = json.loads((ROOT/'right_hand_rotation_20261125/communication_repair/manifest.json').read_bytes())
    hashes = original['source_hashes'].copy()
    for name, expected in hashes.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Existing source changed: '+name)
    hashes.update({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in NEW})
    BASE.mkdir(parents=True, exist_ok=True)
    report = dict(kind='isolated_same_goal_readonly_pause_resume', source_hashes=hashes,
        original_sources_unchanged=original['original_sources_unchanged'],
        v2_recovery_sources_unchanged=True, motion_and_fixed_G_unchanged=True,
        motion_period_s=.075, maximum_goal_step_ticks=2, current_limit_mA=450,
        maximum_extra_pause_s=1., pause_register_writes=0, watchdog_clear_or_extension=False,
        automatic_torque_enable=False, original_launchers_unchanged=True,
        actual_hardware_resume_not_yet_verified=True)
    (BASE/'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k != 'source_hashes'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
