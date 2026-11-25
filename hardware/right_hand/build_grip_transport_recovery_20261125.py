"""Bind an isolated1x communication repair to untouched original sources."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/communication_repair'
NEW = ['grip_transport_recovery_20261125.py', 'leap_vertical_direct_grip_recovery_20261125.py',
       'serial_error_observer_20261125.py',
       'test_grip_transport_recovery_20261125.py', 'build_grip_transport_recovery_20261125.py',
       'leap_vertical_direct_grip_recovery.cmd']


def main():
    originals = json.loads((ROOT/'right_hand_rotation_20261125/vertical_3x_separate_trial/original_source_hashes.json').read_bytes())
    for name, value in originals.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != value:
            raise RuntimeError('Original source changed: '+name)
    source_hashes = dict(originals)
    source_hashes.update({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in NEW})
    BASE.mkdir(parents=True, exist_ok=True)
    value = dict(kind='isolated1x_communication_repair', source_hashes=source_hashes,
        original_sources_unchanged=len(originals), motion_period_s=.075, maximum_goal_step_ticks=2,
        current_limit_mA=450, PID_unchanged=True, fixed_G_and_motion_candidate_unchanged=True,
        candidate_sha256='41af28ada212563aa81558f081e5a7eb9f9712e7bd0868eca501327e65fa8591',
        original_launcher_unchanged=True, firmware_or_EEPROM_writes=False,
        full200ms_and_two_healthy_snapshots_preserved=True,
        hardware_failure_root_cause_not_claimed_resolved=True)
    (BASE/'manifest.json').write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in value.items() if k != 'source_hashes'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
