"""Original1x vertical motion with a bounded READ-only pause and same-goal resume."""
import hashlib
import json
from pathlib import Path
import sys

import calibrated_two_full_health_20261028 as calibrated
import leap_vertical_direct_grip_recovery_20261125 as base
from grip_pause_resume_20261125 import PauseBus, sample

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/communication_pause_resume'


def validate_sources():
    manifest = json.loads((BASE/'manifest.json').read_bytes())
    for name, expected in manifest['source_hashes'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Pause/resume source changed: '+name)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        validate_sources()
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print('NOT READY:', exc, 'No devices opened.')
        return 2
    old_bus, old_sample = base.RecoveryBus, calibrated.sample
    try:
        base.RecoveryBus, calibrated.sample = PauseBus, sample
        if '--execute' in argv:
            print('통신 대기/재개판: 경로·1배속 동일. 통신 오류 후 최대1초 동안 READ만 재시도합니다.', flush=True)
            print('전체16개 정상 상태2회·같은 목표·자세 확인 후 자동 재개. 응답 없음/자세 변화/워치독 오류면 중단합니다.', flush=True)
        return base.main(argv)
    finally:
        base.RecoveryBus, calibrated.sample = old_bus, old_sample


if __name__ == '__main__':
    raise SystemExit(main())
