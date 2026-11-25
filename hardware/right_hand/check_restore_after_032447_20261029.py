"""Compare actual OFF settings with the saved pre-motion originals, READ data only."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
run=ROOT/'retained_slow_yaw_20261029_032447_137236';r=json.loads((run/'result.json').read_bytes())
latest=json.loads((ROOT/'fast_off_readonly_20261029_033100_431974/result.json').read_bytes())
print('PREFLIGHT KEYS',list(r['preflight_state']));print('INITIAL TYPE',type(r['initial']).__name__)
expected=r['preflight_state'];actual=latest['snapshots'][-1]
keys=('d_gain','i_gain','p_gain','current_cap','acceleration','velocity')
comparison={k:dict(expected=expected.get(k),actual=actual.get(k),matches=expected.get(k)==actual.get(k)) for k in keys}
result=dict(saved_motion_result_sha256=__import__('hashlib').sha256((run/'result.json').read_bytes()).hexdigest(),
    actual_OFF_source='fast_off_readonly_20261029_033100_431974/result.json',comparison=comparison,
    all_torque_OFF=not any(actual['torque']),watchdog_zero=not any(actual['watchdog']),all_settings_match=all(v['matches'] for v in comparison.values()))
(run/'offline_review/restoration_readonly_check.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result))
