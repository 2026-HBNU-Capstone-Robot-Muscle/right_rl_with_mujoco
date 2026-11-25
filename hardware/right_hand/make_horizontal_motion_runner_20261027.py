"""Copy the existing runner and replace only its pre-enable settling block."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
source=ROOT/'camera_gap_motion_runner_20261026.py';target=ROOT/'horizontal_quarter_motion_runner_20261027.py';original=source.read_text(encoding='utf-8')
old="""        previous=None
        for _ in range(2):
            cancel();s=bus.snapshot()
            report.setdefault('off_preload_snapshots',[]).append({k:v.tolist() for k,v in s.items()})
            check_off(s,original,configured=True)
            if previous is not None and any(np.any(abs(s[k]-previous[k])>1) for k in ('position','goal')):
                raise RuntimeError('OFF preload unstable; no enable')
            previous=s;sleep(.02)
"""
assert original.count(old)==1
updated=original.replace(old,"        confirm_off_preload(bus,original,report,cancel=cancel,sleep=sleep,now=now)\n")
updated=updated.replace('from leap_roots_empty import check_off\n','from leap_roots_empty import check_off\nfrom horizontal_preload_confirmation_20261027 import confirm_off_preload\n',1)
marker='        bus.write(98,1,np.full(16,25))'
assert updated[updated.index(marker):]==original[original.index(marker):]
if target.exists() and target.read_text(encoding='utf-8')!=updated:raise ValueError('Refuse to replace a different dedicated runner')
target.write_text(updated,encoding='utf-8')
manifest=dict(original=source.name,original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),dedicated_runner=target.name,dedicated_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),all_code_from_watchdog_enable_through_motion_and_cleanup_identical=True,original_file_unchanged=True,devices_opened=False)
(ROOT/'right_hand_rotation_20261027/horizontal_preload_runner_diff.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8');print(json.dumps(manifest))
