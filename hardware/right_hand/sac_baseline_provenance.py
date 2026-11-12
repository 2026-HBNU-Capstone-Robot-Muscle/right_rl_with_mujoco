"""Attach post-run hash to the completed, read-only exported-teacher audit."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
path=ROOT/'sac_stage0_teacher_v2_result.json'
report=json.loads(path.read_text())
assert report['teacher'] and report['trials']==10 and report['metric_version']=='forward_quaternion_v2'
teacher=ROOT.parent/'work'/'Leap_Test'/'policy'/'policy_np.npz'
report['teacher_sha256']=hashlib.sha256(teacher.read_bytes()).hexdigest()
report['teacher_hash_provenance']='Recorded after this completed audit; work/policy source was read-only and not changed during the run.'
path.write_text(json.dumps(report,indent=2)+'\n')
print('Teacher audit:',report['successes'],'/',report['trials'],report['teacher_sha256'])
