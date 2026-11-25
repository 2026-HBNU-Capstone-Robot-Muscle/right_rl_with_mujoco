"""Refresh only stale console wording, proving model/control files unchanged."""
import hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review,make_review
plan=HorizontalQuarterPlan(REQUEST)
path=ROOT/'leap_horizontal_quarter_20261027.py'
a='v6: fresh current-pose recovery. Held-thumb/book numerical allowance <=0.02mm in EMPTY finite-PD screen only; commanded geometry must remain clear.'
b='v7: current pose is measured automatically; ten fresh stable EMPTY readings are required before PLACE.'
expected=json.loads(PROOF.read_bytes())['code_hashes'][path.name]
current=path.read_bytes()
if hashlib.sha256(current).hexdigest()!=expected:
    # Restore only bytes whose hash exactly matches the prior reviewed file.
    # A failed text-mode attempt changed line endings on Windows.
    candidates=[current.replace(b.encode(),a.encode())]
    candidates.append(candidates[0].replace(b'\r\n',b'\n'))
    original=next(v for v in candidates if hashlib.sha256(v).hexdigest()==expected)
    path.write_bytes(original)
old_review=validate_review(plan,PROOF);before=hashes();old=path.read_bytes()
assert old.count(a.encode())==1
path.write_bytes(old.replace(a.encode(),b.encode()));after=hashes()
changed=[name for name in before if before[name]!=after[name]]
assert changed==['leap_horizontal_quarter_20261027.py']
assert hashlib.sha256(path.read_bytes().replace(b.encode(),a.encode())).hexdigest()==before[changed[0]]
report=make_review(plan,old_review['empty_recovery'],after)
report['operator_banner_only_refresh']=dict(changed=changed,model_and_control_files_unchanged=True,
    previous_code_hashes=before,loaded_and_empty_targets_unchanged=True,motor_writes=0)
archive=OUT/'review_before_banner_refresh.json';archive.write_bytes(PROOF.read_bytes())
output=OUT/'review_banner_refreshed.json';output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
validate_review(plan,output);PROOF.write_bytes(output.read_bytes());validate_review(plan,PROOF)
print(json.dumps(dict(status='banner_only_review_refresh_verified',changed=changed,motor_writes=0)))
