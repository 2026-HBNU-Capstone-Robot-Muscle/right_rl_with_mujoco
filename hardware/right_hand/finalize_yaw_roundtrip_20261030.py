"""Freeze the fully reviewed experiment; no camera, port or motor writes."""
import json
from yaw_roundtrip_plan_20261030 import OUT,REQUEST,PROOF,YawRoundtripPlan,hashes,sha
from yaw_roundtrip_gate_20261030 import make_review,validate_review
def main():
    if PROOF.exists():raise RuntimeError('Refuse to overwrite frozen roundtrip proof')
    p=YawRoundtripPlan(REQUEST);empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes());r=make_review(p,empty,hashes())
    PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');checked=validate_review(p,PROOF)
    print(json.dumps(dict(status=checked['status'],proof_sha256=sha(PROOF),whole_motion_s=checked['nominal_motion_seconds'],
        cube_cases=checked['composed_cube_models']['cases'],hand_cases=checked['composed_hand_models']['cases'],inverse90_ready=False,devices_opened=False)))
if __name__=='__main__':main()
