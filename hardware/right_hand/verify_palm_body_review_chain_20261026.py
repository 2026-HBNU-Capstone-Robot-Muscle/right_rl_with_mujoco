"""Validate saved proofs without opening any device or authorizing motion."""
import json
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan,extra_hashes
from leap_thumb_clear_root_empty_20261026 import require_review
from record_empty_camera_gap_readiness_20261026 import sha,write,read
def main():
    print('Checking saved review chain only; no devices opened.',flush=True)
    proof=require_review();p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position()
    failed=read(ROOT/'thumb_clear_root_empty_20261026_123524_339920/current_start_screen.json')
    if failed['start_ticks']!=raw.tolist() or failed['plan_sha256']!=p.plan_digest(raw):raise ValueError('Actual failed goals changed')
    review=dict(accepted=True,code_hashes=extra_hashes(),native_proof_sha256=sha(FIXTURE/'screen.json'),
        source_sha256=p.SOURCE_SHA,goal_digest=p.plan_digest(raw),actual_failed_goal_digest_unchanged=True,
        static_cases=len(proof['static_screens']),normal_cases=len(proof['dynamic_sensitivity']),
        hold_cases=len(proof['camera_gap_hold_sensitivity']),hardware_commands_sent=0,devices_opened=False)
    write(FIXTURE/'review_chain.json',review);print(json.dumps({k:v for k,v in review.items() if k!='code_hashes'}),flush=True)
if __name__=='__main__':main()
