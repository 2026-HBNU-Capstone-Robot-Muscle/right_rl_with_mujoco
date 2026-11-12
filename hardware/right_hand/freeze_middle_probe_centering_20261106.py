"""Reuse the identical reference path; separately bind the bounded OFF centering."""
import json,hashlib
import numpy as np
from middle_probe_plan_20261106 import O,R,REQUEST,PROOF,MiddleProbePlan,hashes,sha
from middle_probe_gate_20261106 import make_review
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
def main():
    previous=O/'review.json';old=json.loads(previous.read_bytes());plan=MiddleProbePlan(REQUEST);empty=old['empty_recovery'];rows=[(p,q) for p,q in plan.plan(plan.source_position()) if p in PREFIX_PHASES]
    digest=hashlib.sha256(json.dumps([(p,q.tolist()) for p,q in rows],separators=(',',':')).encode()).hexdigest()
    if digest!=empty['prefix_sha256'] or not np.array_equal(plan.source_position(),empty['start_ticks']) or len(rows)!=empty['goals']:raise RuntimeError('Reference empty path changed; cannot reuse its matrix')
    if PROOF.exists():raise RuntimeError('Refuse to replace revised frozen proof')
    result=make_review(plan,empty,hashes());result['reference_path_equivalence']=dict(previous_review_sha256=sha(previous),identical_reference_prefix_sha256=digest,offset_centering_limit_ticks=4,existing_single_forward_centering_preserved=True,preload_uses_fresh_actual_position=True,enable_shift_and_health_checks_unchanged=True)
    # Extra provenance is permitted; validate_review checks every required key.
    PROOF.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(proof=PROOF.relative_to(R).as_posix(),reference_goals=len(rows),code_hashes=len(result['code_hashes']),devices_opened=False)))
if __name__=='__main__':main()
