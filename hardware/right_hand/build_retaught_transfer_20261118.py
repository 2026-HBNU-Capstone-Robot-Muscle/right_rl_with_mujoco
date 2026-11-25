"""Build the freshly taught combined clearance/transport, with exact old turn."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import BASE,reference,limits
from build_taught_palm_transfer_20261118 import interpolate
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'

def main():
    ref,taught=reference();z=np.load(FIRST,allow_pickle=False);grip=pose.reference()[2]
    offset=int(np.flatnonzero(z['phases']=='stage_7_horizontal_yaw')[0]);ready=z['goals'][offset-1].copy()
    index=taught.copy();index[4:8]=ready[4:8]
    thumb=index.copy();thumb[:4]=ready[:4]
    stages=[('taught_clear_transfer',taught),('release_index',index),
            ('thumb_catch',thumb),('middle_ring_handover',ready)]
    rows=[];phases=[];ends={};previous=grip
    for phase,target in stages:
        part=interpolate(previous,target);rows.extend(part);phases.extend([phase]*len(part));ends[phase]=len(rows)-1;previous=target
    prep=len(rows);rows.extend(z['goals'][offset:]);phases.extend('preserved_'+str(p) for p in z['phases'][offset:])
    goals=np.asarray(rows,dtype=int)
    with limits():validate_rows(grip,goals)
    assert np.array_equal(goals[prep:],z['goals'][offset:])
    np.savez_compressed(BASE/'candidate.npz',start=grip,goals=goals,phases=np.asarray(phases,dtype='<U64'))
    review=dict(kind='retaught_combined_clearance_and_transport',goals=len(goals),preparation_goals=prep,
        taught_end_index=ends['taught_clear_transfer'],phase_end_indices=ends,holding_goal_ticks=taught.tolist(),
        candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(FIRST.read_bytes()).hexdigest(),preserved_turn_source_offset=offset,
        preserved_turn_goals=len(z['goals'])-offset,raw_pose_clipped=False,old_thumb_clear_and_ring_push_removed=True,
        fixed_grip_return_included=False,physical_transfer_and_turn_verified=False)
    (BASE/'construction_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review))

if __name__=='__main__':main()
