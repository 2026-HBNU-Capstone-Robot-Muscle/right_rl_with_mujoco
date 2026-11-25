"""Preserve the raw source and stage the taught relocation and thumb-first pitch."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from vertical_transfer_plan_20261118 import BASE,PITCH,reference
from build_taught_palm_transfer_20261118 import interpolate
from measured_grip_small_cycle_plan_20261117 import validate_rows
from vertical_pitch_order_20261118 import reordered_pitch

def main():
    old=np.load(PITCH,allow_pickle=False);grip=pose.reference()[2];taught=reference();start=old['start']
    lifted=grip.copy();lifted[4:12]=taught[4:12]
    support=taught.copy();support[:4]=start[:4];support[12:16]=start[12:16]
    stages=[('lift_index_middle',lifted),('thumb_push_to_vertical',taught),
            ('thumb_ring_to_pitch_start',support),('index_middle_to_pitch_start',start)]
    rows=[];phases=[];ends={};previous=grip
    for phase,target in stages:
        part=interpolate(previous,target);rows.extend(part);phases.extend([phase]*len(part));ends[phase]=len(rows)-1;previous=target
    pitch,pitch_phases,order_review=reordered_pitch(old)
    preparation=len(rows);rows.extend(pitch);phases.extend('pitch_'+str(p) for p in pitch_phases)
    goals=np.asarray(rows,dtype=int);validate_rows(grip,goals)
    assert np.array_equal(goals[preparation:],pitch)
    np.savez_compressed(BASE/'candidate.npz',start=grip,goals=goals,phases=np.asarray(phases,dtype='<U72'))
    review=dict(kind='taught_vertical_relocation_with_thumb_first_pitch',goals=len(goals),preparation_goals=preparation,
        taught_end_index=ends['thumb_push_to_vertical'],phase_end_indices=ends,holding_goal_ticks=taught.tolist(),
        pitch_start_ticks=start.tolist(),pitch_end_ticks=old['goals'][-1].tolist(),
        source_pitch_goals=len(old['goals']),queued_pitch_goals=len(pitch),pitch_order_review=order_review,
        candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        raw_pitch_sha256=hashlib.sha256(PITCH.read_bytes()).hexdigest(),pose_clipped=False,new_joint_limits_added=False,
        horizontal_ID12_lead_applied=False,final_fixed_grip_return_included=False,physical_link_verified=False,
        relocation_order=['index_middle_first','thumb_push_with_ring_clearance'],
        first_lift_uses_recorded_index_middle_geometry=True,ring_held_at_G_during_finger_lift=True,
        ring_clears_with_thumb_push=True)
    (BASE/'construction_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review))

if __name__=='__main__':main()
