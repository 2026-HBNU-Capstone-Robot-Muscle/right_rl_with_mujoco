"""Offline construction: fixed grip, staged handover, preserved full turn."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_taught_palm_transfer_20261118 import interpolate

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261118/grip_to_rotation'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
TRANSFER=ROOT/'receiving_waypoint_readonly_20261118_085240_074537/result.json'

def main():
    z=np.load(FIRST,allow_pickle=False);grip=pose.reference()[2]
    offset=int(np.flatnonzero(z['phases']=='stage_7_horizontal_yaw')[0]);ready=z['goals'][offset-1].copy()
    ref=json.loads((ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/thumb_wrist_clear_reference.json').read_bytes())
    if hashlib.sha256((ROOT/ref['source']).read_bytes()).hexdigest()!=ref['source_sha256']:raise RuntimeError('Thumb-clear source changed')
    measured=json.loads(TRANSFER.read_bytes())
    if measured['status']!='readonly_observation_complete' or measured['max_two_read_difference_ticks']!=0:
        raise RuntimeError('Stable measured transfer reference missing')
    s=measured['snapshots'][-1]
    if s['torque'][4:12]!=[1]*8 or any(s['error']):raise RuntimeError('Measured index/middle state changed')
    # Only the actively held index/middle geometry is imported. Thumb/ring
    # were OFF during capture; their complete pose is not a loaded target.
    clear=grip.copy();clear[:4]=ref['actual_ticks'][:4]
    transfer=clear.copy();transfer[4:12]=s['position'][4:12]
    _,high,low=pose.bounds();adjustments=[]
    for i in np.flatnonzero((transfer<low)|(transfer>high)):
        before=int(transfer[i]);transfer[i]=int(np.clip(transfer[i],np.ceil(low[i])+4,high[i]))
        adjustments.append(dict(id=int(i+1),measured=before,proposed=int(transfer[i])))
    # OFF ring angles are a user-observed geometric reference, not a measured
    # loaded control target. Introduce their bounded proposal as a distinct
    # slow, operator-gated push, holding the index/middle platform fixed.
    pushed=transfer.copy();pushed[12:16]=s['position'][12:16]
    ring_adjustments=[]
    for i in range(12,16):
        if pushed[i]<low[i] or pushed[i]>high[i]:
            before=int(pushed[i]);pushed[i]=int(np.clip(pushed[i],np.ceil(low[i])+4,high[i]))
            ring_adjustments.append(dict(id=i+1,measured=before,proposed=int(pushed[i])))
    index_open=pushed.copy();index_open[4:8]=ready[4:8]
    caught=index_open.copy();caught[:4]=ready[:4]
    stages=[('thumb_clear_for_turn',clear),('measured_thumbward_transfer',transfer),
            ('ring_push_thumbward',pushed),('release_index',index_open),
            ('thumb_to_turn_catch',caught),('middle_ring_to_turn_ready',ready)]
    rows=[];phases=[];previous=grip.copy();ends={}
    for phase,target in stages:
        part=interpolate(previous,target);rows.extend(part);phases.extend([phase]*len(part))
        ends[phase]=len(rows)-1;previous=target.copy()
    preparation=len(rows);rows.extend(z['goals'][offset:]);phases.extend(['preserved_'+str(p) for p in z['phases'][offset:]])
    goals=np.asarray(rows,dtype=int);validate_rows(grip,goals)
    assert np.array_equal(goals[preparation:],z['goals'][offset:])
    OUT.mkdir(exist_ok=True)
    np.savez_compressed(OUT/'candidate.npz',start=grip,goals=goals,phases=np.array(phases,dtype='<U64'))
    review=dict(kind='fixed_physical_grip_to_preserved_full_turn',goals=len(goals),
        preparation_goals=preparation,preserved_turn_source_offset=offset,
        preserved_turn_goals=len(z['goals'])-offset,preserved_turn_exact=True,
        phase_end_indices=ends,grip_ticks=grip.tolist(),turn_ready_ticks=ready.tolist(),
        endpoint_ticks=goals[-1].tolist(),source_sha256=hashlib.sha256(FIRST.read_bytes()).hexdigest(),
        measured_transfer_source=TRANSFER.relative_to(ROOT).as_posix(),
        measured_transfer_sha256=hashlib.sha256(TRANSFER.read_bytes()).hexdigest(),
        imported_measured_motor_ids=list(range(5,13)),inward_target_adjustments=adjustments,
        transfer_proposal_ticks=transfer.tolist(),cube_translation_direction='toward_thumb',
        ring_push_proposal_ticks=pushed.tolist(),ring_push_inward_adjustments=ring_adjustments,
        ring_source_was_torque_OFF=True,ring_push_physically_verified=False,
        ring_target_held_until_thumb_catch=True,middle_ring_coordinated_after_thumb_catch=True,
        cube_translation_physically_verified=False,
        candidate_sha256=hashlib.sha256((OUT/'candidate.npz').read_bytes()).hexdigest(),
        original_grip_reference_unchanged=True,new_joint_limits_added=False,
        hardware_motion_by_assistant=False,grip_to_turn_transfer_verified=False,
        physical_cube_angle_verified=False,fixed_grip_return_included=False)
    (OUT/'construction_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review))

if __name__=='__main__':main()
