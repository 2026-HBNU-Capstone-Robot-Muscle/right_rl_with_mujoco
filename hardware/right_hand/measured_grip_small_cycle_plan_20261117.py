"""Source-bound diagnostic: fixed grip, two small model-derived turns, hold."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
DATA=BASE/'two_small_turns_model_probe/research_goals.npz'
REVIEW=BASE/'two_small_turns_model_probe/review.json'
MANIFEST=BASE/'small_cycle_manifest.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_rows(start,rows):
    anchor=pose.encoder_vector(start)
    _,high,low=pose.bounds()
    whole=np.vstack([anchor,rows])
    if not np.isfinite(whole).all() or np.any(whole!=np.rint(whole)) or np.any((whole<0)|(whole>4095)):
        raise RuntimeError('Invalid sequence encoder targets')
    if np.max(abs(np.diff(whole,axis=0)))>2:
        raise RuntimeError('Small-cycle step exceeds2ticks')
    excess=np.maximum(np.maximum(low-whole,whole-high),0.)
    if (np.any(np.diff(excess,axis=0)>1e-9) or np.any(rows<np.minimum(np.ceil(low),anchor))
        or np.any(rows>np.maximum(high,anchor))):
        raise RuntimeError('Small-cycle range/return envelope mismatch')


def load_trial():
    manifest=json.loads(MANIFEST.read_bytes())
    for name,digest in manifest['source_hashes'].items():
        if sha(ROOT/name)!=digest:raise RuntimeError('Small-cycle source changed: '+name)
    if sha(DATA)!=manifest['trajectory_sha256'] or sha(REVIEW)!=manifest['model_review_sha256']:
        raise RuntimeError('Small-cycle trajectory/review hash mismatch')
    review=json.loads(REVIEW.read_bytes())
    if not review['model_two_small_turns_passed'] or review['cube_state_reset_during_path'] or review['external_cube_forces']:
        raise RuntimeError('Continuous small-cycle model evidence missing')
    z=np.load(DATA,allow_pickle=False)
    goals=np.asarray(z['goals']);phases=np.asarray(z['phases']);start=np.asarray(z['start_goal'])
    grip=pose.reference()[2]
    if not np.array_equal(start,grip) or goals.shape!=(146,16) or phases.shape!=(146,):
        raise RuntimeError('Small-cycle start/shape mismatch')
    if list(dict.fromkeys(phases.tolist()))!=['turn1','hold1','turn2','hold2']:
        raise RuntimeError('Unexpected small-cycle phases')
    if not np.isfinite(goals).all() or np.any(goals!=np.rint(goals)):
        raise RuntimeError('Small-cycle goals must be finite integer ticks')
    goals=goals.astype(int)
    validate_rows(grip,goals)
    if np.array_equal(goals[-1],grip):raise RuntimeError('Expected a diagnostic turn, not a grip-only path')
    return goals,phases,manifest


class SmallCycle:
    def __init__(self):
        self.goals,self.phases,self.manifest=load_trial()
        self.grip=pose.reference()[2];self.labels=[]
    def rows(self,start):
        prepare=pose.path(start,self.grip)
        result=np.vstack([prepare,np.tile(self.grip,(67,1)),self.goals])
        self.labels=['small_cycle_prepare']*len(prepare)+['small_cycle_reference_hold']*67+['small_cycle_'+p for p in self.phases]
        if len(result)*pose.PERIOD+10>=pose.MAX_ON_SECONDS:
            raise RuntimeError('Small-cycle preparation exceeds existing500second budget')
        validate_rows(start,result)
        return result
    def phase(self,index,moving):
        return self.labels[index] if moving else 'small_cycle_hold'
    def period(self,index,moving):
        return pose.PERIOD if (not moving or 'hold' in self.labels[index]) else pose.MOVE_PERIOD
    def message(self,phase):
        return {'small_cycle_prepare':'현재 자세에서 확정 파지로 이동합니다.',
            'small_cycle_reference_hold':'확정 파지에서5초 유지한 뒤 작은 회전을 시작합니다.',
            'small_cycle_turn1':'첫 소각도 회전 목표로 이동합니다.',
            'small_cycle_hold1':'첫 소각도 회전 뒤 유지합니다.',
            'small_cycle_turn2':'다음 소각도 회전 목표로 이동합니다.',
            'small_cycle_hold2':'두 번째 소각도 회전 뒤 유지합니다.',
            'small_cycle_hold':'소각도 관찰을 마쳤습니다. 현재 자세를 계속 유지합니다.'}[phase]
