"""Operator-gated full recorded quarter-turn, real receiving geometry, fixed grip."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
DATA=BASE/'tip_first_lower_seat/candidate.npz'
MANIFEST=BASE/'operator_trial_manifest.json'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
GATE_AFTER={'transfer_middle_clear':'quarter_middle_clear_check',
            'transfer_thumb_push':'quarter_push_check',
            'transfer_thumb_wrist_and_tips':'quarter_receiving_check',
            'grip_lower_push':'quarter_lower_push_check'}
GATES=('quarter_place','quarter_rotation_check',*GATE_AFTER.values())


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class QuarterFixedGrip:
    def __init__(self):
        self.manifest=json.loads(MANIFEST.read_bytes())
        if not self.manifest.get('operator_gated_observation_enabled'):
            raise RuntimeError('Staged operator observation is not enabled')
        for name,digest in self.manifest['source_hashes'].items():
            if sha(ROOT/name)!=digest:raise RuntimeError('Quarter-grip source changed: '+name)
        if sha(DATA)!=self.manifest['candidate_sha256']:raise RuntimeError('Quarter-grip candidate changed')
        z=np.load(DATA,allow_pickle=False);first=np.load(FIRST,allow_pickle=False)
        self.start=z['start'].astype(int);self.goals=z['goals'].astype(int);self.phases=z['phases'].astype(str)
        self.grip=pose.reference()[2];self.accepted=set();self.labels=[]
        if (self.goals.shape!=(self.manifest['loaded_goal_count'],16) or self.phases.shape!=(len(self.goals),)
            or not np.array_equal(self.start,first['start'])
            or not np.array_equal(self.goals[:1417],first['goals'])
            or not np.array_equal(self.phases[:1417],first['phases'])
            or not np.array_equal(self.goals[-1],self.grip)):
            raise RuntimeError('Preserved quarter-turn/fixed-grip contract mismatch')
        validate_rows(self.start,self.goals)
        if any(not np.any(self.phases==phase) for phase in GATE_AFTER):
            raise RuntimeError('Taught transfer stages missing')
    def rows(self,current):
        empty=pose.path(current,self.start)
        rows=[*empty,self.start.copy()];labels=['quarter_empty_move']*len(empty)+['quarter_place']
        for i,(goal,phase) in enumerate(zip(self.goals,self.phases)):
            rows.append(goal.copy());labels.append('quarter_fixed_grip_settle' if phase=='fixed_grip_hold' else 'quarter_'+str(phase))
            if i==1416:
                rows.append(goal.copy());labels.append('quarter_rotation_check')
            if phase in GATE_AFTER and (i+1==len(self.goals) or self.phases[i+1]!=phase):
                rows.append(goal.copy());labels.append(GATE_AFTER[phase])
        self.labels=labels;result=np.array(rows,dtype=int)
        validate_rows(current,result)
        if len(result)*.075+10>=pose.MAX_ON_SECONDS:
            raise RuntimeError('Movement alone exceeds existing500second budget')
        return result
    def metadata(self):
        return dict(phase_gate_indices={phase:self.labels.index(phase) for phase in GATES},
                    preserved_turn_goal_count=1417,source_goal_count=len(self.goals),
                    fixed_grip_target_ticks=self.grip.tolist(),operator_confirmed_stages=[])
    def phase(self,index,moving):return self.labels[index] if moving else 'quarter_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):
        phase=self.labels[index]
        return phase not in GATES or phase in self.accepted
    def accept(self,phase,report):
        if phase not in GATES:raise RuntimeError('SPACE acceptance outside a waiting gate')
        if phase not in self.accepted:
            self.accepted.add(phase);report.setdefault('operator_confirmed_stages',[]).append(phase)
    def message(self,phase):
        messages={
            'quarter_empty_move':'빈손으로 기존 큐브 배치 자세에 이동합니다. 아직 큐브를 놓지 마세요.',
            'quarter_place':'PLACE: 예전90도 시험과 같은 자리에 큐브를 놓으세요. 손을 뗀 뒤 SPACE로 회전 시작. Q/Esc: 중단.',
            'quarter_rotation_check':'회전 구간 끝. 큐브가 손에 남아 있으면 SPACE로 중지만 먼저 뺍니다. Q/Esc: 중단.',
            'quarter_middle_clear_check':'중지 후퇴 끝. 큐브가 받쳐져 있고 중지가 비켰으면 SPACE로 엄지 준비와 밀기. Q/Esc: 중단.',
            'quarter_push_check':'엄지 밀기 끝. 큐브가 옮겨졌으면 SPACE로 엄지 후퇴와 검지·중지 끝마디 준비. Q/Esc: 중단.',
            'quarter_receiving_check':'엄지 후퇴와 끝마디 준비 끝. 큐브가 받쳐져 있으면 SPACE로 검지·중지 하부 받침과 밀기. Q/Esc: 중단.',
            'quarter_lower_push_check':'검지·중지 하부 밀기 끝. 큐브가 파지 자리에 옮겨졌으면 SPACE로 확정 파지 마무리. Q/Esc: 중단.',
            'quarter_transfer_middle_clear':'엄지는 유지하고 중지만 기록한 후퇴 자세로 이동합니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_transfer_thumb_prepare':'중지는 뺀 상태로 두고 엄지를 기록한 밀기 시작 자세로 이동합니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_transfer_thumb_push':'중지와 나머지 손가락은 유지하고 엄지로 조금 미는 구간입니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_transfer_thumb_wrist_and_tips':'엄지를 손목 쪽으로 빼면서 검지·중지 끝마디를 먼저 준비합니다. 나머지 마디와 약지는 유지합니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_grip_index_middle_seat':'끝마디 모양을 유지하고 검지·중지의 다음 마디를 중간 받침까지 접습니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_grip_lower_push':'끝마디 모양은 유지하고 검지·중지의 다음 마디와 아랫마디를 함께 움직여 아래를 받쳐 밉니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_fixed_grip_close':'확정한 최종 파지 목표로 닫습니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_fixed_grip_settle':'확정 파지 목표 도착. 약10초 유지 상태를 기록합니다. SPACE/Q/Esc: 중단 및OFF.',
            'quarter_fixed_grip_hold':'확정 파지 목표 유지 중. 큐브를 받친 뒤 SPACE로 종료/OFF.',
            }
        return messages.get(phase,'기존 회전 구간: '+phase+' / SPACE/Q/Esc: 중단 및OFF.')


class QuarterTerminal(pose.Terminal):
    def __init__(self,sequence,report):self.sequence,self.report=sequence,report
    def handle(self,key):
        if key in ('q','Q','\x1b','\x03'):raise pose.OperatorStop()
        if key==' ':
            phase=self.report.get('phase')
            if phase in GATES:self.sequence.accept(phase,self.report)
            else:raise pose.OperatorStop()
    def check(self):
        import msvcrt
        while msvcrt.kbhit():self.handle(msvcrt.getwch())
