"""Operator-gated grip-to-turn observation, with the known full turn preserved."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/grip_to_rotation'
DATA=BASE/'candidate.npz';MANIFEST=BASE/'operator_trial_manifest.json'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
AFTER={'thumb_clear_for_turn':'grip_turn_clear_check',
       'measured_thumbward_transfer':'grip_turn_transfer_check',
       'ring_push_thumbward':'grip_turn_ring_push_check',
       'release_index':'grip_turn_release_check',
       'thumb_to_turn_catch':'grip_turn_catch_check',
       'middle_ring_to_turn_ready':'grip_turn_ready_check'}
GATES=('grip_turn_grasp_check',*AFTER.values())

class GripToRotation:
    def __init__(self,transfer_only=False):
        self.transfer_only=bool(transfer_only)
        self.manifest=json.loads(MANIFEST.read_bytes())
        flag='transfer_observation_enabled' if self.transfer_only else 'operator_gated_observation_enabled'
        if not self.manifest.get(flag,False):raise RuntimeError('Grip-to-turn observation not enabled for requested scope')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Grip-to-turn source changed: '+name)
        if hashlib.sha256(DATA.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Grip-to-turn candidate changed')
        z=np.load(DATA,allow_pickle=False);old=np.load(FIRST,allow_pickle=False)
        self.start=pose.reference()[2];self.goals=z['goals'].astype(int);self.phases=z['phases'].astype(str)
        if not np.array_equal(z['start'],self.start) or len(self.goals)!=self.manifest['loaded_goal_count']:raise RuntimeError('Grip start/count changed')
        offset=self.manifest['preserved_turn_source_offset'];self.preparation=self.manifest['preparation_goals']
        if not np.array_equal(self.goals[self.preparation:],old['goals'][offset:]):raise RuntimeError('Full recorded turn changed')
        validate_rows(self.start,self.goals)
        if self.transfer_only:
            end=int(np.flatnonzero(self.phases=='measured_thumbward_transfer')[-1])+1
            self.goals=self.goals[:end].copy();self.phases=self.phases[:end].copy();self.preparation=end
        self.accepted=set();self.labels=[]
    def rows(self,current):
        prepare=pose.path(current,self.start)
        rows=[*prepare,*np.tile(self.start,(40,1)),self.start.copy()]
        labels=['grip_turn_prepare']*len(prepare)+['grip_turn_settle']*40+['grip_turn_grasp_check']
        for i,(goal,phase) in enumerate(zip(self.goals,self.phases)):
            rows.append(goal.copy());labels.append('grip_turn_'+str(phase))
            if phase in AFTER and (i+1==len(self.goals) or self.phases[i+1]!=phase):
                if not (self.transfer_only and i+1==len(self.goals)):
                    rows.append(goal.copy());labels.append(AFTER[phase])
        self.labels=labels;result=np.asarray(rows,dtype=int);validate_rows(current,result)
        if len(result)*.075+10>=pose.MAX_ON_SECONDS:raise RuntimeError('Movement exceeds existing500second budget')
        return result
    def metadata(self):
        return dict(phase_gate_indices={p:self.labels.index(p) for p in GATES if p in self.labels},operator_confirmed_stages=[],
                    fixed_grip_start_ticks=self.start.tolist(),preserved_turn_goals=len(self.goals)-self.preparation,
                    transfer_observation_only=self.transfer_only,final_fixed_grip_return_included=False)
    def phase(self,index,moving):return self.labels[index] if moving else ('grip_turn_transfer_hold' if self.transfer_only else 'grip_turn_rotation_hold')
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        return {
          'grip_turn_prepare':'확정 파지 목표로 천천히 이동합니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_settle':'확정 파지 목표에서 잠시 유지합니다.',
          'grip_turn_grasp_check':'파지 확인: 큐브가 잡혀 있으면 손을 뗀 뒤 SPACE로 회전 준비 시작. Q/Esc: 중단.',
          'grip_turn_release_index':'중지·약지·엄지 목표를 유지하고 검지만 기존 회전 준비 쪽으로 엽니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_release_check':'검지 준비 끝. 중지·약지가 큐브를 받치고 있으면 SPACE로 엄지를 회전 시작 쪽으로 옮깁니다. Q/Esc: 중단.',
          'grip_turn_thumb_clear_for_turn':'검지·중지·약지를 유지하고 엄지를 기록한 손목 쪽 후퇴 자세로 뺍니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_clear_check':'엄지 후퇴 끝. 큐브가 받쳐져 있으면 SPACE로 방금 기록한 검지·중지 받침 자세를 시험합니다. Q/Esc: 중단.',
          'grip_turn_measured_thumbward_transfer':'검지·중지를 기록한 받침 자세로 옮깁니다. 큐브가 엄지 쪽으로 옮겨지는지 관찰하세요. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_transfer_check':'검지·중지 받침 도착. 큐브가 받쳐져 있고 약지가 밀 수 있는 위치면 SPACE로 약지 밀기. 끼거나 벗어나면 큐브를 받치고 Q로 종료하세요.',
          'grip_turn_ring_push_thumbward':'검지·중지·엄지 목표를 유지하고 약지만 기록한 후보 방향으로 천천히 움직입니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_ring_push_check':'약지 밀기 후보 도착. 큐브가 엄지 쪽으로 옮겨져 받쳐졌을 때만 SPACE. 그대로이거나 끼면 받치고 Q로 종료하세요.',
          'grip_turn_thumb_to_turn_catch':'검지·중지·약지 목표를 유지하고 엄지만 기존 회전 시작 쪽으로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_catch_check':'엄지가 큐브를 받아 지지하고 있을 때만 SPACE. 다음은 중지·약지를 함께 회전 준비로 인계합니다. 못 받았거나 끼면 큐브를 받치고 Q로 종료하세요.',
          'grip_turn_middle_ring_to_turn_ready':'엄지·검지 목표를 유지하고 중지·약지를 함께 기존 회전 준비 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
          'grip_turn_ready_check':'회전 시작 파지 도착. 큐브가 실제 회전 시작 자리에 받쳐져 있을 때만 SPACE로 기존90도급 회전 시작. 자리 불일치/끼임이면 큐브를 받친 뒤 Q로 종료하세요.',
          'grip_turn_rotation_hold':'회전 끝 자세를 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
          'grip_turn_transfer_hold':'기록한 받침 자세에서 유지합니다. 큐브가 엄지 쪽으로 옮겨졌는지 확인하세요. 큐브를 받친 뒤 SPACE로 종료/OFF. 이번 파일은 여기까지입니다.',
        }.get(phase,'기존 회전 구간 '+phase+' / SPACE/Q/Esc: 중단/OFF.')

class GripTurnTerminal(pose.Terminal):
    def __init__(self,sequence,report):self.sequence,self.report=sequence,report
    def handle(self,key):
        if key in ('q','Q','\x1b','\x03'):raise pose.OperatorStop()
        if key==' ':
            phase=self.report.get('phase')
            if phase not in GATES:raise pose.OperatorStop()
            if phase not in self.sequence.accepted:
                self.sequence.accepted.add(phase);self.report.setdefault('operator_confirmed_stages',[]).append(phase)
    def check(self):
        import msvcrt
        while msvcrt.kbhit():self.handle(msvcrt.getwch())
