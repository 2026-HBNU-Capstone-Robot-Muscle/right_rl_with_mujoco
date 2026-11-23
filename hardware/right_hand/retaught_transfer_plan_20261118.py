"""New all-finger taught transfer; adopt that pose without returning through G."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import BASE,ROOT,reference,scoped_bounds
from measured_grip_small_cycle_plan_20261117 import validate_rows

MANIFEST=BASE/'operator_trial_manifest.json'
AFTER={'taught_clear_transfer':'retaught_transfer_check','release_index':'retaught_index_check',
       'thumb_catch':'retaught_catch_check','middle_ring_handover':'retaught_ready_check'}
GATES=('retaught_grip_check',*AFTER.values())

class RetaughtTransfer:
    def __init__(self,transfer_only=False):
        self.transfer_only=transfer_only;self.accepted=set();self.labels=[]
        self.manifest=json.loads(MANIFEST.read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Retaught observation disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Retaught source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Retaught candidate changed')
        z=np.load(data,allow_pickle=False);self.start=pose.reference()[2];self.taught=reference()[1]
        self.goals=z['goals'];self.phases=z['phases'];self.preparation=self.manifest['preparation_goals']
        self.taught_end=self.manifest['taught_end_index']
        if not np.array_equal(z['start'],self.start) or len(self.goals)!=self.manifest['loaded_goal_count']:raise RuntimeError('Retaught start/count changed')
        if not np.array_equal(self.goals[self.taught_end],self.taught):raise RuntimeError('Recorded pose endpoint changed')
        old=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)
        if not np.array_equal(self.goals[self.preparation:],old['goals'][645:]):raise RuntimeError('Existing full turn changed')
    def rows(self,current):
        if pose.bounds is not scoped_bounds:raise RuntimeError('Recorded target scope must be active')
        self.adopted=bool(np.max(abs(np.asarray(current)-self.taught))<=4)
        target=self.taught if self.adopted else self.start
        intro=pose.path(current,target)
        rows=[*intro,*np.tile(target,(40,1))]
        labels=['retaught_adopt' if self.adopted else 'retaught_prepare']*len(intro)+['retaught_settle']*40
        if not (self.adopted and self.transfer_only):
            rows.append(target.copy());labels.append('retaught_transfer_check' if self.adopted else 'retaught_grip_check')
        begin=self.taught_end+1 if self.adopted else 0
        end=self.taught_end+1 if self.transfer_only else len(self.goals)
        for i in range(begin,end):
            phase=str(self.phases[i]);rows.append(self.goals[i].copy());labels.append('retaught_'+phase)
            if phase in AFTER and (i+1==end or self.phases[i+1]!=phase):
                if not (self.transfer_only and i+1==end):rows.append(self.goals[i].copy());labels.append(AFTER[phase])
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        if len(result)*.075+10>=pose.MAX_ON_SECONDS:raise RuntimeError('Existing500second movement budget exceeded')
        self.labels=labels;return result
    def metadata(self):
        return dict(adopted_current_taught_pose=self.adopted,retaught_holding_goal_ticks=self.taught.tolist(),
            phase_gate_indices={g:self.labels.index(g) for g in GATES if g in self.labels},operator_confirmed_stages=[],
            preserved_turn_goals=0 if self.transfer_only else 772,transfer_observation_only=self.transfer_only,
            final_fixed_grip_return_included=False)
    def phase(self,index,moving):return self.labels[index] if moving else ('retaught_transfer_hold' if self.transfer_only else 'retaught_rotation_hold')
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        return {
            'retaught_adopt':'지금 기록한 자세를 그대로 이어받습니다. 확정 파지로 되돌리지 않습니다.',
            'retaught_prepare':'현재 자세에서 확정 파지로 이동합니다. SPACE/Q/Esc: 중단/OFF.',
            'retaught_settle':'현재 목표에서 잠시 유지합니다.',
            'retaught_grip_check':'확정 파지 확인. 큐브가 잡혔으면 SPACE로 새 이송 자세에 함께 이동합니다.',
            'retaught_taught_clear_transfer':'16개 관절을 새로 기록한 엄지 후퇴·큐브 이송 자세로 함께 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'retaught_transfer_check':'새 이송 자세 확인. 큐브가 받쳐져 있고 원하는 자리로 옮겨졌으면 SPACE. 불일치/끼임이면 받친 뒤 Q로 종료하세요.',
            'retaught_release_index':'엄지·중지·약지 목표를 유지하고 검지를 회전 준비 위치로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'retaught_index_check':'검지 준비 끝. 큐브가 받쳐져 있으면 SPACE로 엄지가 받는 구간에 진입합니다.',
            'retaught_thumb_catch':'다른 손가락 목표를 유지하고 엄지를 기존 회전 시작 쪽으로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'retaught_catch_check':'엄지가 큐브를 받아 지지하면 SPACE로 중지·약지 인계. 못 받았으면 받친 뒤 Q로 종료하세요.',
            'retaught_middle_ring_handover':'엄지·검지 목표를 유지하고 중지·약지를 함께 기존 회전 준비로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'retaught_ready_check':'엄지·중지 사이 회전 시작 파지와 큐브 위치를 확인한 뒤 SPACE로 기존 가로 회전. 불일치/끼임이면 받치고 Q로 종료하세요.',
            'retaught_transfer_hold':'새로 기록한 이송 자세를 계속 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
            'retaught_rotation_hold':'회전 끝 자세를 계속 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }.get(phase,'기존 가로 회전 구간입니다. SPACE/Q/Esc: 중단/OFF.')

class Terminal(pose.Terminal):
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
