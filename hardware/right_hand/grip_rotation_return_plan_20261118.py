"""Keep torque through observed forward motion, checked join, and G return."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import ROOT,scoped_bounds
from retaught_transfer_plan_20261118 import RetaughtTransfer,GATES as FORWARD_GATES
from measured_grip_small_cycle_plan_20261117 import validate_rows

BASE=ROOT/'right_hand_rotation_20261118/grip_rotation_return'
MANIFEST=BASE/'operator_trial_manifest.json'
GATES=('cycle_start_grip_check','cycle_start_taught_check','cycle_rotation_check')

class GripRotationReturn:
    def __init__(self):
        self.manifest=json.loads(MANIFEST.read_bytes());self.forward=RetaughtTransfer()
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Connected return observation disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Connected return source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Connected return candidate changed')
        z=np.load(data,allow_pickle=False);self.start=pose.reference()[2];self.goals=z['goals'];self.phases=z['phases']
        self.join=self.manifest['forward_goals'];self.accepted=set();self.labels=[]
        old=np.load(ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/tip_first_lower_seat/candidate.npz',allow_pickle=False)
        if (not np.array_equal(z['start'],self.start) or not np.array_equal(self.goals[:self.join],self.forward.goals)
            or not np.array_equal(self.goals[-1],self.start)):raise RuntimeError('Connected recorded paths changed')
        returned=self.goals[self.join:];labels=self.phases[self.join:]
        lead_mask=labels=='return_middle_root_lead';without_lead=returned[~lead_mask];original=old['goals'][1417:]
        others=[i for i in range(16) if i!=11];lead=self.manifest['root_lead'];seat=lead['before_source_return_index']
        expected=original[:,11].copy();expected[seat:]=np.maximum(expected[seat:],lead['to_ticks'])
        if (len(returned)!=self.manifest['return_goals'] or int(np.sum(lead_mask))!=lead['extra_goal_count']
            or not np.array_equal(without_lead[:,others],original[:,others])
            or not np.array_equal(without_lead[:,11],expected)
            or not np.all(returned[lead_mask][:,others]==original[seat-1,others])
            or returned[lead_mask][-1,11]!=lead['to_ticks']):raise RuntimeError('ID12-only early lift contract changed')
    def rows(self,current):
        if pose.bounds is not scoped_bounds:raise RuntimeError('Recorded target scope must be active')
        forward=self.forward.rows(current);rows=[];labels=[];entry_seen=False
        for q,label in zip(forward,self.forward.labels):
            if label in FORWARD_GATES:
                if entry_seen:continue
                entry_seen=True
                label='cycle_start_taught_check' if self.forward.adopted else 'cycle_start_grip_check'
            rows.append(q.copy());labels.append(label)
        rows.append(self.goals[self.join-1].copy());labels.append('cycle_rotation_check')
        for q,phase in zip(self.goals[self.join:],self.phases[self.join:]):
            rows.append(q.copy());labels.append('cycle_'+str(phase))
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        # Preserve the global ON deadline, including all operator waits. Leave
        # 30 seconds beyond nominal motion for the two confirmation points.
        if len(result)*.075+30>=pose.MAX_ON_SECONDS:raise RuntimeError('Movement and30second reserve exceed existing500second ON budget')
        self.labels=labels;return result
    def metadata(self):
        return dict(workflow='fixed_grip_turn_checked_return_fixed_grip',adopted_current_taught_pose=self.forward.adopted,
            phase_gate_indices={g:self.labels.index(g) for g in GATES if g in self.labels},operator_confirmed_stages=[],
            final_fixed_grip_return_included=True,fixed_grip_target_ticks=self.start.tolist(),
            preserved_forward_goals=self.join,preserved_return_goals=len(self.goals)-self.join,
            middle_root_early_lift=self.manifest['root_lead'],return_other_15_joints_unchanged=True,
            automatic_repetition_included=False,physical_join_verified=False)
    def phase(self,index,moving):return self.labels[index] if moving else 'cycle_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        messages={
            'cycle_start_grip_check':'확정 파지 확인. 손을 떼고 SPACE로 이송·가로 회전을 이어서 시작합니다. Q/Esc: 종료.',
            'cycle_start_taught_check':'새 이송 자세와 큐브 자리 확인. 손을 떼고 SPACE로 가로 회전을 이어서 시작합니다. Q/Esc: 종료.',
            'cycle_rotation_check':'회전 끝. 토크를 유지합니다. 큐브가 손에 남아 복귀를 시작할 수 있으면 SPACE. 문제 있으면 큐브를 받치고 Q로 종료하세요.',
            'cycle_return_transfer_middle_clear':'복귀: 중지를 빼며 엄지 목표는 유지합니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_transfer_thumb_prepare':'복귀: 엄지를 밀기 시작 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_transfer_thumb_push':'복귀: 엄지로 큐브를 옮기는 구간입니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_transfer_thumb_wrist_and_tips':'복귀: 엄지 후퇴와 검지·중지 끝마디 준비를 함께 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_grip_index_middle_seat':'복귀: 검지·중지를 중간 받침까지 접습니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_middle_root_lead':'복귀: 중지 맨 아랫마디 ID12를32틱 먼저 올립니다. 나머지 관절 목표는 유지합니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_grip_lower_push':'복귀: 검지·중지로 아래를 받쳐 밀어 올립니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_fixed_grip_close':'복귀: 확정 파지로 닫습니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_return_fixed_grip_hold':'확정 파지 도착. 약10초 유지한 뒤 같은 목표를 계속 유지합니다. SPACE/Q/Esc: 중단/OFF.',
            'cycle_fixed_grip_hold':'회전 뒤 확정 파지로 복귀했습니다. 토크 유지 중입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }
        return messages.get(phase,self.forward.message(phase))

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
