"""One live controller for G -> vertical turn -> checked return -> the same G."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_vertical_rotation_return_20261118 import ROOT,BASE
from vertical_transfer_plan_20261118 import VerticalTransfer
from vertical_return_plan_20261118 import VerticalReturn

GATES=('vertical_cycle_grip_check','vertical_cycle_rotation_check')

class VerticalRotationReturn:
    def __init__(self):
        self.manifest=json.loads((BASE/'operator_trial_manifest.json').read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Vertical combined observation disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Vertical combined source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:
            raise RuntimeError('Vertical combined candidate changed')
        self.forward=VerticalTransfer(force_grip=True);self.returned=VerticalReturn()
        z=np.load(data,allow_pickle=False);self.start=z['start'];self.goals=z['goals'];self.phases=z['phases']
        self.join=self.manifest['forward_goals'];n=self.manifest['return_goals']
        if (not np.array_equal(self.start,pose.reference()[2])
            or not np.array_equal(self.goals[:self.join],self.forward.goals)
            or not np.array_equal(self.goals[self.join:],self.returned.goals[:n])
            or not np.array_equal(self.goals[self.join-1],self.returned.start)
            or not np.array_equal(self.goals[-1],self.start)):
            raise RuntimeError('Vertical combined motion source contract changed')
        validate_rows(self.start,self.goals);self.accepted=set();self.labels=[]
    def rows(self,current):
        intro=pose.path(current,self.start)
        rows=[*intro,*np.tile(self.start,(40,1)),self.start.copy()]
        labels=['vertical_cycle_prepare_grip']*len(intro)+['vertical_cycle_settle']*40+[GATES[0]]
        for i,(q,phase) in enumerate(zip(self.goals,self.phases)):
            rows.append(q.copy());labels.append(str(phase))
            if i+1==self.join:rows.append(q.copy());labels.append(GATES[1])
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        if len(result)*.075+30>=pose.MAX_ON_SECONDS:
            raise RuntimeError('500second budget requires starting near fixed G; prepare G first, then reopen this file')
        self.labels=labels;return result
    def metadata(self):
        return dict(workflow='vertical_rotation_checked_return_same_G',
            phase_gate_indices={p:self.labels.index(p) for p in GATES},operator_confirmed_stages=[],
            final_fixed_grip_return_included=True,fixed_grip_target_ticks=self.start.tolist(),
            preserved_forward_goals=self.join,preserved_return_goals=len(self.goals)-self.join,
            join_endpoint_mismatch_ticks=0,join_step_ticks=self.manifest['join_step_ticks'],
            middle_support_held_during_return_thumb_work=True,
            automatic_repetition_included=False,physical_combined_execution_verified=False,
            single_controller_and_deadline=True)
    def phase(self,index,moving):return self.labels[index] if moving else 'vertical_cycle_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        messages={
            'vertical_cycle_prepare_grip':'확정 파지 G를 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_cycle_settle':'확정 파지 G에서 잠시 정착합니다.',
            GATES[0]:'확정 파지와 큐브 지지를 확인하고 손을 뗀 뒤 SPACE. 세로 회전까지 이어집니다. Q/Esc: 중단/OFF.',
            GATES[1]:'세로 회전 끝. 토크 유지 중입니다. 큐브가 복귀 시작 자리에 받쳐져 있으면 SPACE로 파지 복귀. 손으로 자리를 고쳐 놓지 말고 불일치면 받친 뒤 Q로 종료하세요.',
            'vertical_cycle_fixed_grip_hold':'세로 회전 뒤 확정 파지 G로 복귀해 유지 중입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }
        if phase in messages:return messages[phase]
        if phase.startswith('cycle_forward_'):return self.forward.message('vertical_'+phase[len('cycle_forward_'):])
        if phase.startswith('cycle_return_'):return self.returned.message('vreturn_'+phase[len('cycle_return_'):])
        return phase+' / SPACE/Q/Esc: 중단/OFF.'

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
