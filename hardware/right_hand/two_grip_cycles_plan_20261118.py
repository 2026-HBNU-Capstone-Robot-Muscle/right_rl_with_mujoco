"""Two bounded observed cycles; keep torque through the intermediate G hold."""
from pathlib import Path
import hashlib,json,time
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import ROOT,scoped_bounds
from grip_rotation_return_plan_20261118 import GripRotationReturn,GATES as ONE_GATES
from measured_grip_small_cycle_plan_20261117 import validate_rows

BASE=ROOT/'right_hand_rotation_20261118/two_grip_cycles'
MANIFEST=BASE/'operator_trial_manifest.json'
PER_CYCLE_SECONDS=500.
TOTAL_SECONDS=1000.
BOUNDARY='repeat_next_cycle_check'

class CycleDeadline(Exception):pass

class TwoGripCycles:
    def __init__(self):
        self.manifest=json.loads(MANIFEST.read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Two-cycle observation disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Two-cycle source changed: '+name)
        self.one=GripRotationReturn();self.start=self.one.start.copy();self.accepted=set();self.report=None
        self.labels=[];self.gates=set();self.second_index=None;self.second_started=None
    def bind(self,report,now=time.monotonic):self.report,self.now=report,now
    def rows(self,current):
        if pose.bounds is not scoped_bounds:raise RuntimeError('Recorded target scope must be active')
        first=self.one.rows(current);labels1=list(self.one.labels);self.entry_adopted=self.one.forward.adopted
        second=self.one.rows(self.start);labels2=list(self.one.labels)
        rows=list(first);labels=['repeat1::'+p for p in labels1]
        rows.append(self.start.copy());labels.append(BOUNDARY)
        self.second_index=len(rows)
        for q,p in zip(second,labels2):
            # The boundary confirms the same G/cube placement, so a second
            # immediate start confirmation would add no new observation.
            if p in ('cycle_start_grip_check','cycle_start_taught_check'):continue
            rows.append(q.copy());labels.append('repeat2::'+p)
        self.labels=labels
        self.gates={p for p in labels if p==BOUNDARY or ('::' in p and p.split('::',1)[1] in ONE_GATES)}
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        first_seconds=len(first)*.075
        second_seconds=(len(result)-self.second_index)*.075
        if first_seconds+30>=PER_CYCLE_SECONDS or second_seconds+30>=PER_CYCLE_SECONDS:
            raise RuntimeError('A cycle does not fit the unchanged500second bound and30second reserve')
        if len(result)*.075+60>=TOTAL_SECONDS:raise RuntimeError('Two cycles do not fit the bounded1000second profile')
        return result
    def metadata(self):
        return dict(workflow='two_observed_grip_rotation_return_cycles',planned_cycles=2,completed_cycles=0,
            per_cycle_maximum_ON_seconds=PER_CYCLE_SECONDS,maximum_ON_seconds=TOTAL_SECONDS,
            adopted_current_taught_pose=self.entry_adopted,second_cycle_start_index=self.second_index,
            phase_gate_indices={p:self.labels.index(p) for p in self.gates},operator_confirmed_stages=[],
            final_fixed_grip_return_included=True,second_rotation_link_physically_verified=False)
    def check_deadline(self):
        if self.report is None or 'enabled_at' not in self.report:return
        began=self.report['enabled_at'] if self.second_started is None else self.second_started
        if self.now()-began>=PER_CYCLE_SECONDS:
            cycle=1 if self.second_started is None else 2
            self.report['cycle_deadline_reached']=cycle
            raise CycleDeadline(f'Cycle{cycle} reached the unchanged500second bound')
    def phase(self,index,moving):
        self.check_deadline()
        if not moving:
            if self.report is not None:self.report['completed_cycles']=2
            return 'repeat_final_grip_hold'
        return self.labels[index]
    def ready(self,index):
        self.check_deadline();phase=self.labels[index]
        if phase==BOUNDARY and self.report is not None:self.report['completed_cycles']=1
        if index==self.second_index and self.second_started is None:
            if BOUNDARY not in self.accepted:raise RuntimeError('Second cycle requested without G/cube confirmation')
            # Called only after the shared runner has taken a new complete
            # health sample. No OFF, goal jump, error reset or guard bypass.
            self.second_started=self.now()
            if self.report is not None:self.report['second_cycle_started_at']=self.second_started
        return phase not in self.gates or phase in self.accepted
    def period(self,index,moving):return .075
    def message(self,phase):
        if phase==BOUNDARY:
            return '1回目'.replace('1回目','첫 회전과 파지 복귀 완료. 토크 ON 유지 중입니다. 큐브를 빼거나 재배치하지 말고, 잘 잡혀 있으면 SPACE로 두 번째 회전을 시작하세요. 문제 있으면 받친 뒤 Q로 종료하세요.')
        if phase=='repeat_final_grip_hold':
            return '두 번째 회전과 확정 파지 복귀까지 완료했습니다. 토크 ON 유지 중입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.'
        if '::' in phase:
            cycle,original=phase.split('::',1)
            return ('첫 번째: ' if cycle=='repeat1' else '두 번째: ')+self.one.message(original)
        return phase

class Terminal(pose.Terminal):
    def __init__(self,sequence,report):self.sequence,self.report=sequence,report
    def handle(self,key):
        if key in ('q','Q','\x1b','\x03'):raise pose.OperatorStop()
        if key==' ':
            phase=self.report.get('phase')
            if phase not in self.sequence.gates:raise pose.OperatorStop()
            if phase not in self.sequence.accepted:
                self.sequence.accepted.add(phase);self.report.setdefault('operator_confirmed_stages',[]).append(phase)
    def check(self):
        import msvcrt
        while msvcrt.kbhit():self.handle(msvcrt.getwch())
