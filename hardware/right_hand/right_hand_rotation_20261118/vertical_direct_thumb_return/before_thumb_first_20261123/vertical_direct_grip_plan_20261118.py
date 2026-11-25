"""Checked direct thumb return using the two user-recorded thumb waypoints."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_vertical_direct_grip_20261118 import ROOT,BASE,REFS,grip_entry
from vertical_transfer_plan_20261118 import VerticalTransfer

AFTER={'direct_return_thumb_push_toward_ring':'direct_cube_position_check',
       'direct_return_thumb_clear_cube':'direct_clearance_check',
       'direct_return_lower_support_close':'direct_support_check'}
GATES=('direct_start_grip_check','direct_rotation_check',*AFTER.values())
RUN_LIMIT_SECONDS=float('inf')

class VerticalDirectGrip:
    def __init__(self):
        self.manifest=json.loads((BASE/'operator_trial_manifest.json').read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Direct return trial disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Direct return source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Direct candidate changed')
        z=np.load(data,allow_pickle=False);self.start=z['start'];self.goals=z['goals'];self.phases=z['phases']
        self.forward=VerticalTransfer(force_grip=True);self.join=self.manifest['forward_goals']
        if (not np.array_equal(self.start,pose.reference()[2]) or not np.array_equal(self.goals[:self.join],self.forward.goals)
            or not np.array_equal(self.goals[-1],self.start)):raise RuntimeError('Direct forward or final G changed')
        for name,phase in zip(REFS,['direct_return_thumb_push_toward_ring','direct_return_thumb_clear_cube']):
            ref=json.loads((BASE/name).read_bytes());q=self.goals[self.phases==phase][-1]
            if not np.array_equal(q[:4],ref['holding_goal_ticks']):raise RuntimeError('Taught thumb endpoint changed')
        validate_rows(self.start,self.goals);self.labels=[];self.accepted=set()
    def rows(self,current):
        intro,labels=grip_entry(current,self.start);rows=[*intro,*np.tile(self.start,(40,1)),self.start.copy()]
        labels=labels+['direct_entry_settle']*40+['direct_start_grip_check']
        for i,(q,p) in enumerate(zip(self.goals,self.phases)):
            phase=str(p);rows.append(q.copy());labels.append(phase)
            gate='direct_rotation_check' if i+1==self.join else AFTER.get(phase)
            if gate and (i+1==len(self.goals) or self.phases[i+1]!=p):rows.append(q.copy());labels.append(gate)
        result=np.asarray(rows,dtype=int);validate_rows(current,result);self.labels=labels;return result
    def metadata(self):
        return dict(workflow='vertical_turn_direct_thumb_return_G',phase_gate_indices={p:self.labels.index(p) for p in GATES},
            operator_confirmed_stages=[],final_fixed_grip_return_included=True,fixed_grip_target_ticks=self.start.tolist(),
            old_palm_transfer_removed=True,maximum_ON_seconds=None,elapsed_time_limit_enabled=False,
            physical_direct_return_verified=False,automatic_repetition_included=False)
    def phase(self,index,moving):return self.labels[index] if moving else 'direct_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        messages={
            'direct_entry_range_return':'시작값의 범위 초과분만 안쪽으로 복귀합니다. 제한을 넓히지 않습니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_tips':'엄지는 유지하고 손가락 끝마디부터 파지 모양으로 준비합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_support':'엄지를 유지하며 나머지 손가락을 확정 파지로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_thumb':'다른 손가락이 파지 모양인 상태에서 엄지를 확정 G로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_settle':'확정 G에서 잠시 정착합니다.',
            'direct_start_grip_check':'확정 파지와 큐브 지지를 확인하고 손을 뗀 뒤 SPACE로 세로 회전. Q/Esc: 중단/OFF.',
            'direct_rotation_check':'세로 회전 끝. 큐브가 손에 받쳐져 있으면 SPACE로 새 직접 복귀. 재배치하지 말고 불일치면 받친 뒤 Q.',
            'direct_return_lower_index_middle_receiving':'엄지·약지는 유지하고 검지·중지를 기록된 낮은 받침 자세로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_thumb_push_toward_ring':'다른 손가락을 유지하고 새 엄지 밀기 자세로 큐브를 약지 쪽에 보냅니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_cube_position_check':'큐브가 최종 파지 자리에 왔으면 SPACE로 엄지 후퇴. 안 왔거나 끼였으면 받친 뒤 Q.',
            'direct_return_thumb_clear_cube':'새로 기록한 엄지 후퇴 자세로 빠집니다. 다른 손가락 목표는 유지합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_clearance_check':'엄지가 큐브에서 비켰고 큐브가 받쳐져 있으면 SPACE로 끝마디부터 파지 준비. 불일치면 받친 뒤 Q.',
            'direct_return_finger_tips_first':'엄지를 비켜둔 채 검지·중지·약지 끝마디부터 파지 모양으로 준비합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_lower_support_close':'엄지는 유지하고 다른 마디들을 기존 확정 G로 닫습니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_support_check':'아래쪽 파지 자리에 큐브가 받쳐져 있으면 SPACE로 엄지까지 마무리합니다. 끼임/자리 불일치면 받친 뒤 Q.',
            'direct_return_thumb_finish_G':'기존 확정 G 엄지 자세로 마무리합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_fixed_grip_hold':'확정 G로 유지 중입니다. 시간 자동 종료는 없습니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }
        if phase in messages:return messages[phase]
        if phase.startswith('direct_forward_'):return self.forward.message('vertical_'+phase[len('direct_forward_'):])
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
