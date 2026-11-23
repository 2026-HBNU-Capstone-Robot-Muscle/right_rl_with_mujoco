"""One user start, continuous vertical turn/return, then fixed G hold."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_vertical_direct_grip_20261118 import ROOT,BASE,REFS,FINAL_BASE,FINAL_REFS,grip_entry,final_references
from vertical_transfer_plan_20261118 import VerticalTransfer

AFTER={'direct_return_lower_index_middle_receiving':'direct_receiving_check',
       'direct_return_final_taught_1':'direct_final_waypoint_1_check',
       'direct_return_final_taught_2':'direct_final_waypoint_2_check',
       'direct_return_final_taught_to_G':'direct_support_check'}
CHECKPOINTS=('direct_start_grip_check','direct_rotation_check',*AFTER.values())
GATES=()  # No operator confirmation waits inside the sequence.
RUN_LIMIT_SECONDS=float('inf')

class VerticalDirectGrip:
    def __init__(self):
        self.manifest=json.loads((BASE/'operator_trial_manifest.json').read_bytes())
        if (not self.manifest.get('automatic_stage_transitions_enabled')
            or self.manifest.get('operator_gated_observation_enabled')):
            raise RuntimeError('Continuous direct return not prepared')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Direct return source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Direct candidate changed')
        z=np.load(data,allow_pickle=False);self.start=z['start'];self.goals=z['goals'];self.phases=z['phases']
        self.forward=VerticalTransfer(force_grip=True);self.join=self.manifest['forward_goals']
        if (not np.array_equal(self.start,pose.reference()[2]) or not np.array_equal(self.goals[:self.join],self.forward.goals)
            or not np.array_equal(self.goals[-1],self.start)):raise RuntimeError('Direct forward or final G changed')
        if np.any(np.isin(self.phases,['direct_return_thumb_push_toward_ring','direct_return_thumb_clear_cube'])):
            raise RuntimeError('Removed thumb push/withdrawal reintroduced')
        for name,phase in zip(REFS,['direct_return_thumb_clear_cube']):
            ref=json.loads((BASE/name).read_bytes());q=self.goals[self.phases==phase][-1]
            if not np.array_equal(q[:4],ref['holding_goal_ticks']):raise RuntimeError('Taught thumb endpoint changed')
        lower=self.goals[self.phases=='direct_return_thumb_lower_before_support']
        if (not np.array_equal(lower[-1,:4],self.start[:4])
            or not np.all(lower[:,4:12]==lower[0,4:12])):
            raise RuntimeError('Thumb must reach G before index/middle support motion')
        fixed=[0,1,2,3,12,13,14,15]
        for i,ref in enumerate(final_references(),1):
            rows=self.goals[self.phases==f'direct_return_final_taught_{i}']
            if (not len(rows) or not np.array_equal(rows[-1,4:12],np.array(ref['holding_goal_ticks'])[4:12])
                or not np.all(rows[:,fixed]==self.start[fixed])):
                raise RuntimeError('Final taught waypoint or fixed thumb/ring changed')
        finish=self.goals[self.phases=='direct_return_final_taught_to_G']
        if not len(finish) or not np.all(finish[:,fixed]==self.start[fixed]):
            raise RuntimeError('Thumb/ring moved during final G closing')
        validate_rows(self.start,self.goals);self.labels=[]
    def rows(self,current):
        intro,labels=grip_entry(current,self.start);rows=[*intro,*np.tile(self.start,(40,1)),self.start.copy()]
        labels=labels+['direct_entry_settle']*40+['direct_start_grip_check']
        for i,(q,p) in enumerate(zip(self.goals,self.phases)):
            phase=str(p);rows.append(q.copy());labels.append(phase)
            gate='direct_rotation_check' if i+1==self.join else AFTER.get(phase)
            if gate and (i+1==len(self.goals) or self.phases[i+1]!=p):rows.append(q.copy());labels.append(gate)
        result=np.asarray(rows,dtype=int);validate_rows(current,result);self.labels=labels;return result
    def metadata(self):
        return dict(workflow='vertical_turn_direct_thumb_return_G',phase_gate_indices={},
            phase_checkpoint_indices={p:self.labels.index(p) for p in CHECKPOINTS},
            operator_gated_observation_enabled=False,automatic_stage_transitions_enabled=True,
            operator_confirmed_stages=[],final_fixed_grip_return_included=True,fixed_grip_target_ticks=self.start.tolist(),
            old_palm_transfer_removed=True,maximum_ON_seconds=None,elapsed_time_limit_enabled=False,
            thumb_push_removed=True,
            thumb_withdrawal_removed=True,
            thumb_G_target_before_other_finger_push=True,
            final_G_via_two_taught_waypoints=True,final_waypoint_motor_ids=list(range(5,13)),
            ring_held_at_G_during_final_waypoints=True,
            physical_direct_return_verified=False,automatic_repetition_included=False)
    def phase(self,index,moving):return self.labels[index] if moving else 'direct_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return True
    def message(self,phase):
        messages={
            'direct_entry_range_return':'시작값의 범위 초과분만 안쪽으로 복귀합니다. 제한을 넓히지 않습니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_thumb_lower':'다른 손가락은 현재 목표를 유지하고 엄지를 먼저 확정 파지 위치로 내립니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_tips':'엄지를 내린 목표로 유지하고 손가락 끝마디를 파지 모양으로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_support':'엄지를 내린 채 검지·중지와 나머지 받침을 확정 파지로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_entry_settle':'확정 G에서 잠시 정착합니다.',
            'direct_start_grip_check':'확정 파지 구간 끝. 세로 회전으로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_rotation_check':'세로 회전 구간 끝. 파지 복귀로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_lower_index_middle_receiving':'엄지·약지는 유지하고 검지·중지를 기록된 낮은 받침 자세로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_receiving_check':'낮은 받침 구간 끝. 바로 엄지를 내리는 구간으로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_thumb_lower_before_support':'검지·중지는 유지하고 엄지를 먼저 내립니다. 약지는 엄지 이동 길을 함께 비킵니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_final_taught_1':'엄지·약지는 유지하고 검지·중지를 방금 기록한 1번 자세로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_final_waypoint_1_check':'1번 자세 구간 끝. 2번 자세로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_final_taught_2':'엄지·약지는 유지하고 검지·중지를 기록한 2번 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_final_waypoint_2_check':'2번 자세 구간 끝. 기존 G로 자동 마무리합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_return_final_taught_to_G':'엄지·약지는 유지하고 검지·중지만 기존 확정 G로 마무리합니다. SPACE/Q/Esc: 중단/OFF.',
            'direct_support_check':'확정 G 목표 구간 끝. 같은 목표를 계속 유지합니다. SPACE/Q/Esc: 종료/OFF.',
            'direct_fixed_grip_hold':'확정 G로 유지 중입니다. 시간 자동 종료는 없습니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }
        if phase in messages:return messages[phase]
        if phase.startswith('direct_forward_'):return self.forward.message('vertical_'+phase[len('direct_forward_'):])
        return phase+' / SPACE/Q/Esc: 중단/OFF.'

class Terminal(pose.Terminal):
    def __init__(self,sequence,report):self.sequence,self.report=sequence,report
    def handle(self,key):
        if key in (' ','q','Q','\x1b','\x03'):raise pose.OperatorStop()
    def check(self):
        import msvcrt
        while msvcrt.kbhit():self.handle(msvcrt.getwch())
