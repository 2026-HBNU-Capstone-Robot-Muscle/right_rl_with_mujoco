"""User-taught vertical relocation, checked pitch-start handover, saved pitch."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from vertical_pitch_order_20261118 import reordered_pitch

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/vertical_cycle'
MANIFEST=BASE/'operator_trial_manifest.json'
PITCH=ROOT/'right_hand_rotation_20261028/two_axis_connection/saved_pitch/loaded_pitch_path.npz'
AFTER={'lift_index_middle':'vertical_lift_check',
       'thumb_push_to_vertical':'vertical_relocate_check',
       'thumb_ring_to_pitch_start':'vertical_support_check',
       'index_middle_to_pitch_start':'vertical_pitch_start_check',
       'pitch_thumb_retreat_first':'vertical_thumb_lowered_check'}
GATES=('vertical_grip_check',*AFTER.values())

def reference():
    ref=json.loads((BASE/'taught_reference_104640.json').read_bytes());source=ROOT/ref['source']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=ref['source_sha256']:raise RuntimeError('Vertical taught source changed')
    raw=json.loads(source.read_bytes());samples=raw['snapshots'];goal=pose.encoder_vector(ref['holding_goal_ticks'])
    if raw['status']!='readonly_observation_complete' or raw['max_two_read_difference_ticks']>1:raise RuntimeError('Vertical reference not stable')
    for s in samples:
        if s['torque']!=[1]*16 or any(s['error']) or s['goal']!=goal.tolist():raise RuntimeError('Vertical all-ON holding reference changed')
        if np.max(abs(np.asarray(s['position'])-goal))*2*np.pi/4096>.12:raise RuntimeError('Vertical reference exceeded existing tracking bound')
    return goal

class VerticalTransfer:
    def __init__(self,force_grip=False):
        self.force_grip=force_grip
        self.manifest=json.loads(MANIFEST.read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Vertical observation not enabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Vertical source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Vertical candidate changed')
        z=np.load(data,allow_pickle=False);old=np.load(PITCH,allow_pickle=False)
        self.start=pose.reference()[2];self.taught=reference();self.goals=z['goals'];self.phases=z['phases']
        self.taught_end=self.manifest['taught_end_index'];self.preparation=self.manifest['preparation_goals']
        if not np.array_equal(z['start'],self.start) or not np.array_equal(self.goals[self.taught_end],self.taught):raise RuntimeError('Vertical grip/taught endpoint changed')
        pitch,pitch_phases,order_review=reordered_pitch(old)
        if not np.array_equal(self.goals[self.preparation:],pitch):raise RuntimeError('Thumb-first pitch contract changed')
        validate_rows(self.start,self.goals);self.accepted=set();self.labels=[]
    def rows(self,current):
        self.adopted=bool(not self.force_grip and np.max(abs(np.asarray(current)-self.taught))<=4)
        target=self.taught if self.adopted else self.start
        intro=pose.path(current,target);rows=[*intro,*np.tile(target,(40,1)),target.copy()]
        labels=['vertical_adopt' if self.adopted else 'vertical_prepare']*len(intro)+['vertical_settle']*40
        labels.append('vertical_relocate_check' if self.adopted else 'vertical_grip_check')
        begin=self.taught_end+1 if self.adopted else 0
        for i in range(begin,len(self.goals)):
            phase=str(self.phases[i]);rows.append(self.goals[i].copy());labels.append('vertical_'+phase)
            if phase in AFTER and (i+1==len(self.goals) or self.phases[i+1]!=phase):
                rows.append(self.goals[i].copy());labels.append(AFTER[phase])
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        if len(result)*.075+30>=pose.MAX_ON_SECONDS:raise RuntimeError('Vertical movement and30second reserve exceed existing500second bound')
        self.labels=labels;return result
    def metadata(self):
        return dict(workflow='taught_vertical_relocation_then_thumb_first_pitch',adopted_current_taught_pose=self.adopted,
            taught_vertical_goal_ticks=self.taught.tolist(),source_vertical_goals=1268,
            queued_vertical_goals=len(self.goals)-self.preparation,thumb_before_index_middle_raise=True,
            forced_grip_entry=self.force_grip,relocation_order='index_middle_then_thumb_with_ring_clearance',
            phase_gate_indices={p:self.labels.index(p) for p in GATES if p in self.labels},operator_confirmed_stages=[],
            final_fixed_grip_return_included=False,physical_vertical_link_verified=False)
    def phase(self,index,moving):return self.labels[index] if moving else 'vertical_rotation_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        return {
            'vertical_adopt':'방금 기록한 세로 이송 자세를 그대로 이어받습니다. G로 돌아가지 않습니다.',
            'vertical_prepare':'현재 자세에서 확정 파지 G로 이동합니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_settle':'현재 목표에서 잠시 유지합니다.',
            'vertical_grip_check':'확정 파지와 큐브 지지 확인. SPACE로 검지·중지를 먼저 들어 비웁니다. 엄지·약지 목표는 유지합니다.',
            'vertical_lift_index_middle':'검지·중지만 기록한 준비 모양으로 먼저 옮깁니다. 엄지·약지는 파지 목표 유지. SPACE/Q/Esc: 중단/OFF.',
            'vertical_lift_check':'검지·중지가 큐브를 가로막지 않고 비켰는지 확인하세요. 엄지·약지가 받쳐주고 있으면 SPACE로 엄지 밀기. 불일치면 받치고 Q로 종료하세요.',
            'vertical_thumb_push_to_vertical':'검지·중지 목표를 유지하고 엄지로 밀면서 약지를 함께 비켜 기록한 이송 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_relocate_check':'큐브가 방금 가르친 세로 시작 자리에 받쳐져 있으면 SPACE로 엄지·약지를 먼저 옮깁니다. 불일치/끼임이면 받치고 Q로 종료하세요.',
            'vertical_thumb_ring_to_pitch_start':'검지·중지 목표를 유지하고 엄지·약지를 기존 세로 시작 쪽으로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_support_check':'엄지·약지가 큐브를 받치고 있으면 SPACE로 검지·중지를 세로 시작 자세로 옮깁니다. 못 받치면 큐브를 받치고 Q로 종료하세요.',
            'vertical_index_middle_to_pitch_start':'엄지·약지 목표를 유지하고 검지·중지를 기존 세로 시작 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_pitch_start_check':'세로 시작 자리와 큐브 지지를 확인한 뒤 SPACE로 엄지 내림부터 시작합니다. 자리 불일치/끼임이면 받치고 Q로 종료하세요.',
            'vertical_pitch_thumb_lower_first':'검지·중지 뿌리 ID8/12는 시작 각도로 유지하고 엄지 내림 구간을 먼저 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_pitch_thumb_clear_first':'검지·중지 뿌리를 유지하고 엄지를 낮춰 뒤집을 공간을 만듭니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_pitch_thumb_retreat_first':'검지·중지 뿌리를 유지하고 엄지 후퇴 준비를 끝냅니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_thumb_lowered_check':'엄지가 내려가 비켰고 큐브가 받쳐져 있으면 SPACE로 검지·중지를 세워 뒤집습니다. 불일치면 큐브를 받치고 Q로 종료하세요.',
            'vertical_pitch_index_middle_raise':'엄지 목표는 내린 위치로 유지하고 검지·중지 뿌리를 세워 큐브를 뒤집습니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_pitch_source_stage_10_loaded_index_middle_lift':'엄지 목표를 유지하고 검지·중지를 기존 끝 자세까지 세워 마무리합니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_rotation_hold':'세로 회전 끝 자세를 유지합니다. 이번 파일은 여기까지입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }.get(phase,'세로 회전 구간입니다. SPACE/Q/Esc: 중단/OFF.')

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
