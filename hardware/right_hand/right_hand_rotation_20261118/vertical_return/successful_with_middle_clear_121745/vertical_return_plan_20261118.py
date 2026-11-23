"""Empty pitch-end setup, operator placement, staged return to the same G."""
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_vertical_return_20261118 import ROOT,BASE,TRANSFER,GRIP_RETURN,VERTICAL,TAUGHT_PALM

MANIFEST=BASE/'operator_trial_manifest.json'
AFTER={'taught_palm_receiving':'vertical_return_palm_check',
       'middle_clear_for_push':'vertical_return_middle_clear_check',
       'return_transfer_thumb_push':'vertical_return_center_check',
       'return_middle_root_lead':'vertical_return_receiving_check',
       'return_grip_lower_push':'vertical_return_lower_check'}
GATES=('vertical_return_place',*AFTER.values())

class VerticalReturn:
    def __init__(self):
        self.manifest=json.loads(MANIFEST.read_bytes())
        if not self.manifest['operator_gated_observation_enabled']:raise RuntimeError('Vertical return observation disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Vertical return source changed: '+name)
        data=BASE/'candidate.npz'
        if hashlib.sha256(data.read_bytes()).hexdigest()!=self.manifest['candidate_sha256']:raise RuntimeError('Vertical return candidate changed')
        z=np.load(data,allow_pickle=False);v=np.load(VERTICAL,allow_pickle=False);t=np.load(TRANSFER,allow_pickle=False);g=np.load(GRIP_RETURN,allow_pickle=False)
        self.start=z['start'];self.goals=z['goals'];self.phases=z['phases'];self.grip=pose.reference()[2]
        n=self.manifest['alignment_goals'];m=self.manifest['historical_loaded_transfer_goals'];tail=self.manifest['observed_fixed_grip_tail_goals']
        if (not np.array_equal(self.start,v['goals'][-1]) or not np.array_equal(self.goals[n:n+m],t['goals'])
            or not np.array_equal(self.goals[-tail:],g['goals'][self.manifest['shared_tail_source_offset']:])
            or not np.array_equal(self.goals[-1],self.grip)):raise RuntimeError('Vertical return recorded-source contract changed')
        taught=np.asarray(json.loads(TAUGHT_PALM.read_bytes())['goal_ticks'],dtype=int)
        if not np.array_equal(self.goals[self.phases=='taught_palm_receiving'][-1],taught):
            raise RuntimeError('Taught palm receiving target changed')
        validate_rows(self.start,self.goals);self.accepted=set();self.labels=[]
    def rows(self,current):
        empty=pose.path(current,self.start)
        rows=[*empty,*np.tile(self.start,(20,1)),self.start.copy()]
        labels=['vertical_return_empty_setup']*len(empty)+['vertical_return_setup_settle']*20+['vertical_return_place']
        for i,(q,p) in enumerate(zip(self.goals,self.phases)):
            phase=str(p);rows.append(q.copy());labels.append('vreturn_'+phase)
            if phase in AFTER and (i+1==len(self.goals) or self.phases[i+1]!=phase):rows.append(q.copy());labels.append(AFTER[phase])
        result=np.asarray(rows,dtype=int);validate_rows(current,result)
        if len(result)*.075+30>=pose.MAX_ON_SECONDS:raise RuntimeError('Vertical return exceeds existing500second budget and30second reserve')
        self.labels=labels;return result
    def metadata(self):
        return dict(workflow='pitch_end_checked_return_to_fixed_G',phase_gate_indices={p:self.labels.index(p) for p in GATES},
            operator_confirmed_stages=[],final_fixed_grip_return_included=True,fixed_grip_target_ticks=self.grip.tolist(),
            preserved_loaded_transfer_goals=600,thumb_centering_push_included=True,
            taught_palm_target_ticks=self.manifest['taught_palm_target_ticks'],
            required_cube_center='between_index_and_middle',physical_vertical_return_verified=False,pitch_rotation_replayed=False)
    def phase(self,index,moving):return self.labels[index] if moving else 'vertical_return_fixed_grip_hold'
    def period(self,index,moving):return .075
    def ready(self,index):return self.labels[index] not in GATES or self.labels[index] in self.accepted
    def message(self,phase):
        return {
            'vertical_return_empty_setup':'빈손으로 세로 회전 끝 자세를 준비합니다. 큐브는 아직 손 밖에 두세요. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_setup_settle':'세로 회전 끝 자세에서 잠시 유지합니다.',
            'vertical_return_place':'세로 회전 직후와 같은 자리에 큐브를 놓고 손을 뗀 뒤 SPACE로 복귀 시작. 끼워 맞추지 말고 안 놓이면 Q로 종료하세요.',
            'vreturn_pitch_side_clearance':'기존 세로 이송의 작은 옆 여유를 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_pitch_palm_transfer':'기존 적재 이송 경로로 큐브를 손바닥 쪽에 받칩니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_taught_palm_receiving':'방금 기록한 손바닥 받침 자세로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_palm_check':'방금 기록한 받침 자세입니다. 큐브가 받쳐져 있으면 SPACE로 중지를 비우고 엄지 밀기를 준비합니다. 불일치면 받치고 Q로 종료하세요.',
            'vreturn_middle_clear_for_push':'엄지·검지·약지는 유지하고 중지만 기록한 비움 자세로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_middle_clear_check':'중지가 큐브 이동 길을 비웠고 큐브가 받쳐져 있으면 SPACE로 엄지 밀기 준비/밀기. 불일치면 받치고 Q로 종료하세요.',
            'vreturn_thumb_prepare_for_center':'중지는 비운 채 엄지·검지·약지를 기존 밀기 시작 자세로 맞춥니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_return_transfer_thumb_push':'가로 복귀에서 쓰던 엄지 밀기로 큐브를 중지 쪽으로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_center_check':'큐브가 검지 쪽에만 남지 않고 검지·중지 사이로 옮겨졌는지 확인하세요. 맞으면 SPACE로 하부 받침 준비. 아니면 큐브를 받친 뒤 Q로 종료하세요.',
            'vreturn_return_transfer_thumb_wrist_and_tips':'엄지를 손목 쪽으로 빼며 검지·중지 끝마디를 하부 받침 모양으로 준비합니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_return_middle_root_lead':'중지 맨 아랫마디를 기존 성공 높이1880틱까지 먼저 올립니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_receiving_check':'손가락 끝이 큐브 아래를 받칠 자리이고 큐브가 유지되면 SPACE. 불일치/끼임이면 받치고 Q로 종료하세요.',
            'vreturn_return_grip_index_middle_seat':'검지·중지를 접어 중간 받침을 만듭니다. 중지 뿌리1880틱 선행 높이를 사용합니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_return_grip_lower_push':'검지·중지로 큐브 아래를 받쳐 파지 쪽으로 옮깁니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_lower_check':'큐브가 검지·중지 사이의 확정 파지 자리에 받쳐졌으면 SPACE로 닫아 유지. 검지 쪽으로 치우쳤으면 받치고 Q로 종료하세요.',
            'vreturn_return_fixed_grip_close':'원래 확정 G 목표로 닫습니다. SPACE/Q/Esc: 중단/OFF.',
            'vreturn_return_fixed_grip_hold':'확정 파지 목표에서 약10초 정착합니다. SPACE/Q/Esc: 중단/OFF.',
            'vertical_return_fixed_grip_hold':'확정 G 목표로 유지 중입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }.get(phase,phase+' / SPACE/Q/Esc: 중단/OFF.')

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
