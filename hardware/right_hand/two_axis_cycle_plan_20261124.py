"""Source-bound horizontal -> G hold -> vertical -> G, using one controller."""
from contextlib import contextmanager
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from grip_rotation_return_plan_20261118 import GripRotationReturn
from vertical_direct_grip_plan_20261118 import VerticalDirectGrip
from build_vertical_direct_grip_20261118 import grip_entry
from retaught_transfer_scope_20261118 import limits as horizontal_limits,ORIGINAL_BOUNDS
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261124/two_axis_cycle'
PERIOD=.075
RUN_LIMIT_SECONDS=float('inf')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def components():
    if pose.bounds is not ORIGINAL_BOUNDS:raise RuntimeError('Unexpected active joint bounds')
    horizontal=GripRotationReturn();vertical=VerticalDirectGrip()
    if (not np.array_equal(horizontal.start,vertical.start)
        or not np.array_equal(horizontal.goals[-1],vertical.start)
        or not np.array_equal(vertical.goals[-1],vertical.start)):
        raise RuntimeError('Both cycles must start/end at the exact same G')
    with horizontal_limits():
        h_bounds=pose.bounds()
        validate_rows(horizontal.start,horizontal.goals)
    validate_rows(vertical.start,vertical.goals)
    if np.max(abs(vertical.goals[0]-horizontal.goals[-1]))>2:
        raise RuntimeError('Horizontal-to-vertical goal join exceeds2ticks')
    return horizontal,vertical,h_bounds

class TwoAxisCycle:
    def __init__(self):
        self.manifest=json.loads((BASE/'operator_trial_manifest.json').read_bytes())
        if not self.manifest.get('automatic_stage_transitions_enabled'):
            raise RuntimeError('Two-axis automatic cycle disabled')
        for name,digest in self.manifest['source_hashes'].items():
            if sha(ROOT/name)!=digest:raise RuntimeError('Two-axis source changed: '+name)
        if sha(BASE/'candidate.npz')!=self.manifest['candidate_sha256']:
            raise RuntimeError('Two-axis candidate changed')
        self.horizontal,self.vertical,self.h_bounds=components()
        z=np.load(BASE/'candidate.npz',allow_pickle=False)
        self.start=z['start'];self.goals=z['goals'];self.phases=z['phases']
        self.h_count=len(self.horizontal.goals)
        expected=np.concatenate([np.array(['h_'+str(p) for p in self.horizontal.phases]),
                                 np.array(['v_'+str(p) for p in self.vertical.phases])])
        if (not np.array_equal(self.start,self.vertical.start)
            or not np.array_equal(self.goals[:self.h_count],self.horizontal.goals)
            or not np.array_equal(self.goals[self.h_count:],self.vertical.goals)
            or not np.array_equal(self.phases,expected)):
            raise RuntimeError('Two-axis component goals/phases changed')
        self.active_leg='entry';self.labels=[]
        self._runtime_bounds=self.current_bounds

    def horizontal_bounds(self):return tuple(a.copy() for a in self.h_bounds)
    def current_bounds(self):
        # The existing taught horizontal exception is limited to that leg.
        return self.horizontal_bounds() if self.active_leg=='horizontal' else ORIGINAL_BOUNDS()

    @contextmanager
    def runtime_limits(self):
        if pose.bounds is not ORIGINAL_BOUNDS:raise RuntimeError('Unexpected runtime limit override')
        previous=pose.bounds;pose.bounds=self._runtime_bounds
        try:yield
        finally:pose.bounds=previous

    def check_rows(self,start,rows,bounds):
        previous=pose.bounds;pose.bounds=bounds
        try:validate_rows(start,rows)
        finally:pose.bounds=previous

    def rows(self,current):
        self.active_leg='entry'
        intro,labels=grip_entry(current,self.start)
        self.check_rows(current,intro,ORIGINAL_BOUNDS)
        self.check_rows(self.start,self.goals[:self.h_count],self.horizontal_bounds)
        self.check_rows(self.start,self.goals[self.h_count:],ORIGINAL_BOUNDS)
        rows=[*intro,*np.tile(self.start,(40,1)),self.start.copy()]
        labels=['chain_entry_'+p for p in labels]+['chain_settle_G']*40+['chain_start_G_check']
        self.h_begin=len(rows)
        rows.extend(self.goals[:self.h_count]);labels.extend('chain_'+str(p) for p in self.phases[:self.h_count])
        self.h_end=len(rows);self.handoff_index=len(rows)
        rows.append(self.start.copy());labels.append('chain_handoff_G_check')
        self.v_begin=len(rows)
        rows.extend(self.goals[self.h_count:]);labels.extend('chain_'+str(p) for p in self.phases[self.h_count:])
        self.v_end=len(rows)
        rows.append(self.start.copy());labels.append('chain_final_G_check')
        result=np.asarray(rows,dtype=int)
        if np.max(abs(np.diff(np.vstack([current,result]),axis=0)))>2:
            raise RuntimeError('Combined goal step exceeds2ticks')
        if not np.array_equal(result[-1],self.start):raise RuntimeError('Combined final G changed')
        self.labels=labels;return result

    def metadata(self):
        hold=int(np.sum(self.horizontal.phases=='return_fixed_grip_hold'))
        return dict(workflow='G_horizontal_G_vertical_G_one_controller',
            fixed_grip_target_ticks=self.start.tolist(),horizontal_goals=self.h_count,
            vertical_goals=len(self.vertical.goals),horizontal_G_hold_seconds=hold*PERIOD,
            phase_gate_indices={},operator_confirmed_stages=[],
            automatic_stage_transitions_enabled=True,intermediate_OFF_included=False,
            handoff_G_index=self.handoff_index,horizontal_indices=[self.h_begin,self.h_end],
            vertical_indices=[self.v_begin,self.v_end],maximum_ON_seconds=None,
            elapsed_time_limit_enabled=False,period_seconds=PERIOD,maximum_step_ticks=2,
            horizontal_recorded_limits_scoped_to_horizontal_only=True,
            vertical_uses_original_bounds=True,automatic_repetition_included=False,
            physical_two_axis_cycle_verified=False,physical_cube_placement_handoff_verified=False)

    def phase(self,index,moving):
        self.active_leg=('horizontal' if moving and self.h_begin<=index<self.h_end else
                         'vertical' if moving and self.v_begin<=index<self.v_end else 'entry')
        return self.labels[index] if moving else 'chain_final_G_hold'
    def period(self,index,moving):return PERIOD
    def ready(self,index):return True
    def message(self,phase):
        if phase.startswith('chain_entry_'):return self.vertical.message(phase[len('chain_entry_'):])
        special={
            'chain_settle_G':'시작 G에서 잠시 정착합니다.',
            'chain_start_G_check':'시작 파지 구간 끝. 가로 회전으로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'chain_h_return_fixed_grip_hold':'가로 회전 뒤 G에서 약10초 유지한 다음 세로 회전으로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.',
            'chain_handoff_G_check':'가로 복귀 G 유지 끝. 토크를 유지한 채 세로 회전으로 이어집니다. SPACE/Q/Esc: 중단/OFF.',
            'chain_final_G_check':'세로 회전 뒤 G 목표 구간 끝. 같은 목표를 계속 유지합니다. SPACE/Q/Esc: 종료/OFF.',
            'chain_final_G_hold':'가로·세로 회전 연결 끝. G로 계속 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',
        }
        if phase in special:return special[phase]
        if phase.startswith('chain_h_forward_'):
            return '가로: '+self.horizontal.forward.message('retaught_'+phase[len('chain_h_forward_'):])
        if phase.startswith('chain_h_'):
            return '가로: '+self.horizontal.message('cycle_'+phase[len('chain_h_'):])
        if phase.startswith('chain_v_'):return '세로: '+self.vertical.message(phase[len('chain_v_'):])
        return phase+' / SPACE/Q/Esc: 중단/OFF.'

class Terminal(pose.Terminal):
    def handle(self,key):
        if key in (' ','q','Q','\x1b','\x03'):raise pose.OperatorStop()
    def check(self):
        import msvcrt
        while msvcrt.kbhit():self.handle(msvcrt.getwch())
