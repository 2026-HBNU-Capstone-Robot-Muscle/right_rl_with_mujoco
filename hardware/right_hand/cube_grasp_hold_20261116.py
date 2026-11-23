"""Reuse the reviewed natural-recess closing route and keep its target latched."""
import json,hashlib,time,sys
from pathlib import Path
import numpy as np
from final_grip_pose_plan_20261116 import FinalGripPosePlan,ROOT,empty_prefix,hashes as empty_hashes,sha
from final_pose_start_scope_20261116 import validate_path
from natural_cradle_grasp_plan_20261030 import validate_candidate,ALLOWED,FIXED,ACTIVE
from new_pair_grasp_plan_20261027 import ENTRY
from final_grip_bus_20261116 import StandardGripBus
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from leap_reference_speed_yaw_20261029 import ReferenceCapture,runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from held_joint_guard import HeldJointGuard
from interrupted_finger_scope_20261116 import validate_report
import four_reply_yaw_gate_20261029 as transport
from natural_cradle_grasp_gate_20261030 import loaded_models
from final_grip_single_instance_20261116 import SingleInstance

OUT=ROOT/'right_hand_rotation_20261116/cube_grasp_hold';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';CANDIDATE=OUT/'candidate.npz'
def candidate():
    old=validate_candidate();mask=np.isin(old['phases'],ALLOWED[:2]);q=old['goals'][mask];p=old['phases'][mask]
    return dict(start=old['start'],goals=q,phases=p,forward_end_sample=np.asarray(len(q)))
def hashes():
    names=['cube_grasp_hold_20261116.py','screen_cube_grasp_hold_20261116.py','screen_cube_grasp_loaded_20261116.py','leap_cube_grasp_hold.cmd']
    return {**empty_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in [*(ROOT/n for n in names),REQUEST,CANDIDATE,OUT/'loaded/review.json']}}

class Plan(FinalGripPosePlan):
    PLAN_VERSION='natural_cube_grasp_latched_hold_20261116'
    TITLE='큐브 배치 -> 기존 자연 홈 파지 -> 유지 -> SPACE 종료/OFF'
    FOLDER_PREFIX='cube_grasp_hold';STATUS='cube_grasp_hold_observation_complete'
    COMPLETE_MESSAGE='큐브 파지 유지 관찰 종료. 토크 OFF와 설정 복구 결과를 확인하세요.'
    EMPTY_RELEASE_ONLY=False;CAPTURE_BEFORE_PHASE=ALLOWED[0]
    MODEL_LIMITS_DESCRIPTION='Natural-recess grasp, fixed index/ring; no rotation or automatic opening'
    proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        a=self.validate_start(raw);z=candidate();rows=empty_prefix(self,a)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]
        values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise RuntimeError('Thumb travel bound exceeded')
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Maximum ON bound exceeded')
        return rows
    def frozen_for_phase(self,p):
        if p==ALLOWED[0]:return FIXED
        if p in (ALLOWED[1],'cube_user_hold'):return np.arange(16)
        return super().frozen_for_phase(p)
    def progress_for_phase(self,p):
        if p==ALLOWED[0]:return ACTIVE
        if p in (ALLOWED[1],'cube_user_hold'):return np.array([],int)
        return super().progress_for_phase(p)

class Capture(ReferenceCapture):
    def place(self,bus,command,report,guard):
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report['camera_instruction']='GRASP then KEEP HOLDING. No automatic OPEN. SPACE ends/OFF.'
        print('이번 파일은 잡은 뒤 계속 유지합니다. 자동 OPEN 없음. 종료 전 큐브를 받치고 SPACE.',flush=True)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now()
            print('파지 자세 도착. 계속 유지합니다. 종료 전 큐브를 받치고 터미널 SPACE.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed') or bus.pending or not np.array_equal(command,candidate()['goals'][-1]):
            raise RuntimeError('Cube placement/closing queue incomplete; OFF')
        report.update(automatic_open=False,physical_grasp_verified=False,final_hold_goal_writes=0)
        self.keys.arm();began=self.now();deadline=report['enabled_at']+bus.plan_module.ON_TIMEOUT
        state,_=bus.checked_sample(bus,command,report,'cube_user_hold');guard=HeldJointGuard(state['position'],command)
        while True:
            cycle=self.now();self.cancel()
            if cycle>=deadline:report['hold_end_reason']='500s_total_on_limit';break
            state,_=bus.checked_sample(bus,command,report,'cube_user_hold',snapshot_check=lambda s:guard.check(s['position'],command,np.arange(16),'cube_user_hold'))
            if self.keys.consume_space():report['hold_end_reason']='operator_SPACE';break
            self.sleep(min(max(0,bus.plan_module.SAMPLE_PERIOD-(self.now()-cycle)),max(0,deadline-self.now())))
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_hold_seconds=self.now()-began)

def make_review(p,empty):
    validate_report(empty);models=loaded_models();r=transport.make_review(p,empty,transport.hashes())
    short=candidate();stored=dict(np.load(CANDIDATE));n=len(short['goals'])
    if (not np.array_equal(stored['start'],short['start']) or len(stored['goals'])!=n+400
        or not np.array_equal(stored['goals'][:n],short['goals']) or not np.array_equal(stored['phases'][:n],short['phases'])
        or not np.all(stored['goals'][n:]==short['goals'][-1]) or not np.all(stored['phases'][n:]==ALLOWED[1])):
        raise RuntimeError('Loaded observation no longer matches actual closing queue')
    loaded=json.loads((OUT/'loaded/review.json').read_bytes())
    if loaded.get('candidate_sha256')!=sha(CANDIDATE) or not loaded.get('all_passed') or len(loaded.get('cases',[]))!=3:
        raise RuntimeError('New persistent hold loaded model review incomplete')
    if loaded.get('model_source_sha256')!=sha(ROOT/'screen_natural_grasp_hold_20261030.py'):
        raise RuntimeError('Loaded checker changed')
    if {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in loaded['cases']}!={(f,1.,.3) for f in (-15.,-10.,-5.)}:
        raise RuntimeError('Missing loaded hold conditions')
    for c in loaded['cases']:
        if not c['passed'] or c['stop'] is not None or c['last_sample']!=len(np.load(CANDIDATE)['goals']):raise RuntimeError('Loaded hold did not complete')
        # Keep task failures visible. This is a supervised observation file,
        # not a declaration that the long hold meets the2mm/80% task target.
        from calibrated_quarter_gate_20261028 import bounded
        if (not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand']
            or not bounded(c,dict(max_self_mm=1e-6,max_support_mm=.15,max_cube_mm=1,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6,longest_no_hand_contact_seconds=.25,center_translation_mm=80))):
            raise RuntimeError('Loaded hold safety check failed')
    for key in tuple(r):
        if key.startswith(('loaded_','completed_physical','nominal_model')):r['historical_transport_'+key]=r.pop(key)
    from final_grip_standard_read_20261116 import POLICY
    r.update(status='supervised_cube_grasp_hold_ready',code_hashes=hashes(),empty_only=False,
        cube_placement_allowed=True,automatic_open=False,rotation_included=False,
        natural_cradle_models=models,loaded_hold_review=loaded,active_motion_limit_s=500,
        task_target_thresholds_unchanged=True,long_hold_task_verified=all(c['grasp_hold_observation']['passed'] for c in loaded['cases']),
        active_health_read_policy=dict(POLICY),physical_grasp_verified=False,
        final_TH_MF_RF_model_pose_used=False,scope='Existing natural recess TH/MF grasp; IF/RF fixed. Hold until SPACE or500s total. Not rotation-to-final-three-finger transition.')
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'])
    if r!=expected:raise RuntimeError('Cube hold proof/source/queue changed; no writes')
    return r
def run_screen(request,output,**kwargs):
    from pitch_handoff_offscreen_process_20261028 import run_screen as process
    p=Plan(request);old=validate_review(Plan(REQUEST),PROOF)
    if np.array_equal(p.source_position(),Plan(REQUEST).source_position()):
        Path(output).write_text(json.dumps(make_review(p,old['empty_recovery']),indent=2)+'\n')
    else:process(request,output,checker=ROOT/'screen_cube_grasp_hold_20261116.py',failure_details=lambda _:'Cube hold EMPTY preparation rejected',**kwargs)
    return validate_review(p,output)
class Bus(StandardGripBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    def checked_sample(self,*args,**kwargs):
        # The inherited runner pauses its motion clock during placement.
        report=args[2] if len(args)>2 else self.report
        if report.get('enabled_at') is not None and time.monotonic()-report['enabled_at']>=500:raise RuntimeError('Total torque ON500s reached; OFF')
        return super().checked_sample(*args,**kwargs)
Plan.CAMERA_TYPE=NaturalCamera;Plan.CAPTURE_PROTOCOL_TYPE=Capture
def main():
    from contextlib import nullcontext
    try:
        with SingleInstance(ROOT/'.final_grip_pose.lock') if '--execute' in sys.argv else nullcontext():
            validate_review(Plan(REQUEST),PROOF)
            print('빈손으로 시작. 첫 SPACE 준비 -> PLACE 안내 후 큐브 -> 손 빼고 두번째 SPACE 파지/유지.',flush=True)
            print('기존 자연 홈 파지를 유지합니다. 엄지/중지 동작, 검지/약지 고정. 회전 없음.',flush=True)
            print('자동으로 열지 않습니다. 종료하려면 큐브를 받치고 SPACE. Q/Esc/Ctrl+C 중단.',flush=True)
            saved=runtime.run
            try:runtime.run=confirmed_run;return runtime.main(Plan,bus_type=Bus,screener=run_screen)
            finally:runtime.run=saved
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as e:
        print('NOT READY:',e,'. No new motion started.',flush=True);return 2
if __name__=='__main__':raise SystemExit(main())
