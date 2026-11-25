"""User-started EMPTY preparation and45s hold; no cube or loaded commands."""
import time,numpy as np
from empty_hold_comm_plan_20261029 import EmptyHoldCommPlan,ROOT,REQUEST,PROOF,HOLD_SECONDS
from empty_hold_comm_gate_20261029 import validate_review,run_screen,OFF_READ_SCOPES
from leap_retained_slow_yaw_20261029 import RetainedBus,PairGraspCamera,runtime
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from empty_hold_motion_runner_20261029 import run as guarded_run
from fast_health_read_20261029 import block_once as fast_block
from combined_quiet_reserve_20261029 import receive
class EmptyCamera(PairGraspCamera):EMPTY_ONLY=True
def fixed_hold(observe,*,now=time.monotonic,sleep=time.sleep,duration=45.,period=.075):
    began=now();count=0
    while now()-began<duration:
        cycle=now();observe();count+=1
        delay=min(max(0.,period-(now()-cycle)),max(0.,duration-(now()-began)))
        if delay:sleep(delay)
    return dict(checks=count,elapsed_seconds=now()-began,period_s=period,goal_writes=0)
class EmptyProtocol(NewPairGraspCapture):
    def release(self,*args):raise RuntimeError('Unexpected contact-release callback')
    def finish(self,bus,command,report):
        guard=getattr(bus,'empty_hold_guard',None)
        if guard is None:raise RuntimeError('Original held-joint guard missing')
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        report.update(cube_present=False,empty_hold_started=True)
        print('준비 완료. 큐브는 계속 밖에 둡니다.45초 동안 목표를 고정하고 통신을 확인한 뒤 자동 OFF합니다.',flush=True)
        def check():
            if self.now()-report['enabled_at']>bus.plan_module.ON_TIMEOUT:raise RuntimeError('Existing300s bound exceeded; OFF')
            observe('empty_fixed_hold_probe')
        report['empty_fixed_hold_probe']=fixed_hold(check,now=self.now,sleep=self.sleep,duration=HOLD_SECONDS)
        report['empty_hold_complete']=True
class EmptyBus(RetainedBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    def _block_once(self,address,length,fields):
        if (address,length) not in OFF_READ_SCOPES:return super()._block_once(address,length,fields)
        augmented=list(fields)+[('_OFF_torque',64,1),('_OFF_error',70,1)]
        values,comm,error,detail=fast_block(self,64,83,augmented)
        if not comm and not error:
            if np.any(values.pop('_OFF_torque')) or np.any(values.pop('_OFF_error')):raise RuntimeError('OFF setting verification requested while torque/error present')
        self.report.setdefault('fast_OFF_setting_readbacks',[]).append(dict(address=address,length=length,comm=comm,error=error))
        return values,comm,error,detail
    def _receive(self,operation,label,request):
        if request.get('kind')=='sync_read' and (request.get('address'),request.get('length')) in OFF_READ_SCOPES:
            normalized=dict(request,address=64,length=83,OFF_verified_address=request['address'],OFF_verified_length=request['length'])
            return receive(self,operation,label,normalized)
        return super()._receive(operation,label,request)
EmptyHoldCommPlan.CAMERA_TYPE=EmptyCamera;EmptyHoldCommPlan.CAPTURE_PROTOCOL_TYPE=EmptyProtocol
def main():
    try:validate_review(EmptyHoldCommPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손 통신 유지 검사: 처음부터 끝까지 큐브를 놓지 않습니다. SPACE 한 번, 약1분30초 뒤 자동 OFF.',flush=True)
    print('준비 자세로 이동한 뒤45초 고정합니다. 유지 중 새 목표 쓰기 없이75ms 주기로 상태를 읽습니다.',flush=True)
    print('450mA/PID/전압/온도/추종/200ms 정상 상태2회 확인과 원래 held-joint 검사를 유지합니다. Q/Esc/Ctrl+C는 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=guarded_run;return runtime.main(EmptyHoldCommPlan,bus_type=EmptyBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
