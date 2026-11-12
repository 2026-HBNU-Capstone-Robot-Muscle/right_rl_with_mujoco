"""Create isolated runner/runtime copies with a cooperative camera hold hook."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def rep(s,a,b):
    if s.count(a)!=1:raise ValueError('Unexpected source block: '+a[:90])
    return s.replace(a,b,1)
def main():
    s=(ROOT/'quarter_motion_runner_20261025.py').read_text(encoding='utf-8')
    s=rep(s,'def observe(phase):\n            cycle_start=now()', 'def observe(phase):\n            nonlocal progress_guard\n            cycle_start=now()')
    s=rep(s,'def check_snapshot(state):','def check_snapshot(state,check_progress=True):')
    s=rep(s,'if len(progress):','if check_progress and len(progress):')
    old="state,errors=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)\n            return state,errors,cycle_start"
    new="""state,errors=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)
            guard=getattr(bus,'camera_motion_guard',None)
            if guard is not None:
                def read_hold():
                    report['phase']='camera_gap_hold:'+phase
                    return checked.sample(bus,command,report,'camera_gap_hold:'+phase,
                        snapshot_check=lambda state:check_snapshot(state,check_progress=False))
                elapsed=guard.wait_if_needed(read_hold,phase=phase,command=command,cancel=cancel,
                    now=now,sleep=sleep,motion_deadline=report['enabled_at']+timeout+paused_seconds)
                if elapsed:
                    # A deliberate fixed-goal hold is not commanded progress.
                    # Actual health and held-joint checks ran for every read.
                    progress_guard=GoalAwareProgress()
                    report['progress_history_reset_after_camera_hold']=True
                    report['phase']=phase;cycle_start=now()
                    state,errors=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)
            return state,errors,cycle_start"""
    s=rep(s,old,new)
    s=rep(s,'p=bus.plan_module',"p=bus.plan_module\n    sample=getattr(bus,'checked_sample',checked.sample)")
    s=s.replace('checked.sample(bus,command,report,','sample(bus,command,report,')
    (ROOT/'camera_gap_motion_runner_20261026.py').write_text(s,encoding='utf-8')
    r=(ROOT/'slow_forward_current_runtime_20261026.py').read_text(encoding='utf-8')
    r=rep(r,'from quarter_motion_runner_20261025 import run','from camera_gap_motion_runner_20261026 import run')
    r=rep(r,'bus=bus_type(sdk,report)',"bus=bus_type(sdk,report)\n        if getattr(plan_type,'CAMERA_GAP_HOLD',False):\n            if capture_mode or not getattr(camera,'EMPTY_ONLY',False):raise RuntimeError('Camera gap hold is EMPTY-only')\n            camera.bind_motion_hold();bus.camera_motion_guard=camera.motion_guard")
    r=rep(r,"Use leap_slow_forward_placement_20261026.py; no devices opened.","Use leap_thumb_clear_root_empty_20261026.py; no devices opened.")
    (ROOT/'camera_gap_current_runtime_20261026.py').write_text(r,encoding='utf-8')
    print('Created isolated EMPTY camera-gap runner/runtime; shared modules preserved.')
if __name__=='__main__':main()
