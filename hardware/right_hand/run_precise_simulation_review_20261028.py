"""User-started simulation/test review only. No motion entry point is called."""
import argparse,datetime as dt,json,subprocess,sys
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_loaded,validate_review
ROOT=p.ROOT
def wsl_path(path):
    path=Path(path).resolve();path.relative_to(ROOT)
    value=path.as_posix()
    if len(value)<3 or value[1:3]!=':/':raise ValueError('Expected a local Windows workspace path')
    return '/mnt/'+value[0].lower()+value[2:]
def model_command():
    return ['wsl','--exec','/home/star1209042/venv/bin/python','-B',wsl_path(ROOT/'screen_calibrated_quarter_20261028.py'),wsl_path(p.REQUEST),wsl_path(p.PROOF)]
def check_sources():
    plan=p.HorizontalQuarterPlan(p.REQUEST);rows=plan.plan(plan.source_position());loaded,static=validate_loaded()
    return dict(status='sources_checked_simulation_still_required',goals=len(rows),loaded_physics_cases=len(loaded['cases']),loaded_geometry_cases=len(static['cases']),current_start_review_exists=p.PROOF.exists(),no_device_access=True,motor_writes=0,command=model_command())
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-simulation',action='store_true');args=ap.parse_args()
    report=check_sources()
    if not args.run_simulation:print(json.dumps(report));return 0
    print('SIMULATION AND TESTS ONLY. No camera, serial port, torque or motor commands.',flush=True)
    print('Keep the robot as it is. Motor power is not needed for this check.',flush=True)
    try:
        if not p.PROOF.exists():
            print('Checking current EMPTY recovery in MuJoCo. This can take a few minutes.',flush=True)
            subprocess.run(model_command(),cwd=ROOT,check=True,timeout=600)
        validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
        for name in ('verify_precise_transport_units_20261028.py','verify_calibrated_prefix638_20261028.py'):
            subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/name)],cwd=ROOT,check=True,timeout=180)
        result=dict(status='simulation_and_all_tests_complete',model_review_passed=True,
            transport_tests=json.loads((p.OUT/'transport_unit_checks.json').read_bytes()),
            operator_tests=json.loads((p.OUT/'calibration_tests.json').read_bytes()),
            devices_opened=False,motor_writes=0,timestamp=dt.datetime.now().astimezone().isoformat())
        print('COMPLETE. No robot motion was run. Send just done for saved-file review.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        result=dict(status='simulation_or_test_review_stopped',error=str(exc),devices_opened=False,motor_writes=0)
        print('CHECK STOPPED:',str(exc),'No robot motion was run.',flush=True)
    (p.OUT/'local_simulation_check_result.json').write_bytes((json.dumps(result,indent=2)+'\n').encode())
    return 0 if result['status']=='simulation_and_all_tests_complete' else 2
if __name__=='__main__':raise SystemExit(main())
