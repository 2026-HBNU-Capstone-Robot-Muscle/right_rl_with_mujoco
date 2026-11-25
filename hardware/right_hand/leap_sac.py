"""Deploy the SAC-derived distilled policy within learned joint limits."""
import json
from pathlib import Path

import hand_prepare_full as motion
from held_policy_shadow import main as run_main


def uncleared_off_failure(root=Path(__file__).resolve().parent):
    """After unverified OFF, require a newer, complete read-only inspection."""
    candidates=list(root.glob('leap_sac_20*.json'))
    candidates.append(root/'leap_sac_result.json')
    failed=[]
    for path in candidates:
        if not path.is_file():
            continue
        try:
            report=json.loads(path.read_text(encoding='utf-8'))
        except (OSError,ValueError):
            continue
        if report.get('all_torque_off_verified') is False:
            failed.append(path)
    if not failed:
        return None
    latest=max(failed,key=lambda p:p.stat().st_mtime)
    for name,status,key in (
            ('hand_readonly_check_result.json','read_complete_not_motion_approval','motors'),
            ('leap_restore_watchdog_result.json','idle_watchdog_restored_verified','postflight')):
        diagnostic=root/name
        if not diagnostic.is_file() or diagnostic.stat().st_mtime<=latest.stat().st_mtime:
            continue
        try:
            result=json.loads(diagnostic.read_text(encoding='utf-8'))
        except (OSError,ValueError):
            result={}
        rows=result.get(key,[])
        if (result.get('status')==status
                and (key!='postflight' or result.get('all_torque_off_verified') is True)
                and len(rows)==16 and {r.get('id') for r in rows}==set(range(1,17))
                and all(r.get('model_number')==1240 and r.get('torque_enabled')==0
                        and (key!='motors' or r.get('torque_enabled_end')==0)
                        and r.get('hardware_error')==0
                        and r.get('bus_watchdog_raw')==0 for r in rows)):
            return None
    return latest


def main():
    failed=uncleared_off_failure()
    if failed is not None:
        print('STOP: a previous motor torque-OFF check was unverified. Cut motor power.',flush=True)
        print('Before any SAC motion, power-cycle the empty hand and run hand_readonly_check.cmd.',flush=True)
        print('The read-only check must show all 16 torque OFF and watchdog=0.',flush=True)
        print(f'Unverified run: {failed}',flush=True)
        return 1
    print('LEAP SAC-derived policy: EMPTY prepare -> PLACE/SPACE -> POLICY 20s -> REMOVE/SPACE -> OFF.',flush=True)
    print('At PLACE, confirm ID40 and two cube markers are visible before starting.',flush=True)
    print('During a flip, markers may be occluded: this policy holds its last goal while vision is invalid.',flush=True)
    print('This trial does not bridge occlusion or reproduce the recorded flip sequence.',flush=True)
    print('PPO teacher + residual SAC distilled policy. Target: cube face ID0 up.',flush=True)
    print('LEARNED JOINT LIMITS: goals can travel beyond +/-0.10rad. Each command max 32 ticks; 550mA.',flush=True)
    print('Read timeout: one fresh read within 200ms; repeated faults stop. No motion retry.',flush=True)
    print('Use padded catch area. Hands clear before SPACE. Q/Esc/Ctrl+C stops.',flush=True)
    return run_main(motion_runner=motion,confirmation=None,result_stem='leap_sac',
                    loaded_policy_mode=True,sac_deployment_mode=True)


if __name__=='__main__': raise SystemExit(main())
