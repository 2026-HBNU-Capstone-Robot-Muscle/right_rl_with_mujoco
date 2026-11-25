"""Review stored user-run telemetry and raw camera frames; never devices."""
import hashlib
import json
from pathlib import Path
from collections import Counter
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'supported_yaw_empty_20261023_030552_675209'

def main():
    r=json.loads((FOLDER/'result.json').read_bytes())
    from supported_yaw_empty_plan import candidate,SUFFIX_PHASE
    rows=r['samples'];suffix=[s for s in rows if s['phase']==SUFFIX_PHASE]
    print('WRITE_EVENT_EXAMPLE',json.dumps(r['write_events'][0])[:700])
    # Samples read the previous accepted target before sending the next one.
    expected=np.vstack([suffix[0]['goal'],candidate()[:-1]])
    exact=bool(np.array_equal(np.array([s['goal'] for s in suffix]),expected))
    final=bool(np.array_equal(r['final_on_state']['goal'],candidate()[-1]))
    phases={}
    for name in dict.fromkeys(s['phase'] for s in rows):
        group=[s for s in rows if s['phase']==name]
        phases[name]=dict(samples=len(group),start_s=group[0]['t'],end_s=group[-1]['t'],
            min_voltage_v=float(np.min([s['voltage_v'] for s in group])),
            peak_absolute_current_ma=float(np.max(abs(np.array([s['current_ma'] for s in group])))),
            max_tracking_rad=float(np.max([s['tracking_rad'] for s in group])))
    result=dict(status=r['status'],source_result_sha256=hashlib.sha256((FOLDER/'result.json').read_bytes()).hexdigest(),
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        planned_suffix_rows=256,recorded_suffix_rows=len(suffix),exact_previous_targets=exact,final_target_exact=final,
        contact_release_operator_confirmed=r['contact_release_operator_confirmed'],
        contact_release_actual_ticks=r['contact_release_actual_ticks'],
        contact_release_threshold_ticks=r['contact_release_threshold_ticks'],
        min_voltage_v=min(p['min_voltage_v'] for p in phases.values()),
        peak_absolute_current_ma=max(p['peak_absolute_current_ma'] for p in phases.values()),
        max_tracking_rad=max(p['max_tracking_rad'] for p in phases.values()),phases=phases,
        camera=r['camera_recording'],cube_allowed=False,physical_rotation_verified=False,
        final_off_endpoint_stable=r['final_off_reference_valid'],
        final_off_endpoint_drift_ticks=r.get('final_off_endpoint_drift_ticks'),
        agent_hardware_commands_sent=0)
    assert r['status']=='empty_supported_yaw_rehearsal_complete' and exact and final and len(suffix)==256
    (FOLDER/'completion_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())
    print('FRAMES_TYPE',type(frames).__name__)
    if isinstance(frames,dict):print('FRAMES_KEYS',list(frames))
    elif frames:print('FRAME_EXAMPLE',json.dumps(frames[0])[:800]);print('FRAME_LAST',json.dumps(frames[-1])[:800])
    print(json.dumps({k:result[k] for k in ('status','recorded_suffix_rows','exact_previous_targets','min_voltage_v',
        'peak_absolute_current_ma','max_tracking_rad','all_torque_off_verified','settings_restored','final_off_endpoint_stable')},indent=2))

if __name__=='__main__':main()
