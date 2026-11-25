"""Summarize saved drive settings; never opens hardware."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    d=json.loads((ROOT/'leap_drive_readonly_result.json').read_text())
    rows=d['motors']
    summary=dict(timestamp=d['timestamp'],status=d['status'],
        all_torque_off=all(r['torque_enabled']==r['torque_enabled_end']==0 for r in rows),
        pwm_limits=sorted({r['drive_settings']['pwm_limit_raw'] for r in rows}),
        goal_pwm=sorted({r['drive_settings']['goal_pwm_raw'] for r in rows}),
        current_limits_ma=sorted({r['drive_settings']['current_limit_ma'] for r in rows}),
        restored_goal_current_ma=sorted({r['drive_settings']['goal_current_raw'] for r in rows}),
        actual_trial_goal_current_ma=450,trial_distal_pid=[450,0,150],
        conclusion='No low configured PWM limit observed after OFF. Prior in-motion output saturation/contact not established.',
        next='Empty-only time-scale2 interpolated path with same bounds and active PWM/velocity telemetry.')
    (ROOT/'sac_drive_diagnosis_result.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))
    p=ROOT/'sac_half_speed_empty_audit_result.json';a=json.loads(p.read_text())
    print('AUDIT_SHA',hashlib.sha256(p.read_bytes()).hexdigest())
    print('audit all_passed',a['all_passed'],'max_tracking',max(r['max_tracking_rad'] for r in a['results']),
          'count',a['command_count'],'duration',a['duration_seconds'],'max_step',a['path_max_step_ticks'])

if __name__=='__main__':main()
