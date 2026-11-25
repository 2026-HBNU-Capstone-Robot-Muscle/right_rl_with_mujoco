"""Compare the saved full/half-speed trials. No hardware access."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    candidates={}
    for p in sorted(ROOT.glob('leap_sac_path_empty_2*.json')):
        d=json.loads(p.read_text(encoding='utf-8'))
        if d.get('path_revision') in (3,4) and d.get('error','').startswith('ID12 tracking'):
            candidates[d['path_revision']]=(p,d)
    results=[]
    for rev,(p,d) in sorted(candidates.items()):
        rows=[r for r in d['samples'] if r['phase']=='reachable_empty_path'];last=rows[-1]
        series=[]
        for r in rows:
            series.append(dict(t=r['t'],goal=r['goal'][11],actual=r['actual'][11],
                current_ma=r['current_ma'][11],voltage_v=r['voltage_v'][11],
                pwm_percent=r['present_pwm_raw'][11]*.113 if 'present_pwm_raw' in r else None,
                velocity_rpm=r['present_velocity_raw'][11]*.229 if 'present_velocity_raw' in r else None))
        result=dict(revision=rev,evidence=p.name,timestamp=d['timestamp'],commands=d['path_commands_sent'],
            torque_off=d['all_torque_off_verified'],settings_restored=d['settings_restored'],last=series[-1],
            max_current_ma=max(abs(r['current_ma']) for r in series),
            max_abs_pwm_percent=max(abs(r['pwm_percent']) for r in series if r['pwm_percent'] is not None) if rev==4 else None,
            tail=series[-12:])
        results.append(result)
        print(json.dumps({k:v for k,v in result.items() if k!='tail'}))
        if rev==4:print('ID12 tail',json.dumps(series[-6:]))
    out=dict(status='offline_hardware_log_comparison',trials=results,motor_commands_sent=0,
        interpretation='At the same goal3350, half speed only changes actual3227 to3232. Simple target-rate lag alone is insufficient; no configured or observed PWM/current saturation at sampled fault. Physical resistance/contact or actuator dynamics require investigation; no permanent joint limit established.')
    (ROOT/'sac_id12_speed_comparison_result.json').write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
