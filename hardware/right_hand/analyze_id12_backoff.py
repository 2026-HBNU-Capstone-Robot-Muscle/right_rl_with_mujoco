"""Offline measured return response; no causal contact claim or hardware access."""
from pathlib import Path
import json,hashlib,math
ROOT=Path(__file__).resolve().parent

def main():
    p=ROOT/'leap_id12_backoff_empty_result.json';d=json.loads(p.read_text())
    a,b=d['approach_hold'],d['return_hold'];rows=[]
    for i in range(16):
        rows.append(dict(motor_id=i+1,goal_before=a['goal'][i],goal_after=b['goal'][i],
            actual_before=a['actual_end_median'][i],actual_after=b['actual_end_median'][i],
            current_before_ma=a['current_end_median_ma'][i],current_after_ma=b['current_end_median_ma'][i],
            error_before_deg=math.degrees(a['tracking_end_median_rad'][i]),
            error_after_deg=math.degrees(b['tracking_end_median_rad'][i])))
    result=dict(timestamp=d['timestamp'],source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        status=d['status'],torque_off=d['all_torque_off_verified'],settings_restored=d['settings_restored'],
        joints=rows,observed=f"ID12 returned {a['actual_end_median'][11]-b['actual_end_median'][11]:g} ticks; compare other joints with fixed goals. Pose/direction-dependent resistance, not bilateral immobility.",
        cause='Unresolved. Contact, cable/load transmission and friction remain possible. Return response does not identify a collision partner.',
        physical_thumb_contact_verified=False,motor_commands_sent=0)
    (ROOT/'id12_backoff_analysis_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({**{k:v for k,v in result.items() if k!='joints'},'relevant_joints':[rows[i] for i in (1,10,11)]},indent=2))

if __name__=='__main__':main()
