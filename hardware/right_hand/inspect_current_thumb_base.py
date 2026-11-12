"""Read-only native pose bounds and thumb-base joint axis from saved files."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent
folder=root/'book_settled_off_readonly_20261024_174424_563949'
r=json.loads((folder/'result.json').read_bytes());request=json.loads((folder/'request.json').read_bytes())
limits=json.loads((root/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
raw=request['off_state']['position']
outside=[dict(motor_id=i+1,actual_ticks=v,lower_ticks=limits['lower_ticks'][i],upper_ticks=limits['upper_ticks'][i],
    below_degrees=max(0.,limits['lower_ticks'][i]-v)*360/4096) for i,v in enumerate(raw)
    if not limits['lower_ticks'][i]<=v<=limits['upper_ticks'][i]]
geometry=json.loads((folder/'geometry_review.json').read_bytes())
result=dict(status=r['status'],stable=r['off_start_endpoint_stable'],minimum_voltage_v=r['minimum_voltage_v'],
    torque=request['off_state']['torque'],outside_native=outside,
    thumb_base_joint=geometry['thumb_base_joint'],axis=geometry['axis'],
    servo_write_commands_sent=r['servo_write_commands_sent'],current_motion_admission_error=r.get('motion_admission_error'))
(folder/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
