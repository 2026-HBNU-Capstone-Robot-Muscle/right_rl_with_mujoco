"""READ/PING-only comparison; never writes a motor register."""
import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import dynamixel_sdk as sdk
from final_grip_standard_read_20261116 import block_once as standard,POLICY
from fast_health_read4_20261029 import block_once as fast
from calibrated_precise_clock_20261028 import install

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261116/final_grip_attempt_review'
port=sdk.PortHandler('COM7');report={};bus=SimpleNamespace(port=port,packet=sdk.PacketHandler(2.),sdk=sdk,report=report)
fields=[('torque',64,1),('error',70,1),('position',132,4),('goal',116,4),('current',126,2),('voltage',144,2),('temperature',146,1)]
rows=[];inventory=[]
try:
    if not port.openPort() or not port.setBaudRate(1_000_000):raise RuntimeError('Cannot open COM7 read-only')
    install(bus)
    for motor in range(1,17):
        model,comm,error=bus.packet.ping(port,motor)
        if comm or error or model!=1240:raise RuntimeError('Unexpected motor inventory')
        inventory.append(dict(id=motor,model=model))
    for n in range(20):
        for name,reader in (('standard',standard),('fast4',fast)):
            began=time.perf_counter();values,comm,error,detail=reader(bus,64,83,fields);elapsed=time.perf_counter()-began
            valid=comm==0 and error==0 and values is not None
            if valid and (np.any(values['torque']) or np.any(values['error'])):
                raise RuntimeError('Comparison requires healthy torque-OFF; no settings changed')
            rows.append(dict(pair=n,method=name,passed=valid,elapsed_ms=elapsed*1000,comm=comm,error=error,detail=detail))
            time.sleep(max(0.,.075-elapsed))
finally:port.closePort()
result=dict(status='readonly_comparison_complete',inventory=inventory,rows=rows,standard_policy=POLICY,
    motor_register_writes=0,torque_changes=0,camera_load_tested=False,port_closed=True,
    summary={method:dict(passed=sum(r['passed'] for r in rows if r['method']==method),
        total=sum(r['method']==method for r in rows),
        max_ms=max(r['elapsed_ms'] for r in rows if r['method']==method)) for method in ('standard','fast4')})
(OUT/'read_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','inventory')}))
