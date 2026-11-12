"""Independent READ-only exercise of the precise-clock production helpers."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
code=(ROOT/'verify_reply40_production_readonly_20261028.py').read_text(encoding='utf-8')
code=code.replace('from horizontal_single_reply40_20261028 import block_once','from calibrated_precise_window_20261028 import block_once')
code=code.replace('from horizontal_receive_resync_20261027 import receive','from calibrated_precise_receive_20261028 import receive\n    from calibrated_precise_clock_20261028 import install')
code=code.replace('from horizontal_read_recovery_20261028 import sample_deadline,confirm_recovered_read','from horizontal_read_recovery_20261028 import sample_deadline\n    from calibrated_two_full_health_20261028 import confirm as confirm_recovered_read')
code=code.replace("'reply40_production_readonly_'","'precise_two_full_readonly_'")
code=code.replace("'reply40_production_readonly_complete'","'precise_two_full_readonly_complete'")
code=code.replace('time.monotonic()','time.perf_counter()')
bus="bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report,_read_recovery_times=[])"
code=code.replace('        '+bus+'\n','')
code=code.replace('        for motor in range(1,17):','        '+bus+'\n        install(bus)\n        for motor in range(1,17):')
code=code.replace("'readonly_link_observation',began=began,read_sample=read,validate=validate)","'readonly_link_observation',began=began,read_sample=read,validate=validate,now=time.perf_counter)")
(ROOT/'verify_precise_production_readonly_20261028.py').write_bytes(code.encode())
print('Precise transport probe ready; no devices opened.')
