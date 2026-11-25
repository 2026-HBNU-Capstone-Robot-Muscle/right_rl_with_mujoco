"""Saved timing plus the known adapter's host registry value, read only."""
import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
def main():
    for name in ('forward_hold_observe_20261105_030930_516359','forward_hold_observe_20261105_034338_763669'):
        d=json.loads((R/name/'result.json').read_bytes());events=d.get('fast_health_complete_reads',[]);a=[v['elapsed_ms'] for v in events]
        result=dict(run=name,error=d.get('error'),sample_count=len(d.get('samples',[])),fast_read_count=len(a),read_latency_ms_p50_p95_p99_max=np.percentile(a,[50,95,99,100]).tolist() if a else None,last_read_latencies_ms=a[-12:],confirmation_events=d.get('read_recovery_confirmation_events',[]),read_failures=d.get('read_failures',[]),minimum_voltage_v=min(min(v['voltage_v']) for v in d['samples']),maximum_tracking_rad=max(max(v['tracking_rad']) for v in d['samples']),camera_blank_frames=d['camera_recording']['image_signal_blank_frames'],all_torque_off_verified=d['all_torque_off_verified'],settings_restored=d['settings_restored'])
        print(json.dumps(result,ensure_ascii=False))
    try:
        import winreg
        rootpath=r'SYSTEM\CurrentControlSet\Enum\FTDIBUS'
        out=[]
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,rootpath,0,winreg.KEY_READ) as root:
            for i in range(winreg.QueryInfoKey(root)[0]):
                sub=winreg.EnumKey(root,i)
                if 'FTBENPSAA' not in sub.upper():continue
                with winreg.OpenKey(root,sub,0,winreg.KEY_READ) as branch:
                    for j in range(winreg.QueryInfoKey(branch)[0]):
                        leaf=winreg.EnumKey(branch,j)
                        try:
                            with winreg.OpenKey(branch,leaf+r'\Device Parameters',0,winreg.KEY_READ) as params:
                                row={}
                                for key in ('PortName','LatencyTimer'):
                                    try:row[key]=winreg.QueryValueEx(params,key)[0]
                                    except FileNotFoundError:row[key]=None
                                out.append(row)
                        except FileNotFoundError:pass
        print('KNOWN_ADAPTER_HOST_SETTINGS',json.dumps(out))
    except Exception as exc:print('HOST_REGISTRY_READ_ERROR',str(exc))
if __name__=='__main__':main()
