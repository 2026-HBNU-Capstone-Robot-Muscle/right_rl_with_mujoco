"""Diagnose saved voltage confirmations and the local clock, with no devices."""
import json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    print('CLOCKS',{k:vars(time.get_clock_info(k)) for k in ('monotonic','perf_counter','time')})
    for name in ('loaded_thumb_rotation_20261026_162938_932327','loaded_thumb_rotation_20261026_163144_150855'):
        r=json.loads((ROOT/name/'result.json').read_bytes())
        print('RUN',name,{k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored','cube_removal_confirmed')})
        for e in r.get('voltage_events',[]):
            sample=[s for s in r['samples'] if e['t']-.04<=s['t']<=e['t']+e['elapsed_ms']/1000+.002]
            print('EVENT',dict(event_t=e['t'],elapsed_ms=e['elapsed_ms'],status=e['status'],rows=[dict(t=s['t'],dt_ms=1000*(s['t']-e['t']),read_ms=s['read_ms'],minimum_v=min(s['voltage_v']),ids_below38=[i+1 for i,v in enumerate(s['voltage_v']) if v<3.8]) for s in sample]))
        print('POST_OFF',r.get('final_off_snapshots',[])[-1:] )
if __name__=='__main__':main()
