"""Inspect saved trial failures without opening cameras or motor ports."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def small(value,depth=0):
    if isinstance(value,dict):
        return {k:small(v,depth+1) for k,v in value.items()} if depth<2 else {'keys':list(value)[:30]}
    if isinstance(value,list):
        if len(value)<=16 and all(not isinstance(v,(dict,list)) for v in value):return value
        return {'count':len(value),'last':small(value[-1],depth+1) if value else None}
    return value
def main():
    folders=sorted([p for p in ROOT.glob('fresh_empty_grasp_*') if p.is_dir()],key=lambda p:p.stat().st_mtime,reverse=True)
    for folder in folders[:3]:
        print('\nFOLDER',folder.name)
        print('FILES',[p.name for p in folder.iterdir() if p.is_file()])
        path=folder/'result.json'
        if path.exists():
            result=json.loads(path.read_bytes());samples=result.get('samples',[]);events=result.get('voltage_events',[])
            summary={k:result.get(k) for k in ('timestamp','status','error','failed_phase','all_torque_off_verified','settings_restored','current_start_screen_accepted','fixed_hold_fault','progress_fault')};summary['sample_count']=len(samples);summary['voltage_events']=[dict(t=e['t'],status=e['status'],initial_min_v=min(e['initial_voltage_v']),last_min_v=min(e.get('last_voltage_v',e['initial_voltage_v'])),elapsed_ms=e.get('elapsed_ms'),confirmations=e.get('healthy_confirmation_count'),reads=e.get('confirmation_reads')) for e in events];summary['sample_max_tracking_rad']=max((max(s['tracking_rad']) for s in samples),default=None);summary['sample_peak_current_ma']=max((max(abs(v) for v in s['current_ma']) for s in samples),default=None);summary['last_sample']={k:samples[-1][k] for k in ('t','goal','actual','current_ma','voltage_v')} if samples else None;summary['camera']={k:result.get('camera_recording',{}).get(k) for k in ('frame_count','image_signal_blank_frames','recovered_gap_count')};summary['initial_position']=result.get('initial',{}).get('position',result.get('preflight_state',{}).get('position'));print(json.dumps(summary,ensure_ascii=False))
if __name__=='__main__':main()
