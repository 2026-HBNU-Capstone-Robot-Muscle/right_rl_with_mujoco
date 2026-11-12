"""Read recent saved trial results only; no camera or motor access."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    for folder in sorted(ROOT.glob('thumb_clear_root_empty_2026*'))[-3:]:
        path=folder/'result.json'
        if not path.exists():continue
        r=json.loads(path.read_bytes());samples=r.get('samples',[])
        summary={k:r.get(k) for k in ('timestamp','status','error','failed_phase','all_torque_off_verified','settings_restored',
            'current_start_screen_accepted','plan_version','camera_recording','camera_image_signal','camera_gap_hold_policy',
            'camera_gap_holds','camera_gap_total_hold_seconds','progress_fault','fixed_hold_fault','final_command','current_start_screen')}
        summary.update(folder=folder.name,writes=len(r.get('write_events',[])),sample_count=len(samples),
            phases=dict(Counter(s['phase'] for s in samples)),last_samples=samples[-3:])
        if samples:
            summary.update(min_voltage=min(min(s['voltage_v']) for s in samples),
                peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
                max_tracking_rad=max(max(s['tracking_rad']) for s in samples))
        frames=folder/'camera/frames.json'
        if frames.exists():
            f=json.loads(frames.read_bytes()).get('frames',[])
            summary['last_frames']=f[-7:]
        (folder/'saved_stop_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in summary.items() if k not in ('last_samples','last_frames')},ensure_ascii=False))
        if summary['folder']==sorted(p.name for p in ROOT.glob('thumb_clear_root_empty_2026*'))[-1]:
            print('LAST_SAMPLES',json.dumps(samples[-2:],ensure_ascii=False))
if __name__=='__main__':main()
