"""Read recent saved empty-run camera and motor events; no device access."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/camera_black_1111'
def main():
    runs=sorted(p for p in ROOT.glob('thumb_clear_root_empty_2026*') if p.is_dir())[-4:];reports=[]
    for folder in runs:
        r=json.loads((folder/'result.json').read_bytes());s=r.get('samples',[])
        out={k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored','current_start_screen_accepted','camera_recording','camera_off_adjustment_waits','camera_image_signal')}
        out.update(folder=folder.name,writes=len(r.get('write_events',[])),samples=len(s),phases=dict(Counter(v['phase'] for v in s)))
        if s:out.update(min_voltage=min(min(v['voltage_v']) for v in s),peak_current_ma=int(max(np.max(abs(np.asarray(v['current_ma']))) for v in s)),max_tracking=max(max(v['tracking_rad']) for v in s),last_sample=s[-1])
        p=folder/'camera/frames.json'
        if p.exists():
            data=json.loads(p.read_bytes());f=data.get('frames',[])
            out['bad_frames']=[v for v in f if v.get('image_signal_usable') is False][-15:]
            out['last_frames']=f[-6:]
            usable=[v for v in f if v.get('image_signal_usable')]
            out['last_good_image']=usable[-1]['image'] if usable else None
            times=np.array([v['camera_read_monotonic'] for v in f])
            if len(times)>1:out['saved_frame_gaps_s']=dict(median=float(np.median(np.diff(times))),max=float(np.max(np.diff(times))))
        reports.append(out)
    OUT.mkdir(exist_ok=True);(OUT/'review.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{k:v for k,v in r.items() if k not in ('last_sample','last_frames','bad_frames')} for r in reports],ensure_ascii=False,indent=2))
if __name__=='__main__':main()
