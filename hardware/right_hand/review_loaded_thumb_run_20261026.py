"""Inspect saved loaded thumb trials without opening any device."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    folders=[p for pattern in ('loaded_thumb_rotation_2026*','loaded_repeat_2026*') for p in ROOT.glob(pattern) if p.is_dir() and (p/'result.json').exists()]
    folders.sort(key=lambda p:(p/'result.json').stat().st_mtime)
    for folder in folders[-2:]:
        path=folder/'result.json'
        if not path.exists():print('NO_RESULT',folder.name);continue
        r=json.loads(path.read_bytes());samples=r.get('samples',[])
        keys=('timestamp','status','error','phase','failed_phase','plan_version','all_torque_off_verified','settings_restored',
              'current_start_screen_accepted','current_start_screen','cube_placement_confirmed','cube_removal_confirmed','cube_present',
              'physical_flip_verified','final_off_reference_valid','read_failures','voltage_events','fixed_hold_fault','progress_fault',
              'camera_recording','camera_image_signal','placement_photo','prepared_empty_photo','final_on_state')
        out={k:r[k] for k in keys if k in r};out.update(folder=folder.name,sample_count=len(samples),writes=len(r.get('write_events',[])),phases=dict(Counter(s['phase'] for s in samples)))
        if samples:out.update(min_voltage=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),max_tracking_rad=max(max(s['tracking_rad']) for s in samples))
        frames_path=folder/'camera/frames.json';selection=[]
        if frames_path.exists():
            frame_data=json.loads(frames_path.read_bytes());frames=frame_data.get('frames',[])
            for phase in dict.fromkeys(v['phase'] for v in frames):
                group=[v for v in frames if v['phase']==phase]
                selection.append(dict(phase=phase,count=len(group),first=group[0].get('image'),middle=group[len(group)//2].get('image'),last=group[-1].get('image')))
            out.update(frame_count=len(frames),phase_images=selection)
        (folder/'saved_run_review.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in out.items() if k not in ('final_on_state','phase_images','voltage_events')},ensure_ascii=False),flush=True)
        print('VOLTAGE_EVENTS',json.dumps(r.get('voltage_events',[]),ensure_ascii=False),flush=True)
        print('PHASE_IMAGES',json.dumps(selection,ensure_ascii=False),flush=True)
        for file in folder.glob('*.log'):
            print('LOG_TAIL',file.name,file.read_text(encoding='utf-8',errors='replace')[-800:],flush=True)
if __name__=='__main__':main()
