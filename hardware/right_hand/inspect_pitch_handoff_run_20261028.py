"""Bounded inspection of saved trial files, without opening any device."""
import argparse,json,hashlib
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder');args=parser.parse_args()
    folder=(ROOT/args.folder).resolve();folder.relative_to(ROOT)
    path=folder/'result.json'
    if not path.exists():print(json.dumps(dict(status='no_final_result',folder=folder.name)));return
    r=json.loads(path.read_bytes());samples=r.get('samples',[])
    keys=('timestamp','status','error','failed_phase','plan_version','camera_worker_stopped',
          'all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_removal_confirmed',
          'cube_present','placement_photo','pitch_observation_photo','transfer_observation_photo','transfer_final_photo',
          'current_start_screen_accepted','camera_recording','off_reference_capture_error')
    review={k:r[k] for k in keys if k in r}
    review.update(folder=folder.name,result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  samples=len(samples),sample_phases=dict(Counter(s['phase'] for s in samples)),
                  read_failures=len(r.get('read_failures',[])))
    if samples:review.update(min_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples))
    framepath=folder/'camera/frames.json'
    if framepath.exists():
        frames=json.loads(framepath.read_bytes())['frames'];groups=[]
        for phase in dict.fromkeys(f['phase'] for f in frames):
            group=[f for f in frames if f['phase']==phase]
            groups.append(dict(phase=phase,count=len(group),first=group[0].get('image'),middle=group[len(group)//2].get('image'),last=group[-1].get('image')))
        review.update(frame_count=len(frames),unusable_frames=sum(f.get('image_signal_usable') is not True for f in frames),phase_images=groups)
    out=folder/'offline_review';out.mkdir(exist_ok=True)
    (out/'saved_summary.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review,ensure_ascii=False))
if __name__=='__main__':main()
