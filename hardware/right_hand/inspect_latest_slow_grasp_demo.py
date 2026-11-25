"""Compact inspection of saved operator-run slow demo; no devices."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    folders=sorted((p for p in ROOT.glob('slow_grasp_demo_2*') if p.is_dir()),reverse=True)
    if not folders:
        print('NO_SLOW_DEMO_FOLDER');return
    folder=folders[0];path=folder/'result.json'
    print('FOLDER',folder.name)
    if not path.is_file():print('NO_RESULT_JSON_YET');return
    data=json.loads(path.read_bytes())
    keys=('timestamp','status','phase','error','failed_phase','all_torque_off_verified',
        'settings_restored','current_start_screen_accepted','cube_placement_confirmed',
        'cube_removal_confirmed','cube_present','contact_release_operator_confirmed',
        'camera_recording','camera_exposure_restoration_verified','current_start_screen',
        'placement_photo','grasp_observation_photo','open_before_cube_photo',
        'contact_release_photo','enabled_at','final_check_error')
    print(json.dumps({k:data.get(k) for k in keys},ensure_ascii=False,indent=2))
    samples=data.get('samples',[]);phases={}
    for row in samples:phases[row['phase']]=phases.get(row['phase'],0)+1
    print('SAMPLES',len(samples),'PHASE_COUNTS',json.dumps(phases))
    if samples:
        print('FIRST_SAMPLE',json.dumps(samples[0],ensure_ascii=False))
        print('LAST_SAMPLE',json.dumps(samples[-1],ensure_ascii=False))
    camera=folder/'camera/frames.json'
    if camera.is_file():
        frames=json.loads(camera.read_bytes()).get('frames',[])
        print('CAMERA_START',json.dumps(frames[:3],ensure_ascii=False))
        print('CAMERA_END',json.dumps(frames[-3:],ensure_ascii=False))


if __name__=='__main__':main()
