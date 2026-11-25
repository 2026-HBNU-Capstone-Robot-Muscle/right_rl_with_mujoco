"""Offline telemetry/image-content review of the user-run 15:05 trial."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_small_rotation_trial_20261022_150532_457924'


def main():
    global FOLDER
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--folder');args=parser.parse_args()
    if args.folder:
        if Path(args.folder).name!=args.folder:raise ValueError('Expected a trial folder name under outputs')
        FOLDER=ROOT/args.folder
    source=FOLDER/'result.json';r=json.loads(source.read_bytes());samples=r['samples']
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    image_cache={};by_phase={};rows=[]
    for frame in frames:
        path=FOLDER/'camera'/frame['image'];blob=path.read_bytes();sha=hashlib.sha256(blob).hexdigest()
        if sha not in image_cache:
            im=cv2.imdecode(np.frombuffer(blob,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            image_cache[sha]=(dict(decoded=False) if im is None else
                dict(decoded=True,minimum=int(im.min()),maximum=int(im.max()),mean=float(im.mean()),
                     std=float(im.std()),fraction_at_or_below_2=float(np.mean(im<=2))))
        quality=image_cache[sha]
        black=(not quality['decoded'] or quality['maximum']<=2)
        phase=frame['phase'];entry=by_phase.setdefault(phase,dict(frames=0,blank=0))
        entry['frames']+=1;entry['blank']+=int(black)
        rows.append(dict(image=frame['image'],phase=phase,blank=black,**quality))
    rotation=[s for s in samples if s['phase']=='stage_4_small_rotation']
    final_hold=[s for s in samples if s['phase']=='stage_2_fixed_hold']
    expected=json.loads((ROOT/'right_hand_rotation_20261022/small_rotation_trial_candidate.json').read_bytes())
    # Reads precede each write. The final rotation target appears in the hold.
    base=np.asarray(expected['base_goal_ticks'])
    observed=np.asarray([s['goal'] for s in rotation]+[final_hold[0]['goal']])
    nominal=np.vstack([base,np.asarray(expected['goal_ticks'])])
    count_ok=observed.shape==nominal.shape and np.array_equal(observed,nominal)
    result=dict(status='completed_motion_and_camera_review',source_trial=FOLDER.name,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        command_sequence_completed=r['status']=='cube_small_rotation_path_complete',
        exact_rotation_targets_confirmed=bool(count_ok),rotation_samples=len(rotation),
        rotation_elapsed_seconds=rotation[-1]['t']-rotation[0]['t'],
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        cube_removal_confirmed=r.get('cube_removal_confirmed',False),
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(abs(x) for s in samples for x in s['current_ma']),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        phase_samples=dict(Counter(s['phase'] for s in samples)),
        camera_transport_status=r['camera_recording']['status'],camera_frames=len(frames),
        unique_encoded_images=len(image_cache),blank_frames=sum(row['blank'] for row in rows),
        camera_any_signal=not all(row['blank'] for row in rows),
        camera_content_usable=all(by_phase.get(phase,{}).get('frames',0)>0
            and by_phase[phase]['blank']<by_phase[phase]['frames']
            for phase in ('stage_3_loaded_approach','stage_4_small_rotation')),
        camera_content_by_phase=by_phase,
        physical_cube_rotation_verified=False,user_observation_pending=True,
        note='Telemetry verifies commands, not cube motion. Inspect motion images and user observation separately.',
        hardware_commands_sent_by_review=0)
    (FOLDER/'completion_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    (FOLDER/'camera_content_review.json').write_text(json.dumps(dict(frames=rows),indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    for phase in ('preflight','release_contact_confirmation','place_cube','stage_4_small_rotation'):
        group=[row for row in rows if row['phase']==phase]
        if group:print(phase,group[0],group[-1])


if __name__=='__main__':main()
