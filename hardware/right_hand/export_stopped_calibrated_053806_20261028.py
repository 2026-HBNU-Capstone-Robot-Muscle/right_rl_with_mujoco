"""Faithful timestamp-based H264 exports from recorded real camera frames."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'calibrated_quarter_trial_20261028_053806_737439'
OUT=ROOT/'right_hand_rotation_20261028/stopped_calibrated_053806';OUT.mkdir(parents=True,exist_ok=True)
frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
assert all(b['camera_read_monotonic']>a['camera_read_monotonic'] for a,b in zip(frames,frames[1:]))

def manifest(rows,name):
    lines=['ffconcat version 1.0']
    for i,row in enumerate(rows):
        path=FOLDER/'camera'/row['image'];assert path.is_file() and "'" not in str(path)
        lines.extend(["file '"+str(path)+"'",'option framerate 1000'])
        duration=(rows[i+1]['camera_read_monotonic']-row['camera_read_monotonic']) if i+1<len(rows) else .125
        lines.append(f'duration {duration:.6f}')
    lines.extend(["file '"+str(FOLDER/'camera'/rows[-1]['image'])+"'",'option framerate 1000'])
    path=OUT/name;path.write_text('\n'.join(lines)+'\n',encoding='utf-8');return path

def encode(rows,name,speed):
    source=manifest(rows,name+'.ffconcat');output=OUT/(name+'.mp4')
    filters=f'setpts=PTS/{speed}'
    if speed!=1:filters+=f",drawtext=text='{speed}x playback':fontsize=26:fontcolor=white:box=1:boxcolor=black@0.65:x=16:y=16"
    command=['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(source),
        '-vf',filters,'-r','10','-fps_mode','cfr','-c:v','libx264','-preset','veryfast','-crf','22','-threads','2',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
    subprocess.run(command,check=True)
    probe=subprocess.run(['/usr/bin/ffprobe','-v','error','-select_streams','v:0','-show_entries',
        'stream=codec_name,width,height,nb_frames:format=duration','-of','json',str(output)],capture_output=True,text=True,check=True)
    meta=json.loads(probe.stdout);record=dict(video=output.relative_to(ROOT).as_posix(),playback_speed=speed,
        source_frames=len(rows),source_duration_s=rows[-1]['camera_read_monotonic']-rows[0]['camera_read_monotonic']+.125,
        probe=meta,no_generated_frames=True,recorded_frames_resampled_to10fps=True,robot_commands=0)
    print(json.dumps(record),flush=True);return record

parser=argparse.ArgumentParser();parser.add_argument('--preview-only',action='store_true');args=parser.parse_args()
if args.preview_only:
    full_path=OUT/'full_run_realtime.mp4'
    probe=subprocess.run(['/usr/bin/ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=codec_name,width,height,nb_frames:format=duration','-of','json',str(full_path)],capture_output=True,text=True,check=True)
    full=dict(video=full_path.relative_to(ROOT).as_posix(),playback_speed=1,source_frames=len(frames),source_duration_s=frames[-1]['camera_read_monotonic']-frames[0]['camera_read_monotonic']+.125,probe=json.loads(probe.stdout),no_generated_frames=True,recorded_frames_resampled_to10fps=True,robot_commands=0)
else:full=encode(frames,'full_run_realtime',1)
start=next(f['camera_read_monotonic'] for f in frames if f['phase']=='loaded_countdown')
end=max(f['camera_read_monotonic'] for f in frames if f['phase']=='stage_8_horizontal_hold')
segment=[f for f in frames if start<=f['camera_read_monotonic']<=end]
preview=encode(segment,'cube_rotation_preview_4x',4)
(OUT/'video_exports.json').write_text(json.dumps(dict(source=FOLDER.name,exports=[full,preview],
    original_frames_modified=False,physical_angle_not_asserted_by_export=True,trial_stopped_before_open=True),indent=2)+'\n',encoding='utf-8')
