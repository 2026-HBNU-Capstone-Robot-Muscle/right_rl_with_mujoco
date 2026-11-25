"""Export existing real frames using their saved capture timestamps. No devices."""
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'pitch_handoff_observe_20261028_221727_181336'
OUT = RUN / 'offline_review'

def encode(frames, name, speed):
    manifest = ['ffconcat version 1.0']
    for i,row in enumerate(frames):
        path = RUN/'camera'/row['image']
        assert path.is_file() and "'" not in str(path)
        duration = frames[i+1]['camera_read_monotonic']-row['camera_read_monotonic'] if i+1<len(frames) else .125
        assert duration>0
        manifest.extend(["file '"+str(path)+"'", 'option framerate 1000', f'duration {duration:.6f}'])
    manifest.extend(["file '"+str(RUN/'camera'/frames[-1]['image'])+"'", 'option framerate 1000'])
    source = OUT/(name+'.ffconcat')
    source.write_text('\n'.join(manifest)+'\n', encoding='utf-8')
    target = OUT/(name+'.mp4')
    filters = f'setpts=PTS/{speed}'
    if speed!=1:
        filters += f",drawtext=text='{speed}x playback':fontsize=26:fontcolor=white:box=1:boxcolor=black@0.65:x=16:y=16"
    subprocess.run(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(source),
        '-vf',filters,'-r','10','-fps_mode','cfr','-c:v','libx264','-preset','veryfast','-crf','22','-threads','2',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(target)], check=True)
    result=subprocess.run(['/usr/bin/ffprobe','-v','error','-select_streams','v:0','-show_entries',
        'stream=codec_name,width,height,nb_frames:format=duration','-of','json',str(target)], capture_output=True,text=True,check=True)
    report=dict(video=target.relative_to(ROOT).as_posix(),playback_speed=speed,source_frames=len(frames),
        source_duration_s=frames[-1]['camera_read_monotonic']-frames[0]['camera_read_monotonic']+.125,
        probe=json.loads(result.stdout), source_capture_timestamps_preserved_before_speedup=True,
        recorded_frames_resampled_to10fps=True, original_images_modified=False,
        generated_scene_content=False, robot_commands=0)
    print(json.dumps(report),flush=True)
    return report

def main():
    OUT.mkdir(exist_ok=True)
    frames=json.loads((RUN/'camera/frames.json').read_bytes())['frames']
    assert all(b['camera_read_monotonic']>a['camera_read_monotonic'] for a,b in zip(frames,frames[1:]))
    start=next(f['camera_read_monotonic'] for f in frames if f['phase']=='loaded_countdown')
    end=max(f['camera_read_monotonic'] for f in frames if f['phase']=='transfer_before_remove')
    segment=[f for f in frames if start<=f['camera_read_monotonic']<=end]
    exports=[encode(frames,'full_run_realtime',1),encode(segment,'pitch_and_transfer_preview_4x',4)]
    (OUT/'video_exports.json').write_text(json.dumps(dict(source=RUN.name,exports=exports,
        original_frames_modified=False, earlier_than_recorded_placement_empty_part_retained=True,
        exact90_degrees_not_asserted_by_export=True),indent=2)+'\n',encoding='utf-8')

if __name__ == '__main__':
    main()
