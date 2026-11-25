"""Real-time timestamped camera exports; no speed-up or generated imagery."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent;F=ROOT/'adaptive_yaw_loaded_20261029_160451_416563';OUT=F/'offline_review';OUT.mkdir(exist_ok=True)
frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
assert all(b['camera_read_monotonic']>a['camera_read_monotonic'] for a,b in zip(frames,frames[1:]))
def encode(rows,name):
    lines=['ffconcat version 1.0']
    for i,row in enumerate(rows):
        path=F/'camera'/row['image'];assert path.is_file() and "'" not in str(path)
        lines.extend(["file '"+str(path)+"'",'option framerate 1000'])
        duration=rows[i+1]['camera_read_monotonic']-row['camera_read_monotonic'] if i+1<len(rows) else .125
        lines.append(f'duration {duration:.6f}')
    lines.extend(["file '"+str(F/'camera'/rows[-1]['image'])+"'",'option framerate 1000'])
    manifest=OUT/(name+'.ffconcat');manifest.write_text('\n'.join(lines)+'\n',encoding='utf-8');output=OUT/(name+'.mp4')
    if output.exists():raise RuntimeError('Refuse to overwrite video')
    subprocess.run(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(manifest),
        '-r','10','-fps_mode','cfr','-c:v','libx264','-preset','veryfast','-crf','22','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
    probe=json.loads(subprocess.run(['/usr/bin/ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=codec_name,width,height,nb_frames:format=duration','-of','json',str(output)],capture_output=True,text=True,check=True).stdout)
    expected=rows[-1]['camera_read_monotonic']-rows[0]['camera_read_monotonic']+.125
    actual=float(probe['format']['duration']);assert abs(expected-actual)<.3
    return dict(video=output.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(output.read_bytes()).hexdigest(),playback_speed=1,
        source_frames=len(rows),source_duration_s=expected,probe=probe,original_images_modified=False,resampled_to10fps_with_frame_repetition_not_interpolation=True)
def main():
    exports=[encode(frames,'full_run_realtime')]
    start=next(f['camera_read_monotonic'] for f in frames if f['phase']=='loaded_countdown')
    end=max(f['camera_read_monotonic'] for f in frames if f['phase']=='stage_10_remove_hold')
    exports.append(encode([f for f in frames if start<=f['camera_read_monotonic']<=end],'grasp_turn_open_realtime'))
    (OUT/'video_exports.json').write_text(json.dumps(dict(source=F.name,exports=exports,motor_writes=0),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(exports),flush=True)
if __name__=='__main__':main()
