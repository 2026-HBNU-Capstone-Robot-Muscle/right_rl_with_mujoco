"""Encode the complete saved rotation interval at its recorded wall-clock pace."""
import hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
if len(sys.argv)>1:SOURCE=(ROOT/sys.argv[1]).resolve()
if not SOURCE.is_relative_to(ROOT):raise ValueError('Saved source must be inside outputs')
OUT=SOURCE/'visual_review'
def main():
    index=json.loads((OUT/'video_source_index.json').read_bytes());rows=index['frames']
    lines=['ffconcat version 1.0'];hashes=[]
    for i,row in enumerate(rows):
        path=SOURCE/'camera'/row['image'];name=path.as_posix()
        if "'" in name or '\n' in name:raise ValueError('Unsupported path in concat file')
        lines.append("file '"+name+"'")
        if i+1<len(rows):lines.append('duration '+format(rows[i+1]['camera_read_monotonic']-row['camera_read_monotonic'],'.9f'))
        hashes.append(dict(image=row['image'],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    concat=OUT/'recorded_timing.ffconcat';concat.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    video=OUT/'actual_forward_flip_realtime.mp4'
    if video.exists():raise ValueError('Do not overwrite an existing exported video')
    command=['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(concat),
             '-vf','fps=30','-c:v','libx264','-threads','4','-preset','fast','-crf','18',
             '-pix_fmt','yuv420p','-movflags','+faststart','-an',str(video)]
    subprocess.run(command,check=True,timeout=180)
    probe=json.loads(subprocess.check_output(['/usr/bin/ffprobe','-v','error','-show_entries',
        'format=duration:stream=codec_name,width,height,nb_frames,avg_frame_rate','-of','json',str(video)],text=True))
    assert abs(float(probe['format']['duration'])-index['recorded_span_seconds'])<.2
    assert probe['streams'][0]['codec_name']=='h264'
    assert (probe['streams'][0]['width'],probe['streams'][0]['height'])==(1280,720)
    review=dict(status='exported_and_decode_probed',source_index_sha256=hashlib.sha256((OUT/'video_source_index.json').read_bytes()).hexdigest(),
        source_frames=hashes,source_images_modified=False,content_cropped_or_warped=False,
        playback='recorded timestamp pace, 30fps with repeated original stills; no interpolation',
        recorded_span_seconds=index['recorded_span_seconds'],maximum_recorded_frame_gap_seconds=index['maximum_recorded_frame_gap_seconds'],
        encoded_file=video.name,encoded_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
        probe=probe,physical_precision_angle_claim=False,hardware_commands_sent=0)
    (OUT/'video_export_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in review.items() if k!='source_frames'}),flush=True)
if __name__=='__main__':main()
