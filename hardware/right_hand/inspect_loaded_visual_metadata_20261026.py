"""Find saved camera timestamps and local encoding tools; no devices."""
import json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
def main():
    r=json.loads((FOLDER/'result.json').read_bytes());f=json.loads((FOLDER/'camera/frames.json').read_bytes())
    print('FRAME_KEYS',list(f));print('FIRST_FRAME',json.dumps(f['frames'][0]));print('LAST_FRAME',json.dumps(f['frames'][-1]))
    print('WRITE_FIRST_LAST',json.dumps(r['write_events'][:3]+r['write_events'][-3:]))
    print('COMMAND_EVENTS',json.dumps([v for v in r['write_events'] if v.get('address')==116][:3]))
    print('ROTATION_PHOTO',json.dumps(r.get('rotation_observation_photo')))
    print('ENCODERS',dict(ffmpeg=shutil.which('ffmpeg'),ffprobe=shutil.which('ffprobe')))
    print('SAVED_TRACKING_HELPERS',[p.name for p in ROOT.glob('*.py') if any(v in p.name for v in ('marker_layout','aruco','orientation','cube_motion','visual_review','clip','video_review'))])
if __name__=='__main__':main()
