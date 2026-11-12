import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    for name in ('leap_sequence_flip_20261011_051350_713486.json','leap_sequence_flip_20261011_051518_433595.json'):
        r=json.loads((ROOT/name).read_bytes());print(json.dumps(dict(file=name,status=r['status'],source_sha256=r['source_sha256'],sample_count=len(r['samples']),phase_names=list(dict.fromkeys(s['phase'] for s in r['samples'])),vision_folder=r.get('vision_recording',{}).get('folder'),placement=r.get('cube_placement_camera_snapshot')),ensure_ascii=False))
    p=ROOT/'sequence_vision_20261011_051517_532925/result.json';v=json.loads(p.read_bytes());print('VISION KEYS',list(v));print('PHOTOS',json.dumps({k:v.get(k) for k in ('phase_end_images','saved_images')},ensure_ascii=False));print('FIRSTFRAME',json.dumps(v['frames'][0],ensure_ascii=False));print('DIRECTORY',[(q.name,q.stat().st_size) for q in p.parent.iterdir() if q.is_file()])
if __name__=='__main__':main()
