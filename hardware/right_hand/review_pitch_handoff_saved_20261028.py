"""Inspect saved camera and encoder evidence. Does not open devices."""
import hashlib
import json
from pathlib import Path
from collections import Counter
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'pitch_handoff_observe_20261028_221727_181336'
OUT = RUN / 'offline_review'

def sheet(frames, indices, name):
    canvas = Image.new('RGB', (1280, 4 * 212), '#151515')
    draw = ImageDraw.Draw(canvas)
    for cell, index in enumerate(indices):
        row = frames[index]
        x, y = (cell % 4) * 320, (cell // 4) * 212
        with Image.open(RUN / 'camera' / row['image']) as source:
            canvas.paste(source.resize((320, 180)), (x, y))
        draw.text((x+5, y+182), f"{index}  t={row['camera_read_monotonic']-frames[0]['camera_read_monotonic']:.2f}s", fill='white')
        draw.text((x+5, y+197), row['phase'].replace('stage_', '')[:39], fill='white')
    canvas.save(OUT / name)

def main():
    OUT.mkdir(exist_ok=True)
    result = json.loads((RUN / 'result.json').read_bytes())
    frames = json.loads((RUN / 'camera/frames.json').read_bytes())['frames']
    samples = result['samples']
    sheet(frames, [760,788,800,811,812,820,830,840,850,860,880,900,950,1000,1100,1200], 'early_placement_and_pitch.jpg')
    sheet(frames, [1250,1350,1450,1523,1557,1558,1640,1735,1840,1911,1935,1948,1955,1970,1990,1996], 'pitch_transfer_and_removal.jpg')
    hashes = [hashlib.sha256((RUN/'camera'/f['image']).read_bytes()).hexdigest() for f in frames]
    duplicate_pairs = sum(a == b for a,b in zip(hashes, hashes[1:]))
    keys = list(result)
    details = {k: result[k] for k in keys if 'recover' in k or 'read_fail' in k or 'enable_goal' in k}
    details.update(result_keys=keys, first_sample=samples[0], last_sample=samples[-1],
                   write_event_count=len(result.get('write_events', [])),
                   first_write_events=result.get('write_events', [])[:3],
                   unique_frame_hashes=len(set(hashes)), exact_consecutive_duplicate_images=duplicate_pairs,
                   source_result_sha256=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest(),
                   generated_contact_sheets=['early_placement_and_pitch.jpg', 'pitch_transfer_and_removal.jpg'],
                   robot_commands=0, original_recording_modified=False)
    (OUT/'inspection_details.json').write_text(json.dumps(details, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(details, ensure_ascii=False))

if __name__ == '__main__':
    main()
