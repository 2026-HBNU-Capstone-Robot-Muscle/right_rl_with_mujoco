"""Inspect the saved camera-only burst without reopening any device."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    pointer=ROOT/'leap_cube_visibility_result.json';r=json.loads(pointer.read_bytes())
    matches=[p for p in ROOT.glob('cube_visibility_2026*') if (p/'result.json').is_file() and hashlib.sha256((p/'result.json').read_bytes()).digest()==hashlib.sha256(pointer.read_bytes()).digest()]
    if not matches:raise RuntimeError('No burst directory matches the latest result')
    folder=sorted(matches)[-1];profiles=[]
    for p in r.get('profiles',[]):
        frames=p.get('frames',[]);pairs=[v for v in frames if v.get('anchor_and_green_visible')];edges=[v['green_short_edge_px'] for v in pairs]
        sharp=[v['green_roi_sharpness'] for v in pairs if v.get('green_roi_sharpness') is not None]
        profiles.append(dict(name=p['name'],settings=p.get('settings'),saved_frames=len(frames),pair_decoded_frames=len(pairs),preferred_40px_frames=sum(v.get('advisory_ready',False) for v in frames),
            green_edge_min_median_max_px=[float(np.min(edges)),float(np.median(edges)),float(np.max(edges))] if edges else None,
            median_sharpness=float(np.median(sharp)) if sharp else None,median_clipped_white_fraction=float(np.median([v['white_clipped_fraction'] for v in frames])) if frames else None,
            first_image=frames[0]['image'] if frames else None))
    report=dict(source_folder=folder.name,status=r['status'],source_sha256=hashlib.sha256(pointer.read_bytes()).hexdigest(),motors_accessed=r['motors_accessed'],motor_commands_sent=r['motor_commands_sent'],
        saved_frames=r.get('saved_frames'),profiles=profiles,restoration=r.get('camera_restoration'),focus=r.get('focus_readbacks'),
        physical_yaw_verified=False,preferred_pixel_size_is_advisory_not_failure=True)
    (folder/'saved_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
