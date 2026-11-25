"""Compact read-only inspection of the latest operator capture."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    folders=sorted((p for p in ROOT.glob('cube_pose_readonly_2*') if p.is_dir()),reverse=True)
    folder=folders[0];data=json.loads((folder/'result.json').read_bytes())
    fields=('timestamp','status','error','failed_phase','phase','motor_connection_verified',
        'capture_elapsed_seconds','camera_restoration_verified','usable_pair_count',
        'usable_pair_indices','completed_encoder_pair_count','all_completed_pairs_have_off_endpoints',
        'preflight_marker_readiness')
    print('FOLDER',folder.name)
    print(json.dumps({k:data.get(k) for k in fields},ensure_ascii=False,indent=2))
    for row in data.get('pairs',[]):
        print(json.dumps(dict(image=row.get('image'),status=row['status'],
            ids=row.get('markers',{}).get('ids'),edges=row.get('markers',{}).get('minimum_edge_px'),
            drift=row.get('maximum_drift_ticks'),usable=row.get('usable_paired_observation'),
            before=row.get('before',{}).get('snapshot',{}).get('position_ticks'),
            after=row.get('after',{}).get('snapshot',{}).get('position_ticks'))))
    worker=data.get('camera_worker',{})
    print('CAMERA',json.dumps({k:worker.get(k) for k in ('status','frames_sent','error','last_marker_readiness','restoration')},ensure_ascii=False))


if __name__=='__main__':main()
