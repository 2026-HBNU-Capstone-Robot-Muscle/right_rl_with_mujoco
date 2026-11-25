"""Scan saved frames for pose observability; never opens a camera or motors."""
from pathlib import Path
import json
import time
from collections import Counter,defaultdict
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def main():
    run=ROOT/'middle_clearance_probe_20261106_230902_998326';folder=run/'camera'
    manifest=json.loads((folder/'frames.json').read_text(encoding='utf-8'))
    actual=json.loads((run/'result.json').read_text(encoding='utf-8'))
    minimum=float(actual['empty_start_marker_screen']['minimum_marker_edge_px'])
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),cv2.aruco.DetectorParameters())
    cv2.setNumThreads(1);rows=[];counts=Counter();phases=defaultdict(Counter);began=time.perf_counter()
    for index,item in enumerate(manifest['frames']):
        path=folder/item['image'];image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_GRAYSCALE)
        if image is None:
            rows.append({'index':index,'image':item['image'],'read_error':True});continue
        corners,ids,_=detector.detectMarkers(image);raw=[];qualified=[];edges={}
        if ids is not None:
            for points,mid in zip(corners,ids.ravel()):
                mid=int(mid);p=points.reshape(4,2);edge=float(np.min(np.linalg.norm(p-np.roll(p,1,axis=0),axis=1)))
                raw.append(mid);edges[str(mid)]=edge
                if edge>=minimum:qualified.append(mid)
        cube=any(mid in range(6) for mid in qualified);anchor=any(mid in (40,41) for mid in qualified)
        phase=item.get('phase','unknown');phases[phase]['frames']+=1;phases[phase]['cube']+=cube
        phases[phase]['anchor']+=anchor;phases[phase]['co_visible']+=cube and anchor
        for mid in qualified:counts[mid]+=1
        rows.append({'index':index,'image':item['image'],'phase':phase,
                     'recorded_time':item['camera_read_monotonic'],'decoded_ids':raw,'qualified_ids':qualified,
                     'minimum_edges_px':edges,'cube_visible':cube,'anchor_visible':anchor,'co_visible':cube and anchor})
        if (index+1)%500==0:print(json.dumps({'frames_read':index+1,'seconds':time.perf_counter()-began}),flush=True)
    report={'status':'recorded_frame_marker_visibility_audit','frames_in_manifest':len(manifest['frames']),
            'frames_read':len(rows),'minimum_edge_px_from_original_run':minimum,'dictionary':'DICT_4X4_50',
            'qualified_marker_frame_counts':dict(counts),'phase_counts':dict(phases),'frames':rows,
            'co_visibility_is_not_full_metric_pose_registration':True,
            'saved_frame_rate_is_not_a_measurement_of_live_camera_policy_rate':True,
            'pixel_marker_observations_not_converted_to_cube_mm_or_angles':True,
            'all_saved_frames_scanned_not_all_unsaved_video_frames':True,
            'devices_opened':0,'new_images_requested':False,'elapsed_seconds':time.perf_counter()-began}
    (HERE/'recorded_pose_visibility.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({key:value for key,value in report.items() if key!='frames'}),flush=True)


if __name__=='__main__':main()
