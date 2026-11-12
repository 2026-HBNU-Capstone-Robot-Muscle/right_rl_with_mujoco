"""Numeric pose-quality diagnostics; no motor commands or relaxed pose gates."""
from collections import Counter, defaultdict
import numpy as np
import encoder_policy_shadow as shadow


def frame_metrics(pose,age_ms,detection):
    reason=shadow.pose_rejection(pose,age_ms)
    reproj=float(pose.get('reproj',np.nan)) if pose else np.nan
    return dict(reason=reason,frame_age_ms=float(age_ms),
        reprojection_px=float(reproj) if np.isfinite(reproj) else None,
        n_cube_markers=int(pose.get('n_cube_markers',0)) if pose else 0,
        detected_ids=[int(i) for i in detection.get('ids',[])],
        min_edge_px_by_id=dict(detection.get('min_edge_px_by_id',{})))


def display_line(record):
    error=record['reprojection_px']
    value='missing' if error is None else f'{error:.2f}px'
    return f'IDs {record["detected_ids"]} | cube {record["n_cube_markers"]} | reprojection {value} (limit 3px)'


def summarize(rows):
    reasons=Counter(r['reason'] or 'eligible' for r in rows)
    pairs=defaultdict(list)
    longest=streak=0
    for r in rows:
        streak=streak+1 if r['reason'] is None else 0
        longest=max(longest,streak)
        key=','.join(map(str,sorted(i for i in r['detected_ids'] if 0<=i<6))) or 'none'
        pairs[key].append(r)
    def stats(group):
        errors=[r['reprojection_px'] for r in group if r['reprojection_px'] is not None]
        return dict(frames=len(group),eligible=sum(r['reason'] is None for r in group),
            median_reprojection_px=float(np.median(errors)) if errors else None,
            max_reprojection_px=max(errors) if errors else None)
    return dict(frames=len(rows),eligible_frames=reasons['eligible'],
        eligible_percent=100*reasons['eligible']/max(1,len(rows)),longest_valid_streak=longest,
        reason_counts=dict(reasons),by_cube_ids={k:stats(v) for k,v in sorted(pairs.items())})


def geometry_snapshot(camera,pose):
    """Save corners and board projection for offline diagnosis of bad faces/calibration."""
    result=dict(camera_matrix=camera.K.tolist(),distortion=camera.dist.tolist(),markers=[])
    detector=camera.detector
    if detector.last_ids is None: return result
    for corners,marker in zip(detector.last_corners,detector.last_ids.ravel()):
        item=dict(id=int(marker),corners_px=np.asarray(corners).reshape(4,2).tolist())
        if 0<=int(marker)<6 and pose and 'R_cube' in pose and 't_cube' in pose:
            obj,img=camera.board.matchImagePoints([corners],np.array([[marker]],dtype=np.int32))
            if obj is not None:
                rv=camera.cv.Rodrigues(np.asarray(pose['R_cube']))[0]
                projected,_=camera.cv.projectPoints(obj,rv,np.asarray(pose['t_cube']),camera.K,camera.dist)
                item['projected_px']=projected.reshape(-1,2).tolist()
                item['mean_reprojection_px']=float(np.linalg.norm(projected.reshape(-1,2)-img.reshape(-1,2),axis=1).mean())
        result['markers'].append(item)
    return result
