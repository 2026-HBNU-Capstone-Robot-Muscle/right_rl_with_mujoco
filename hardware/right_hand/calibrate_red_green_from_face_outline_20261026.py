"""Offline planar homography diagnostic, independent of camera intrinsics.

Outer face corners are analyst annotations, with explicit pixel/edge uncertainty.
This never overwrites measured dimensions or installs a live calibration.
"""
import json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_red_green_20261026_215059_362527'
ANNOTATIONS={2:[[889.,340.],[1095.,365.],[1138.,511.],[912.,489.]],5:[[912.,489.],[1138.,511.],[1073.,642.],[895.,620.]]}
def measure(outer,corners,face,side=55.,K=None,dist=None):
    if K is not None:
        outer=cv2.undistortPoints(np.asarray(outer,np.float64).reshape(-1,1,2),K,dist,P=K).reshape(-1,2)
        corners=cv2.undistortPoints(np.asarray(corners,np.float64).reshape(-1,1,2),K,dist,P=K).reshape(-1,2)
    H=cv2.getPerspectiveTransform(np.asarray(outer,np.float32),np.array([[0,0],[side,0],[side,side],[0,side]],np.float32));xy=cv2.perspectiveTransform(np.asarray(corners,np.float64).reshape(1,4,2),H).reshape(4,2)
    uv=np.column_stack([side/2-xy[:,1],side/2-xy[:,0]]) if face==2 else np.column_stack([xy[:,0]-side/2,side/2-xy[:,1]])
    x=((uv[1]-uv[0])+(uv[2]-uv[3]))/2;y=((uv[0]-uv[3])+(uv[1]-uv[2]))/2;lengths=np.linalg.norm(np.roll(uv,-1,axis=0)-uv,axis=1);U,_,V=np.linalg.svd(np.column_stack([x/np.linalg.norm(x),y/np.linalg.norm(y)]));R=U@V
    if np.linalg.det(R)<0:raise ValueError('Inconsistent corner order')
    angle=float(np.degrees(np.arctan2(R[1,0],R[0,0])));quarter=int(np.rint(angle/90))%4;beta=float((angle-quarter*90+180)%360-180);offset=np.mean(uv,axis=0);mount=R.T@offset
    return dict(side_mean_mm=float(np.mean(lengths)),edge_lengths_mm=lengths.tolist(),face_center_offset_uv_mm=offset.tolist(),marker_frame_center_offset_mm=mount.tolist(),marker_angle_in_face_deg=angle,quarter=quarter,glue_angle_deg=beta,corner_uv_mm=uv.tolist(),square_axis_angle_deg=float(np.degrees(np.arccos(np.clip(x@y/np.linalg.norm(x)/np.linalg.norm(y),-1,1)))))
def main():
    source=json.loads((FOLDER/'all_face_detections.json').read_bytes())['frames'][0];results={};rng=np.random.default_rng(261026);cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist']
    for face,points in ANNOTATIONS.items():
        corners=np.asarray(source['corners'][str(face)]);row=measure(points,corners,face,K=K,dist=dist);trials=[]
        for _ in range(500):
            trial=measure(np.asarray(points)+rng.normal(0,1.5,(4,2)),corners+rng.normal(0,.25,(4,2)),face,K=K,dist=dist);trials.append(trial['side_mean_mm'])
        row.update(outer_face_corners_px=points,annotated_image=source['image'],annotation_pixel_noise_sigma=1.5,corner_noise_sigma=.25,side_annotation_noise_95_percentile_mm=np.percentile(trials,[2.5,97.5]).tolist(),effective_face_span_sensitivity_mm={str(span):measure(points,corners,face,span,K=K,dist=dist)['side_mean_mm'] for span in (53.5,55.,56.5)})
        results[str(face)]=row
    report=dict(status='annotated_outer_face_homography_marker_diagnostic',cube_side_mm=55.,camera_intrinsics_used_only_to_undistort=True,lens_distortion_corrected_before_planar_homography=True,faces=results,
        outer_corners_manually_annotated=True,rounded_bevel_edges_not_calibrated=True,uncertainty_is_annotation_sensitivity_not_absolute_accuracy=True,
        user_marker5_measurement_mm=10.7,does_not_override_user_measurements=True,does_not_install_runtime_configuration=True,hardware_execution_allowed=False,real_devices_opened=False)
    (FOLDER/'outer_face_marker_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
