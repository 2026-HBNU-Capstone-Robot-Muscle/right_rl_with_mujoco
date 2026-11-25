"""Compare saved-image corner methods against the user's marker size range."""
import json
from pathlib import Path
import cv2,numpy as np
from calibrate_red_green_from_face_outline_20261026 import measure,ANNOTATIONS
from center_marker_cube_pose_20261026 import estimate
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_red_green_20261026_215059_362527'
def main():
    source=json.loads((FOLDER/'all_face_detections.json').read_bytes())['frames'];cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);rows=[];saved=[]
    for item in [v for v in source if v['image'] in ('original_00.png','original_04.png','original_05.png')]:
        im=cv2.imdecode(np.frombuffer((FOLDER/item['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        if item['image']=='original_00.png':
            for label,(x0,y0,x1,y1) in dict(red=(945,375,1065,455),green=(940,535,1050,610)).items():
                zoom=cv2.resize(im[y0:y1,x0:x1],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST);ok,png=cv2.imencode('.png',zoom);(FOLDER/f'audit_{label}_raw_zoom.png').write_bytes(png.tobytes())
        for name,refinement,window in (('none',cv2.aruco.CORNER_REFINE_NONE,3),('subpix2',cv2.aruco.CORNER_REFINE_SUBPIX,2),('subpix3',cv2.aruco.CORNER_REFINE_SUBPIX,3),('subpix5',cv2.aruco.CORNER_REFINE_SUBPIX,5),('contour',cv2.aruco.CORNER_REFINE_CONTOUR,3),('apriltag',cv2.aruco.CORNER_REFINE_APRILTAG,3)):
            for constant in (3,7,11):
                p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=refinement;p.cornerRefinementWinSize=window;p.adaptiveThreshConstant=constant;p.adaptiveThreshWinSizeMax=53;p.minMarkerPerimeterRate=.015;corners,ids,_=cv2.aruco.ArucoDetector(dictionary,p).detectMarkers(gray);ids=[] if ids is None else ids.ravel().tolist();selected={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners) if i in (2,5,41)}
                if '2' not in selected or '5' not in selected:continue
                pose=estimate(selected,K,dist);sizes={}
                for face in (2,5):
                    measured=measure(ANNOTATIONS[face],selected[str(face)],face,K=K,dist=dist);sizes[str(face)]=dict(side_mean_mm=measured['side_mean_mm'],square_axis_angle_deg=measured['square_axis_angle_deg'])
                row=dict(image=item['image'],profile=item['profile'],method=name,threshold_constant=constant,corners=selected,pose=pose,face_outline_sizes=sizes);rows.append(row)
                if item['image']=='original_00.png' and constant==7:
                    show=im.copy()
                    for mid,quad in selected.items():
                        cv2.polylines(show,[np.rint(quad).astype(np.int32)],True,(255,0,255),1)
                    crop=show[365:620,935:1070];ok,png=cv2.imencode('.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST));(FOLDER/f'audit_{name}_corners.png').write_bytes(png.tobytes())
    report=dict(status='saved_image_corner_method_audit',user_marker2_side_range_mm=[10.5,10.7],user_green_marker_side_mm_approx=10.7,methods=rows,annotation_size_not_ground_truth=True,live_detector_unchanged=True,real_devices_opened=False,hardware_execution_allowed=False)
    (FOLDER/'corner_refinement_audit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    for method in sorted({r['method'] for r in rows}):
        group=[r for r in rows if r['method']==method];print(json.dumps(dict(method=method,frames_with_pair=len(group),red_size_mm=[round(v['face_outline_sizes']['2']['side_mean_mm'],3) for v in group],green_size_mm=[round(v['face_outline_sizes']['5']['side_mean_mm'],3) for v in group],rms_px=[round(v['pose']['reprojection_rms_px'],3) for v in group],accepted=sum(v['pose']['full_orientation_observable'] for v in group))),flush=True)
if __name__=='__main__':main()
