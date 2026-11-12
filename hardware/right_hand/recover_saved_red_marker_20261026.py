"""Offline saved-frame decode comparison; never changes live motion gates."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from analyze_cube_visibility_pose_20261026 import pose
ROOT=Path(__file__).resolve().parent;RUN=ROOT/'loaded_repeat_20261026_170932_163649';OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    result=json.loads((RUN/'result.json').read_bytes());photo=result['rotation_observation_photo'];when=photo['camera_read_monotonic'];meta=json.loads((RUN/'camera/frames.json').read_bytes())['frames'];chosen=[v for v in meta if abs(v['camera_read_monotonic']-when)<=2][::2];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);rows=[];cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist']
    for item in chosen:
        image=cv2.imdecode(np.frombuffer((RUN/'camera'/item['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);found=[]
        for correction in (.6,1.):
            for constant in (3,7,11):
                p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.cornerRefinementWinSize=3;p.minMarkerPerimeterRate=.015;p.adaptiveThreshConstant=constant;p.errorCorrectionRate=correction;det=cv2.aruco.ArucoDetector(dictionary,p);corners,ids,rejected=det.detectMarkers(gray)
                ids=[] if ids is None else ids.ravel().tolist()
                if 2 not in ids or 41 not in ids:continue
                quad={str(int(i)):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)};solutions=pose(quad['2'],.0107,K,dist)
                source=np.asarray(quad['2'],dtype=np.float32);target=np.array([[0,0],[119,0],[119,119],[0,119]],dtype=np.float32);patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(source,target),(120,120));threshold=cv2.threshold(patch,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)[0]
                bits=np.array([[patch[y*20+5:y*20+15,x*20+5:x*20+15].mean()>threshold for x in range(6)] for y in range(6)]);template=cv2.aruco.generateImageMarker(dictionary,2,120);truth=np.array([[template[y*20+10,x*20+10]>128 for x in range(6)] for y in range(6)]);code_errors=int(np.count_nonzero(bits[1:5,1:5]!=truth[1:5,1:5]));border=np.ones((6,6),dtype=bool);border[1:5,1:5]=False;border_errors=int(np.count_nonzero(bits[border]!=truth[border]))
                center=np.mean(source,axis=0);xx,yy=np.rint(center).astype(int);roi=image[max(0,yy-40):min(720,yy+40),max(0,xx-55):min(1280,xx+55)];hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV);red=((hsv[:,:,0]<12)|(hsv[:,:,0]>170))&(hsv[:,:,1]>110)&(hsv[:,:,2]>60)
                found.append(dict(correction_rate=correction,dictionary_max_correctable_bits=int(dictionary.maxCorrectionBits),threshold_constant=constant,ids=ids,corners=quad,red_plane_candidates=solutions,sampled_code_bit_errors=code_errors,sampled_border_bit_errors=border_errors,red_context_fraction=float(red.mean())))
        if found:
            row=dict(image=item['image'],phase=item['phase'],time_from_end_s=item['camera_read_monotonic']-when,methods=found);rows.append(row);print('FOUND',row['image'],[(m['correction_rate'],m['ids']) for m in found],flush=True)
    report=dict(status='saved_red_top_detection_recovery',source_snapshot=photo,images_checked=len(chosen),frames_with_red_and_anchor=rows,live_marker_detector_unchanged=True,nonzero_bit_correction_is_provisional=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'saved_red_marker_recovery.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('SUMMARY',dict(checked=len(chosen),found=len(rows)),flush=True)
if __name__=='__main__':main()
