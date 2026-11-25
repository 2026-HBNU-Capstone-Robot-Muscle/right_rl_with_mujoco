"""Independent synthetic perspective/distortion validation of marker sizing."""
import json
from pathlib import Path
import cv2,numpy as np
from calibrate_red_green_from_face_outline_20261026 import measure
from center_marker_cube_pose_20261026 import FACE_BASES
ROOT=Path(__file__).resolve().parent
def main():
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];h=.0275;outer={2:np.array([[h,h,h],[h,h,-h],[h,-h,-h],[h,-h,h]]),5:np.array([[h,-h,h],[h,-h,-h],[-h,-h,-h],[-h,-h,h]])};R=np.array([[.109320567657,-.011597551184,-.993938886599],[-.750248640668,-.656904889558,-.074852810575],[-.652055205226,.753884250321,-.080514262434]]);rv=cv2.Rodrigues(R)[0];tv=np.array([.091,.027,.326]);rows=[]
    for face,side,offset in ((2,9.8,np.array([0.,2.5])),(5,10.7,np.array([.9,-.1]))):
        n,U,V=(np.asarray(v,dtype=float) for v in FACE_BASES[face]);angle=np.radians(-86 if face==2 else -90);u=np.cos(angle)*U+np.sin(angle)*V;v=-np.sin(angle)*U+np.cos(angle)*V;c=n*h+(u*offset[0]+v*offset[1])*.001;s=side*.0005;obj=np.array([c-s*u+s*v,c+s*u+s*v,c+s*u-s*v,c-s*u-s*v]);image_outer=cv2.projectPoints(outer[face],rv,tv,K,dist)[0].reshape(-1,2);image_marker=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(-1,2);recovered=measure(image_outer,image_marker,face,K=K,dist=dist);size_error=abs(recovered['side_mean_mm']-side);offset_error=float(np.linalg.norm(np.asarray(recovered['marker_frame_center_offset_mm'])-offset));assert size_error<.001 and offset_error<.001;rows.append(dict(face=face,size_error_mm=size_error,offset_error_mm=offset_error,passed=True))
    result=dict(status='perspective_and_distortion_geometry_checks_passed',checks=rows,uses_independent_3d_projection=True,does_not_validate_real_annotations=True,real_devices_opened=False);(ROOT/'right_hand_rotation_20261026/face_homography_geometry_checks.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
