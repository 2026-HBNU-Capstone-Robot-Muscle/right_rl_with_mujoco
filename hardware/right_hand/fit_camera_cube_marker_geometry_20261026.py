"""Compare observed marker sizes/angles with a fixed 55mm cube, OFFLINE only."""
import json
from pathlib import Path
import sys,numpy as np
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_visibility_20261026_192140_408522'
def main():
    global FOLDER
    if '--folder' in sys.argv:
        FOLDER=(ROOT/sys.argv[sys.argv.index('--folder')+1]).resolve();FOLDER.relative_to(ROOT)
    faces=(2,5) if '--red-green' in sys.argv else (3,5)
    if '--export-input' in sys.argv:
        import cv2
        from center_marker_cube_pose_20261026 import FACE_BASES
        report=json.loads((FOLDER/'spin_invariant_pose_review.json').read_bytes());detections=json.loads((FOLDER/'all_face_detections.json').read_bytes())['frames'];cal=np.load(ROOT/'camera1_720.npz');usable=[]
        for row in report['frames']:
            p=row['pose']
            eligible=p.get('full_orientation_observable') or ('--include-uncertain' in sys.argv and p.get('status')=='multi_face_pose_uncertain' and p.get('reprojection_rms_px',99)<6 and p.get('distinct_rotation_rival_error_gap_px',0)>.25)
            if eligible and p['visible_face_ids']==list(faces) and row['profile'] in ('original','manual_minus5','manual_minus6'):
                frame=next(v for v in detections if v['image']==row['image']);usable.append((frame,p))
        base=usable[0][1]
        data=dict(K=cal['K'].tolist(),dist=cal['dist'].ravel().tolist(),usable=usable,bases=FACE_BASES,faces=faces,base_rvec=cv2.Rodrigues(np.asarray(base['R_cube_camera']))[0].ravel().tolist())
        (FOLDER/'marker_geometry_fit_input.json').write_text(json.dumps(data)+'\n',encoding='utf-8');print('Exported',len(usable),'saved frames; no device access.');return
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    data=json.loads((FOLDER/'marker_geometry_fit_input.json').read_bytes());K=np.asarray(data['K']);dist=np.asarray(data['dist']);usable=data['usable'];FACE_BASES=data['bases'];faces=tuple(data.get('faces',(3,5)))
    def project(obj,rv,tv):
        pc=obj@Rotation.from_rotvec(rv).as_matrix().T+tv;x=pc[:,0]/pc[:,2];y=pc[:,1]/pc[:,2];r2=x*x+y*y;k1,k2,p1,p2,k3=dist[:5];scale=1+k1*r2+k2*r2**2+k3*r2**3
        xd=x*scale+2*p1*x*y+p2*(r2+2*x*x);yd=y*scale+p1*(r2+2*y*y)+2*p2*x*y
        return np.column_stack([K[0,0]*xd+K[0,2],K[1,1]*yd+K[1,2]])
    base=usable[0][1];turns=base['marker_quarter_rotations'];rv=np.asarray(data['base_rvec']);tv=np.asarray(base['cube_center_in_camera_m'])
    def points(face,size_mm,beta):
        n,u,v=(np.asarray(q,dtype=float) for q in FACE_BASES[face])
        for _ in range(turns[str(face)]):u,v=v,-u
        U=np.cos(beta)*u+np.sin(beta)*v;V=-np.sin(beta)*u+np.cos(beta)*v;c=n*.0275;s=size_mm*.0005
        return np.asarray([c-s*U+s*V,c+s*U+s*V,c+s*U-s*V,c-s*U-s*V])
    output=[]
    for mode in ('fixed_size_angle','size_only','size_and_angle','first_face_size_and_angles'):
        start=np.r_[rv,tv,[10.7,10.7,0.,0.]];low=np.r_[[-6.3]*3,[-1.,-1.,.05],[9.,9.,-.55,-.55]];high=np.r_[[6.3]*3,[1.,1.,1.5],[12.5,12.5,.55,.55]]
        active=np.asarray(list(range(6))+([6] if mode=='first_face_size_and_angles' else ([6,7] if mode!='fixed_size_angle' else []))+([8,9] if mode!='size_only' else []))
        def residual(value,prior=True):
            p=start.copy();p[active]=value;obj=np.concatenate([points(f,p[6+i],p[8+i]) for i,f in enumerate(faces)]);pred=project(obj,p[:3],p[3:6]);rows=[]
            for frame,_ in usable:rows.extend((pred-np.concatenate([np.asarray(frame['corners'][str(f)]) for f in faces])).ravel())
            if prior:rows.extend((p[6:8]-10.7)/.5);rows.extend(p[8:]/.15)
            return np.asarray(rows)
        fit=least_squares(residual,start[active],bounds=(low[active],high[active]),loss='soft_l1',f_scale=1.,max_nfev=400);p=start.copy();p[active]=fit.x;R=Rotation.from_rotvec(p[:3]).as_matrix();errors=residual(fit.x,False).reshape(-1,2)
        output.append(dict(mode=mode,marker_size_mm_by_id={str(f):float(p[6+i]) for i,f in enumerate(faces)},glue_angle_deg_by_id={str(f):float(np.degrees(p[8+i])) for i,f in enumerate(faces)},
            reprojection_rms_px=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),cube_center_in_camera_m=p[3:6].tolist(),R_cube_camera=R.tolist(),
            condition_number=float(np.linalg.cond(fit.jac)),success=bool(fit.success),sizes_not_independently_measured=True))
    result=dict(status='marker_geometry_sensitivity_fit_only',cube_side_mm_fixed=55.,input_frames=[v[0]['image'] for v in usable],face_quarter_rotations=turns,fits=output,
        size_angle_pose_coupling_not_fully_calibrated=True,does_not_override_user_measurements=True,hardware_execution_allowed=False,real_devices_opened=False)
    (FOLDER/'marker_geometry_sensitivity.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
