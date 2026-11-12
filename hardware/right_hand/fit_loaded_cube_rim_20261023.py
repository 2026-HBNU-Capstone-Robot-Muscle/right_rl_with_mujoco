"""Constrain the stored top-face pose using its visible full-size rim.

Three rounded outer corners are manually read from the raw pixel grid. Their
uncertainty is explicit; this is an image diagnostic, not a hardware transform.
"""
import json
import argparse
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
source=json.loads((folder/'conditional_cube_registration.json').read_bytes())
f=next(x for x in source['rows'] if x['image']=='frame_01560.jpg')
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
parser=argparse.ArgumentParser();parser.add_argument('--front-corner',action='store_true');parser.add_argument('--rim-only',action='store_true');parser.add_argument('--saved-shape',action='store_true');args=parser.parse_args()
if args.rim_only and not args.front_corner:parser.error('--rim-only needs the non-planar front corner')
rim=np.array([[767.,126.],[850.,225.],[579.,246.]])
if args.front_corner:rim=np.vstack([rim,[593.,464.]])
h=.0107/2;side=.055/2
marker_obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float64)
if args.saved_shape:
    shape=json.loads((folder/'independent_marker_shape_diagnostic.json').read_bytes())
    marker_obj=np.c_[np.array(shape['marker_corners_in_face_plane_mm'])/1000,np.zeros(4)]
square=np.array([[-side,side,0],[side,side,0],[side,-side,0],[-side,-side,0]],np.float64)
marker_img=np.array(f['corners']['5'],np.float64)
fits=[]
for reverse in (False,True):
    sq=square[::-1] if reverse else square
    for spin in range(4):
        outer=np.roll(sq,spin,axis=0)
        rim_obj=outer[:3]
        if args.front_corner:rim_obj=np.vstack([rim_obj,outer[2]-[0,0,.055]])
        op=np.vstack([marker_obj,rim_obj]);ip=np.vstack([marker_img,rim])
        fit_op,fit_ip=(rim_obj,rim) if args.rim_only else (op,ip)
        ok,rv,tv=cv.solvePnP(fit_op,fit_ip,K,dist,flags=cv.SOLVEPNP_SQPNP)
        if not ok:continue
        rv,tv=cv.solvePnPRefineLM(fit_op,fit_ip,K,dist,rv,tv)
        R=cv.Rodrigues(rv)[0];pr=cv.projectPoints(op,rv,tv,K,dist)[0].reshape(-1,2)
        err=np.linalg.norm(pr-ip,axis=1)
        if np.min((op@R.T+tv.ravel())[:,2])<=0 or (R[:,2]@tv.ravel())>=0:continue
        fits.append(dict(reverse=reverse,spin=spin,mean_error_px=float(err.mean()),
            marker_error_px=float(err[:4].mean()),rim_error_px=float(err[4:].mean()),
            R_camera=R.tolist(),top_center_camera_m=tv.ravel().tolist(),
            cube_center_camera_m=(tv.ravel()-R[:,2]*side).tolist(),
            full_top_corners_px=cv.projectPoints(outer,rv,tv,K,dist)[0].reshape(-1,2).tolist(),
            object_points_m=op.tolist(),observed_image_points_px=ip.tolist()))
fits.sort(key=lambda x:x['rim_error_px'] if args.rim_only else x['mean_error_px']);best=fits[0]
im=cv.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
cv.polylines(im,[np.rint(best['full_top_corners_px']).astype(np.int32)],True,(0,255,255),2)
for p in rim:cv.circle(im,tuple(np.rint(p).astype(int)),4,(0,0,255),-1)
cv.putText(im,f"55mm rim + 10.7mm marker fit: {best['mean_error_px']:.2f}px; DIAGNOSTIC ONLY",(8,28),cv.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
tag='full_rim_only' if args.rim_only else ('full_rim' if args.front_corner else 'rim')
if args.saved_shape:tag+='_saved_shape'
cv.imencode('.jpg',im)[1].tofile(str(folder/('cube_'+tag+'_fit.jpg')))
hypotheses=[]
Rc=np.array(best['R_camera']);tc=np.array(best['cube_center_camera_m'])
for old in f['hypotheses']:
    Rcw=np.array(old['world_to_camera_rotation']);tcw=np.array(old['world_to_camera_translation'])
    hypotheses.append(dict(**{k:v for k,v in old.items() if k not in ('cube_center_world_m','cube_rotation_world','error_px')},
        error_px=old['error_px']-f['cube_pose_candidates'][old['cube_solution']]['error_px']+best['mean_error_px'],
        cube_center_world_m=(Rcw.T@(tc-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist()))
out=dict(status='same_frame_rim_constrained_pose_diagnostic',image=f['image'],
         rim_corner_uncertainty_px=3.,hidden_fourth_top_corner_used=False,front_bottom_left_used=args.front_corner,
         marker_side_mm=10.7,cube_side_mm=55,marker_corners_used_in_fit=not args.rim_only,
         independent_saved_closeup_shape_used=args.saved_shape,
         best_fit=best,competing_fits=[{k:v for k,v in x.items() if k not in ('object_points_m','observed_image_points_px')} for x in fits],
         registration_verified=False,hardware_execution_allowed=False,rows=[dict(**{k:v for k,v in f.items() if k!='hypotheses'},hypotheses=hypotheses)])
(folder/(tag+'_constrained_cube_registration.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in best.items() if k not in ('object_points_m','observed_image_points_px')},indent=2))
