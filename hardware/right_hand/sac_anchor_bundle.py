"""Offline joint ID40/41 fit from several camera views, NumPy/SciPy only.

No model/cube target is used in the objective. Intrinsics and physical marker
sizes are fixed. Result is a calibration candidate, never installed implicitly.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent

def square(side):
    h=side/2
    return np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])

def project(points,rvec,tvec,K,dist):
    camera=points@Rotation.from_rotvec(rvec).as_matrix().T+tvec
    if np.any(camera[:,2]<=0):
        # Optimization residual remains finite; candidate is rejected later.
        depth=np.maximum(camera[:,2],1e-6)
    else:depth=camera[:,2]
    x,y=(camera[:,:2]/depth[:,None]).T;r2=x*x+y*y
    coefficients=np.zeros(8);coefficients[:len(dist)]=dist
    k1,k2,p1,p2,k3,k4,k5,k6=coefficients
    radial=(1+k1*r2+k2*r2*r2+k3*r2**3)/(1+k4*r2+k5*r2*r2+k6*r2**3)
    xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)
    yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y
    return np.column_stack((K[0,0]*xd+K[0,2],K[1,1]*yd+K[1,2]))

def fit(report):
    frames=report['frames'];K=np.asarray(report['intrinsic']);dist=np.asarray(report['distortion'])
    if len(dist) not in (4,5,8):raise ValueError('Unsupported distortion coefficient count')
    if len(frames)<3:raise ValueError('Need three or more paired frames')
    objects={k:square(v) for k,v in report['marker_sides_m'].items()}
    if set(objects)!= {'40','41'}:raise ValueError('Require ID40/41 sizes')
    starts=[];cameras=[]
    for frame in frames:
        markers=frame['markers'];a,h=markers['41'],markers['40']
        Ra=Rotation.from_rotvec(a['rvec']).as_matrix();Rh=Rotation.from_rotvec(h['rvec']).as_matrix()
        starts.append(np.r_[Rotation.from_matrix(Ra.T@Rh).as_rotvec(),Ra.T@(np.asarray(h['tvec_m'])-a['tvec_m'])])
        cameras.extend([*a['rvec'],*a['tvec_m']])
    observed=np.array([[frame['markers'][str(i)]['corners_px'] for i in (41,40)] for frame in frames])
    def predictions(parameters):
        relR=Rotation.from_rotvec(parameters[:3]).as_matrix();relt=parameters[3:6]
        hand=objects['40']@relR.T+relt
        return np.array([[project(points,c[:3],c[3:],K,dist) for points in (objects['41'],hand)]
                         for c in parameters[6:].reshape(-1,6)])
    def residual(parameters):return (predictions(parameters)-observed).ravel()
    solutions=[]
    seeds=sorted(set((0,len(frames)//2,len(frames)-1)))
    for seed in seeds:
        initial=np.r_[starts[seed],cameras]
        solution=least_squares(residual,initial,loss='soft_l1',f_scale=1.,x_scale='jac',max_nfev=150)
        errors=np.linalg.norm(predictions(solution.x)-observed,axis=-1)
        solutions.append((float(np.sqrt(np.mean(errors**2))),solution,errors))
    rms,sol,errors=min(solutions,key=lambda v:v[0])
    rotations=[Rotation.from_rotvec(v[:3]).as_matrix() for v in sol.x[6:].reshape(-1,6)]
    # Normal diversity, not image-plane roll, constrains planar depth ambiguity.
    normals=np.array([r[:,2] for r in rotations]);maximum=0.
    for i in range(len(normals)):
        for j in range(i):
            if frames[i]['view']==frames[j]['view']:continue
            maximum=max(maximum,float(np.degrees(np.arccos(np.clip(normals[i]@normals[j],-1,1)))))
    per_view={str(v):float(np.sqrt(np.mean(errors[[i for i,f in enumerate(frames) if f['view']==v]]**2))) for v in sorted(set(f['view'] for f in frames))}
    reasons=[]
    if maximum<12:reasons.append('insufficient_camera_normal_diversity_below_12deg')
    if rms>2 or any(v>3 for v in per_view.values()):reasons.append('image_fit_error_too_large')
    if not sol.success:reasons.append('optimizer_did_not_converge')
    hand=objects['40']@Rotation.from_rotvec(sol.x[:3]).as_matrix().T+sol.x[3:6]
    for camera in sol.x[6:].reshape(-1,6):
        R=Rotation.from_rotvec(camera[:3]).as_matrix()
        if min(np.min((points@R.T+camera[3:])[:,2]) for points in (objects['41'],hand))<=0:
            reasons.append('points_behind_camera');break
    return dict(status='offline_anchor_bundle_candidate' if not reasons else 'offline_anchor_bundle_rejected',
                diagnostic_fit_passed=not reasons,hardware_authorized=False,motor_commands_sent=0,
                relative_rotation_41_to_40=Rotation.from_rotvec(sol.x[:3]).as_matrix().tolist(),
                marker40_origin_in_marker41_m=sol.x[3:6].tolist(),pixel_rms=rms,per_view_pixel_rms=per_view,
                maximum_camera_normal_separation_deg=maximum,rejection_reasons=reasons,
                fit_seed_pixel_rms=[s[0] for s in solutions],frames=len(frames),
                limitations=['Physical sizes and intrinsics treated as fixed, with unquantified measurement uncertainty.',
                             'Camera diversity/image fit is a diagnostic, not full metric calibration validation.',
                             'Marker-to-hand-model transform is not calibrated by this fit.',
                             'No hardware runtime configuration is automatically changed.'])

def main():
    p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('--output',default='sac_anchor_bundle_result.json')
    args=p.parse_args();path=args.input if args.input.is_absolute() else ROOT/args.input
    report=json.loads(path.read_text(encoding='utf-8'));result=fit(report)
    result['source']=str(path)
    (ROOT/args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
