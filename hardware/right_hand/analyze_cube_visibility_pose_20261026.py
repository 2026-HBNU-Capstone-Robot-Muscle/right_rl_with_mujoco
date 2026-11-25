"""Pose/noise diagnostics from camera-only frames; never a deployment proof."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_visibility_20261026_192140_408522'
def pose(corners,side,K,dist):
    s=side/2.;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64);img=np.asarray(corners,np.float64).reshape(4,2)
    out=cv2.solvePnPGeneric(obj,img,K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE);candidates=[]
    if out[0]:
        for rv,tv in zip(out[1],out[2]):
            R=cv2.Rodrigues(rv)[0];points=(R@obj.T).T+tv.ravel();projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
            error=float(np.sqrt(np.mean(np.sum((projected-img)**2,axis=1))))
            if np.isfinite(points).all() and np.all(points[:,2]>0):candidates.append(dict(R=R.tolist(),t=tv.ravel().tolist(),normal=R[:,2].tolist(),reprojection_rms_px=error))
    return sorted(candidates,key=lambda v:v['reprojection_rms_px'])
def main():
    r=json.loads((FOLDER/'result.json').read_bytes());cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];rows=[]
    for profile in r['profiles']:
        for f in profile.get('frames',[]):
            co=f['corners_px_by_id']
            if not {'5','41'}.issubset(co):continue
            a=pose(co['41'],.027,K,dist);g=pose(co['5'],.0107,K,dist)
            if not a or not g:continue
            A,G=a[0],g[0];Ra=np.asarray(A['R']);ta=np.asarray(A['t']);Rg=np.asarray(G['R']);tg=np.asarray(G['t']);center=tg-Rg[:,2]*.0275
            row=dict(profile=profile['name'],image=f['image'],anchor_candidates=a,green_candidates=g,
                green_normal_in_anchor=(Ra.T@Rg[:,2]).tolist(),center_in_anchor_m=(Ra.T@(center-ta)).tolist(),
                anchor_height_relative_cube_center_along_green_normal_mm=float(1000*(ta-center)@Rg[:,2]))
            rows.append(row)
    summary=[]
    for profile in r['profiles']:
        group=[v for v in rows if v['profile']==profile['name']]
        if not group:continue
        n=np.asarray([v['green_normal_in_anchor'] for v in group]);median=np.median(n,axis=0);median/=np.linalg.norm(median);spread=np.degrees(np.arccos(np.clip(n@median,-1,1)))
        centers=np.asarray([v['center_in_anchor_m'] for v in group]);gaps=[]
        for v in group:
            c=v['green_candidates']
            if len(c)>1:gaps.append(c[1]['reprojection_rms_px']-c[0]['reprojection_rms_px'])
        summary.append(dict(profile=profile['name'],samples=len(group),median_green_normal_in_anchor=median.tolist(),
            maximum_normal_deviation_deg=float(max(spread)),median_cube_center_in_anchor_mm=(np.median(centers,axis=0)*1000).tolist(),
            center_axis_peak_to_peak_mm=(np.ptp(centers,axis=0)*1000).tolist(),
            green_planar_branch_pixel_error_gaps=gaps,
            green_rms_px=[v['green_candidates'][0]['reprojection_rms_px'] for v in group],
            anchor_rms_px=[v['anchor_candidates'][0]['reprojection_rms_px'] for v in group]))
    report=dict(status='camera_only_marker_pose_and_noise_diagnostic',source_sha256=hashlib.sha256((FOLDER/'result.json').read_bytes()).hexdigest(),
        intrinsic_source='camera1_720.npz',intrinsic=K.tolist(),distortion=dist.ravel().tolist(),profiles=summary,frames=rows,
        green_face_as_gravity_reference_not_verified=True,cube_support_surface_in_photo_not_confirmed_level=True,
        hand_registration_not_verified=True,hardware_execution_allowed=False,real_devices_opened=False)
    (FOLDER/'pose_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(profiles=summary,intrinsic=K.tolist()),indent=2),flush=True)
if __name__=='__main__':main()
