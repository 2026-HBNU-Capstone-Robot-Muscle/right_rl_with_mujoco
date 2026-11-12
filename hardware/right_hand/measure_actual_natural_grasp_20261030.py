"""Saved-frame body-relative motion review; no new capture or devices."""
import json
from pathlib import Path
import numpy as np,cv2 as cv
from placement_reference_geometry_20261030 import decoded,fused,poses
R=Path(__file__).resolve().parent;F=R/'natural_cradle_grasp_20261030_110443_655705';O=R/'right_hand_rotation_20261030/natural_cradle_grasp/actual_review';run=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames'];selected=[]
for name in ('cube_before_grasp.jpg','cube_grasp_hold.jpg','cube_after_open.jpg'):selected.append((F/name,'named'))
for phase in ('stage_6_grasp_hold','stage_10_remove_hold'):
 fs=[v for v in frames if v['phase']==phase]
 for f in fs[::max(1,len(fs)//12)][:13]:selected.append((F/'camera'/f['image'],phase))
rows=[]
for path,phase in selected:
 im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);found=decoded(im);fit=fused(im,found);row=dict(image=path.relative_to(R).as_posix(),phase=phase,decoded_ids=sorted(found),cube_fused=fit['accepted'],top_marker_inferred=False)
 if fit['accepted']:
  fp=fit['candidates'][0];row['cube_fit']=fp
  for ident,size in ((40,.0282),(41,.027)):
   if ident not in found:continue
   pp=poses(found[ident],size)
   if pp:
    b=pp[0];row.setdefault('body_relative',{})[str(ident)]=dict(center_mm=(b['rotation'].T@(np.array(fp['center_camera'])-b['translation'])*1000).tolist(),orange_rotation_body=(b['rotation'].T@np.array(fp['rotation'])).tolist(),body_rmse_px=b['rmse_px'])
 rows.append(row)
print(json.dumps([dict(image=Path(r['image']).name,phase=r['phase'],ids=r['decoded_ids'],fit=r['cube_fused'],bodies=list(r.get('body_relative',{}))) for r in rows]))
summary={}
for ident in ('40','41'):
 base=rows[0].get('body_relative',{}).get(ident)
 if base:
  B=np.array(base['orange_rotation_body']);C=np.array(base['center_mm']);stat=[]
  for r in rows:
   v=r.get('body_relative',{}).get(ident)
   if not v:continue
   A=np.array(v['orange_rotation_body'])@B.T;rv=cv.Rodrigues(A)[0].ravel();theta=np.linalg.norm(rv);angle=np.degrees(2*np.arctan2((rv[2]/theta if theta>1e-12 else 0.)*np.sin(theta/2),np.cos(theta/2)));stat.append(dict(image=Path(r['image']).name,phase=r['phase'],twist_about_body_normal_deg=float(angle),center_shift_mm=float(np.linalg.norm(np.array(v['center_mm'])-C)),center_mm=v['center_mm']))
  summary[ident]=stat
report=dict(status='saved_natural_grasp_visual_motion_review',rows=rows,relative_motion=summary,body_marker_frame_not_robot_axes=True,absolute_registration_verified=False,physical_grasp_force_not_measured=True,physical90_not_verified=True,motor_writes=0)
(O/'visual_motion_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary,indent=2))
