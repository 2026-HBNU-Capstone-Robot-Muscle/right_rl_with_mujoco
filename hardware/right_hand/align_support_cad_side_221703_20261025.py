"""Provisional CAD/image edge comparison; not a camera registration certificate."""
import itertools,json
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703'
SOURCE=ROOT/'quarter_support_photo_20261025_221703_805528'
image=cv.imdecode(np.frombuffer((SOURCE/'empty_support_reference.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
cad=json.loads((OUT/'cad_geometry.json').read_bytes());vision=json.loads((OUT/'provisional_camera_orientation.json').read_bytes())
pose=next(b for b in vision['branches'] if b['orange_face_visibility']>0)
Rcube=np.array(pose['R_cam_cube']);tCube=np.array(pose['t_cam_cube']);palmR=np.array(cad['palm_world_rotation']);origin=np.array(cad['palm_world_position'])
up=-palmR[:,2]
if up[2]<0:up=-up
x=palmR[:,0];x=x-up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x)
RworldCube=np.column_stack([x,-up,np.cross(x,-up)]);RcamWorld=Rcube@RworldCube.T
frame=np.column_stack([x,y,up])
with np.load(ROOT/'camera1_720.npz') as cal:K=cal['K'].copy();dist=cal['dist'].copy()
boxes=[g for g in cad['geoms'] if g['group']==3 and 'corners_world' in g and g['body']!='palm']
points=[];lines=[]
for g in boxes:
    corners=np.array(g['corners_world']);R=np.array(g['rotation']);camCenter=RcamWorld@(np.array(g['center'])-origin)+tCube
    visible={(axis,sgn) for axis in range(3) for sgn in (-1,1) if (RcamWorld@(R[:,axis]*sgn))@camCenter<0}
    signs=list(itertools.product((-1,1),repeat=3))
    for i,j in itertools.combinations(range(8),2):
        changed=[k for k in range(3) if signs[i][k]!=signs[j][k]]
        if len(changed)!=1:continue
        if not any((k,signs[i][k]) in visible for k in range(3) if k!=changed[0]):continue
        lines.append((g,corners[i],corners[j]));points.extend(np.linspace(corners[i],corners[j],8))
points=np.array(points);rcPoints=points@RcamWorld.T
edges=cv.Canny(cv.cvtColor(image,cv.COLOR_BGR2GRAY),35,100);distance=cv.distanceTransform(255-edges,cv.DIST_L2,3)
def world(params):return origin+frame@(np.array(params[:3])*.001)
def camera_rotation(params):
    angle=np.radians(params[3] if len(params)>3 else 0.);forward=x*np.cos(angle)+y*np.sin(angle)
    return Rcube@np.column_stack([forward,-up,np.cross(forward,-up)]).T
def projected(points,params):
    R=camera_rotation(params);T=tCube-R@world(params);q=np.array(points)@R.T+T
    return q[:,:2]/q[:,2:]*np.array([K[0,0],K[1,1]])+np.array([K[0,2],K[1,2]])
def objective(params):
    p=projected(points,params);valid=(p[:,0]>=20)&(p[:,0]<1250)&(p[:,1]>=30)&(p[:,1]<710)
    p=np.clip(np.rint(p),[0,0],[1279,719]).astype(int);d=distance[p[:,1],p[:,0]];d[~valid]=25
    # This is only a ranking for inspection. Cable/internal edges are clutter.
    return float(np.minimum(d,20).mean())
rank=[]
for s,yy,h,yaw in itertools.product(np.arange(20,66,10),np.arange(-8,33,10),np.arange(45,66,5),np.arange(-45,46,10)):
    p=np.array([s,yy,h,yaw],float);rank.append((objective(p),p))
rank.sort(key=lambda r:r[0]);seeds=[]
for cost,p in rank:
    if all(np.linalg.norm(p-q)>5 for _,q in seeds):seeds.append((cost,p))
    if len(seeds)==6:break
fitted=[]
for _,p in seeds:
    for step in (2.,1.,.5):
        for _ in range(8):
            candidates=[p]+[p+np.eye(4)[k]*v*step for k in range(4) for v in (-1,1)]
            candidates=[v for v in candidates if 15<=v[0]<=75 and -25<=v[1]<=50 and 43<=v[2]<=70 and -60<=v[3]<=60]
            new=min(candidates,key=objective)
            if np.array_equal(new,p):break
            p=new
    fitted.append((objective(p),p))
fitted.sort(key=lambda r:r[0]);canvas=np.zeros((2*360,3*640,3),np.uint8);rows=[]
for n,(cost,p) in enumerate(fitted):
    view=image.copy()
    for g,a,b in lines:
        uv=projected([a,b],p)
        if np.isfinite(uv).all() and abs(uv).max()<3000:cv.line(view,tuple(np.rint(uv[0]).astype(int)),tuple(np.rint(uv[1]).astype(int)),(40,200,40),1)
    for axis in cad['motor_axes']:
        uv=projected([axis['world_anchor']],p)[0]
        if 0<=uv[0]<1280 and 0<=uv[1]<720:
            xy=tuple(np.rint(uv).astype(int));cv.circle(view,xy,4,(0,0,255),-1);cv.putText(view,str(axis['motor_id']),xy,cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    cv.putText(view,f'PROVISIONAL s/y/h/yaw={p.round(1).tolist()}  score={cost:.2f}',(20,45),cv.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
    yy,xx=divmod(n,3);canvas[yy*360:(yy+1)*360,xx*640:(xx+1)*640]=cv.resize(view,(640,360))
    R=camera_rotation(p)
    rows.append(dict(edge_score=cost,cube_center_book_frame_mm=p[:3].tolist(),cube_yaw_from_palm_degrees=float(p[3]),cube_center_model_world=world(p).tolist(),R_cam_world_assuming_upright=R.tolist(),camera_t_world=(tCube-R@world(p)).tolist()))
cv.imencode('.jpg',canvas)[1].tofile(str(OUT/'provisional_cad_alignment_yaw_candidates.jpg'))
(OUT/'provisional_cad_alignment_yaw.json').write_text(json.dumps(dict(candidates=rows,upright_cube_assumed=True,markers_do_not_register_hand=True,edge_ranking_not_registration=True,hardware_execution_allowed=False),indent=2),encoding='utf-8')
print(json.dumps(rows))
