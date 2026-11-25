"""Scientific overlay of unverified model geometry on a saved photo."""
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    reference=json.loads((FOLDER/'direct_reference_cube_pose.json').read_bytes())
    geometry=json.loads((FOLDER/'direct_pose_geometry_audit.json').read_bytes())
    with np.load(ROOT/'camera1_720.npz') as data:K,dist=data['K'],data['dist']
    base=cv.imdecode(np.frombuffer((FOLDER/'cube_after_approach.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    edges=[(a,b) for a in range(8) for b in range(a+1,8) if np.count_nonzero(signs[a]!=signs[b])==1]
    colors={'if_':(255,150,30),'mf_':(40,220,40),'rf_':(0,200,255),'th_':(80,70,255)}
    for branch in geometry['hypotheses']:
        show=base.copy()
        R=np.array(branch['world_to_camera_rotation']);tv=np.array(branch['world_to_camera_translation'])
        rv=cv.Rodrigues(R)[0]
        def project(pts):return cv.projectPoints(np.array(pts,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
        for tip in geometry['tips']:
            points=project(tip['box_vertices_world_m']);color=colors[tip['name'][:3]]
            for a,b in edges:cv.line(show,tuple(np.rint(points[a]).astype(int)),tuple(np.rint(points[b]).astype(int)),color,2,cv.LINE_AA)
            center=np.rint(project([tip['center_world_m']])[0]).astype(int)
            cv.circle(show,tuple(center),5,color,-1)
            cv.putText(show,tip['name'],tuple(center+[8,0]),cv.FONT_HERSHEY_SIMPLEX,.6,color,2)
        for name,pt in geometry['joint_anchors_world_m'].items():
            if name.endswith('mcp') or name=='th_cmc':
                xy=np.rint(project([pt])[0]).astype(int)
                cv.circle(show,tuple(xy),4,colors[name[:3]],-1)
                cv.putText(show,name,tuple(xy+[7,-7]),cv.FONT_HERSHEY_SIMPLEX,.45,colors[name[:3]],1)
        for pt in reference['visible_rim_pixels']:cv.circle(show,tuple(np.rint(pt).astype(int)),4,(0,255,255),-1)
        cv.rectangle(show,(0,0),(1280,60),(0,0,0),-1)
        cv.putText(show,'UNVERIFIED manual borders + provisional palm mount; diagnostic only',(8,23),cv.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
        cv.putText(show,'blue index / green middle / gold ring / red thumb',(8,49),cv.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        output=FOLDER/f"direct_pose_branch_{branch['palm_ippe_branch']}_overlay.jpg"
        cv.imencode('.jpg',show)[1].tofile(str(output))
        print('BRANCH',branch['palm_ippe_branch'],'TIP CENTERS',[(t['name'],np.round(project([t['center_world_m']])[0],1).tolist()) for t in geometry['tips']])

if __name__=='__main__':main()
