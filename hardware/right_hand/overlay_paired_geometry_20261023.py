"""Scientific projection of measured pose and provisional model registration."""
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_171233_601943'


def main():
    data=json.loads((FOLDER/'native_geometry_review.json').read_bytes())
    image=cv.imdecode(np.frombuffer((FOLDER/'pair_05.png').read_bytes(),np.uint8),cv.IMREAD_COLOR)
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    edges=[(a,b) for a in range(8) for b in range(a+1,8) if np.count_nonzero(signs[a]!=signs[b])==1]
    colors={'if_':(255,150,30),'mf_':(40,220,40),'rf_':(0,200,255),'th_':(80,70,255)}
    for row in data['representatives']:
        if row['source']!='direct_ID40_and_solved_green_grid_diagnostic':continue
        show=image.copy();R=np.array(row['world_to_camera_rotation']);t=np.array(row['world_to_camera_translation'])
        rv=cv.Rodrigues(R)[0]
        def project(points):return cv.projectPoints(np.array(points,float),rv,t,K,dist)[0].reshape(-1,2)
        cube=project(row['cube_vertices_world_m'])
        for a,b in edges:cv.line(show,tuple(np.rint(cube[a]).astype(int)),tuple(np.rint(cube[b]).astype(int)),(255,0,255),1,cv.LINE_AA)
        for tip in row['tip_boxes']:
            points=project(tip['box_vertices_world_m']);color=colors[tip['name'][:3]]
            for a,b in edges:cv.line(show,tuple(np.rint(points[a]).astype(int)),tuple(np.rint(points[b]).astype(int)),color,2,cv.LINE_AA)
            point=np.rint(project([tip['center_world_m']])[0]).astype(int)
            cv.putText(show,tip['name'],tuple(point+[7,0]),cv.FONT_HERSHEY_SIMPLEX,.6,color,2)
        anchor_pixels={}
        for name,point in row['joint_anchors_world_m'].items():
            if not (name.endswith('mcp') or name=='th_cmc'):continue
            xy=np.rint(project([point])[0]).astype(int);anchor_pixels[name]=xy.tolist()
            cv.circle(show,tuple(xy),5,colors[name[:3]],2)
            cv.putText(show,name,tuple(xy+[8,-7]),cv.FONT_HERSHEY_SIMPLEX,.5,colors[name[:3]],1)
        cv.rectangle(show,(0,0),(1280,64),(0,0,0),-1)
        cv.putText(show,'Same-frame q + decoded ID40; marker/model mount remains provisional',(10,24),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        cv.putText(show,'blue=index green=middle gold=ring red=thumb magenta=cube grid diagnostic',(10,50),cv.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        target=FOLDER/f"paired_model_palm_branch_{row['palm_branch']}.jpg"
        target.write_bytes(cv.imencode('.jpg',show)[1].tobytes())
        print('BRANCH',row['palm_branch'],'ROOT AXES',anchor_pixels)


if __name__=='__main__':main()
