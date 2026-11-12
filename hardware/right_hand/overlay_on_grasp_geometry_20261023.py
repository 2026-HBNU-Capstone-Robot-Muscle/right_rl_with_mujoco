"""Project the unchanged diagnostic model onto the actual ON image."""
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'


def main():
    data=json.loads((FOLDER/'on_native_geometry_review.json').read_bytes())
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    edges=[(a,b) for a in range(8) for b in range(a+1,8) if np.count_nonzero(signs[a]!=signs[b])==1]
    colors={'if_':(255,150,30),'mf_':(40,220,40),'rf_':(0,200,255),'th_':(80,70,255)}
    for row in data['representatives']:
        if not row['source'].endswith('grid_diagnostic'):continue
        show=cv.imdecode(np.frombuffer((FOLDER/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        R=np.array(row['world_to_camera_rotation']);t=np.array(row['world_to_camera_translation']);rv=cv.Rodrigues(R)[0]
        def project(points):return cv.projectPoints(np.array(points,float),rv,t,K,dist)[0].reshape(-1,2)
        cube=project(row['cube_vertices_world_m'])
        for a,b in edges:cv.line(show,tuple(np.rint(cube[a]).astype(int)),tuple(np.rint(cube[b]).astype(int)),(255,0,255),1,cv.LINE_AA)
        for box in row['tip_boxes']:
            points=project(box['vertices_world_m']);color=colors[box['name'][:3]]
            for a,b in edges:cv.line(show,tuple(np.rint(points[a]).astype(int)),tuple(np.rint(points[b]).astype(int)),color,2,cv.LINE_AA)
            point=np.rint(np.mean(points,axis=0)).astype(int)
            cv.putText(show,box['name'],tuple(point+[7,0]),cv.FONT_HERSHEY_SIMPLEX,.6,color,2)
        cv.rectangle(show,(0,0),(1280,64),(0,0,0),-1)
        cv.putText(show,'Recorded ON q; unchanged provisional marker/model mount',(10,25),cv.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
        cv.putText(show,'blue=index green=middle gold=ring red=thumb magenta=cube grid diagnostic',(10,50),cv.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        target=FOLDER/f"on_model_palm_branch_{row['palm_branch']}.jpg"
        target.write_bytes(cv.imencode('.jpg',show)[1].tobytes());print(target)


if __name__=='__main__':main()
