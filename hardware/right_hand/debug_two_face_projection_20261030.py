import json
from pathlib import Path
import numpy as np,cv2 as cv
from fit_two_face_placement_20261030 import R,F,O,K,D,RG,read_image,decoded,grid_poses
for folder in (R/'single_forward_trial_20261030_052636_068402',F):
 name='cube_placed.jpg' if folder==F else 'cube_before_grasp.jpg';im=read_image(folder/name);found=decoded(im);fit=grid_poses(im,found,3);m=found[3];p=fit['candidates'][0];rot=np.array(p['rotation']);tv=np.array(p['translation']);rv=cv.Rodrigues(rot)[0];points=np.array([[0,0,0],[-.0275,0,-.0275],[.0275,0,-.0275],[0,.0275,-.0275],[0,-.0275,-.0275]]);project=cv.projectPoints(points,rv,tv,K,D)[0].reshape(-1,2);print(folder.name,'IDs',list(found),'marker3',m.tolist(),'R',rot.tolist(),'t',tv.tolist(),'projections',project.tolist(),'grid',fit['grid_image_points'])
