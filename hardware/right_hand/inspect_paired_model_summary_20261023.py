import json
from pathlib import Path
import numpy as np
from grasp_place_plan import endpoints

root=Path(__file__).resolve().parent;folder=root/'cube_pose_readonly_20261023_171233_601943'
data=json.loads((folder/'native_geometry_review.json').read_bytes())
poses=json.loads((folder/'direct_marker_pose_review.json').read_bytes())
for row in data['rows']:
    if row['image']=='pair_05.png' and row['palm_branch']==0:
        print('TYPE',row['source'],'BRANCH',row['cube_branch'],'CONTACTS',row['cube_contacts'])
for row in poses['hypotheses']:
    if row['image']=='pair_05.png' and row['palm_branch']==0:
        print('TYPE',row['source'],'BRANCH',row['cube_branch'],'WORLD_CENTER',row['cube_center_world_m'])
pair=poses['rows'][3]
print('IMAGE',pair['image'])
print('GRID_ERROR',pair['grid_diagnostic']['grid_pnp_mean_error_px'])
print('GRID_MARKER_SIDES',pair['grid_diagnostic']['marker_side_from_grid_mm'])
opened,closed=endpoints()
current=np.array(pair['before']['snapshot']['position_ticks'])
print('OPENED',opened.tolist())
print('CLOSED',closed.tolist())
print('CURRENT',current.tolist())
print('CLOSED_MINUS_CURRENT',(closed-current).tolist())
