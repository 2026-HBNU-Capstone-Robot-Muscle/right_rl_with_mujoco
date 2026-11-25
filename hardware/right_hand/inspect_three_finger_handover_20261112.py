"""Compact, read-only offline geometry report for the pending grasp handover."""
from pathlib import Path
import json
import sys
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'
sys.path.insert(0,str(WORK))
from native_cycle import assets,restore,Guard,motor
from whole_hand_support import sample_model

def main():
    m,initial,*_=assets('active_ccd50')
    base=ROOT/'right_hand_rotation_20261112/support_first_transfer'
    paths=[base/'middle_shape_like_index/end_state.npz',
           base/'lower_pad_lift_after_clear_v2/start_state.npz',
           base/'lower_pad_lift_after_clear_v2/end_state.npz',
           WORK/'next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz']
    for path in paths:
        d=restore(m,dict(np.load(path,allow_pickle=False)));g=Guard(m);cube=m.geom('cube').id
        C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
        geoms={}
        for f in ('if','mf','rf','th'):
            for j in range(m.ngeom):
                n=m.geom(j).name or ''
                if n.startswith(f+'_'):
                    geoms[n]=np.round((B.T@(d.geom_xpos[j]-C))*1000,2).tolist()
        r=dict(path=path.relative_to(ROOT).as_posix(),cube_world_mm=np.round(C*1000,3).tolist(),
               cube_rotation=np.round(B,4).tolist(),up=initial['up'].tolist(),
               goal=motor(d.ctrl).tolist(),actual=motor(d.qpos[:16]).tolist(),guard=g.check(d),
               cube_frame_geoms_mm=geoms,load=sample_model(m,d,g.hand,cube))
        print(json.dumps(r),flush=True)

if __name__=='__main__':main()
