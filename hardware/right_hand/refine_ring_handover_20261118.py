"""Device-free comparison of thumb catch before middle/ring handover."""
from pathlib import Path
import json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for
from build_taught_palm_transfer_20261118 import interpolate

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/grip_to_rotation'

def main():
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});d=mujoco.MjData(m)
    check=inspect_for(m)[2];z=np.load(BASE/'candidate.npz',allow_pickle=False)
    goals=z['goals'];phases=z['phases'];start=z['start']
    def endpoint(name):return goals[np.flatnonzero(phases==name)[-1]].copy()
    index=endpoint('release_index');ready=endpoint('middle_ring_to_turn_ready')
    thumb=index.copy();thumb[:4]=ready[:4]
    ring=thumb.copy();ring[12:16]=ready[12:16]
    root=thumb.copy();root[15]=ready[15]
    mid=thumb.copy();mid[8:12]=ready[8:12]
    variants={'middle_before_ring_release':[('thumb_middle_catch',mid),('ring_release',ready)],
        'thumb_then_coordinated': [('thumb_catch',thumb),('middle_ring_handover',ready)],
        'thumb_ring_middle': [('thumb_catch',thumb),('ring_clear',ring),('middle_ready',ready)],
        'thumb_ring_root_then_coordinated':[('thumb_catch',thumb),('ring_root_clear',root),('middle_ring_handover',ready)]}
    output={}
    for name,stages in variants.items():
        end=int(np.flatnonzero(phases=='release_index')[-1])+1
        rows=list(goals[:end]);labels=list(phases[:end]);previous=index
        for phase,target in stages:
            part=interpolate(previous,target);rows.extend(part);labels.extend([phase]*len(part));previous=target
        worst=dict(depth_mm=0.)
        for i,(q,phase) in enumerate(zip(rows,labels)):
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for pair,depth in check(d)[1].items():
                if depth>worst['depth_mm']:worst=dict(depth_mm=float(depth),index=i,phase=str(phase),pair=pair,ticks=np.asarray(q).tolist())
        output[name]=worst
    (BASE/'ring_handover_options.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(output))

if __name__=='__main__':main()
