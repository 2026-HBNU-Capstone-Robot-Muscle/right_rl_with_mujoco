"""Device-free counter-yaw hypotheses; strict geometry before finite cube replay."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim
from calibrated_quarter_plan_20261028 import LOADED_PATH,OPEN_PATH
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/counter_yaw_contact_scout';OUT.mkdir(exist_ok=True)
def interp(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
    original=np.load(LOADED_PATH);opened=np.load(OPEN_PATH)['open_goal'];grasp=original['goals'][original['phases']=='grasp'];yaw=original['goals'][original['phases']=='yaw'];base=grasp[-1]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'geometry.xml',book_projection_mm=-10.);correct_model(m,opened)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);lo,hi=limits();rows=[]
    experiments=[('thumb_axial',[2]),('middle_side',[9]),('thumb_and_middle_side',[2,9])]
    mapping={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold','release':'stage_9_loaded_open','remove_hold':'stage_10_remove_hold'}
    for name,indices in experiments:
        turn=yaw.copy();turn[:,indices]=2*base[indices]-turn[:,indices]
        release=interp(turn[-1],opened)
        pieces=[('grasp',grasp),('grasp_hold',np.repeat(base[None,:],40,axis=0)),('yaw',turn),('yaw_hold',np.repeat(turn[-1][None,:],40,axis=0)),('release',release),('remove_hold',np.repeat(opened[None,:],40,axis=0))]
        goals=np.vstack([q for s,q in pieces]);phases=np.asarray([mapping[s] for s,q in pieces for _ in q]);values=np.vstack([opened,goals])
        within=bool(np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2)
        folder=OUT/name;folder.mkdir(exist_ok=True);path=folder/'candidate.npz';np.savez_compressed(path,start=opened,goals=goals,phases=phases)
        row=dict(name=name,moving_side_motor_ids=[i+1 for i in indices],native_and_two_tick_bounds=within,goals=len(goals),nominal_s=len(goals)*.075,candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if within:
            d=mujoco.MjData(m);d.qpos[-7:]=[0,0,10,1,0,0,0];peakself=peaksupport=0.;worst=None;first=None
            for i,q in enumerate(goals):
                d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);su,sp=inspect(d);sp=classify(sp)
                cur=max(sp.values(),default=0.);peakself=max(peakself,cur);peaksupport=max(peaksupport,max(su.values(),default=0.))
                if cur>1e-6 or peaksupport>.15:first=i;worst=max(sp,key=sp.get) if sp else None;break
            row.update(static_max_self_mm=peakself,static_max_support_mm=peaksupport,first_rejected_goal=first,worst_self_pair=worst,geometry_passed=first is None)
        else:row.update(geometry_passed=False)
        rows.append(row);print(json.dumps(row),flush=True)
        (OUT/'geometry_review.json').write_text(json.dumps(dict(cases=rows,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
