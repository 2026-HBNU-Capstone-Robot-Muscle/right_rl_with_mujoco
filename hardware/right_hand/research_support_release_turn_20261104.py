"""Test whether lowering the index support unlocks the remaining yaw.

Every evaluated stage uses a free cube. Saved geometry proposals cannot grant
hardware admission. Existing force, contact, step and native bounds persist.
"""
import json,time
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import R,CP,OUT,propose,evaluate,motor,restore,angle
from hand_motor_mapping import ticks_to_sim

def state(d,z):
    return {**{k:getattr(d,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')},'time':float(d.time),'up':z['up'],'initial_cube_rotation':z['initial_cube_rotation'],'initial_cube_center':z['initial_cube_center']}

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));original=np.load(CP/'closest_state.npz')
    first=np.load(OUT/'turn_0_lower_8_normal_10/research_suffix.npz')['goals']
    low=np.load(OUT/'turn_0_lower_8_normal_10/end_state.npz')
    results=[]
    for release in (32,64,96,128,160):
      for normal in (2.,10.):
        O=OUT/f'release_{release}_normal_{normal:g}';O.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
        start=motor(low['ctrl']);end=start.copy();end[7]-=release
        n=max(1,int(np.ceil(release/2)));part=np.rint([start+(end-start)*i/n for i in range(1,n+1)]).astype(int)
        part=np.vstack([part,np.tile(end,(40,1))])
        release_result=evaluate(m,low,part,O/'release_state.npz')
        report=dict(release_ticks=release,normal_weight=normal,release=release_result,hardware_execution_allowed=False,devices_opened=False)
        if release_result['safety_passed']:
            s=np.load(O/'release_state.npz');d=restore(m,s);turn=90-angle(d.xmat[m.body('cube').id].reshape(3,3),s['initial_cube_rotation'],s['up'])
            suffix,ik,error=propose(m,s,turn,0.,normal)
            report.update(turn_goal_delta=turn,geometry_stop=error,IK=ik)
            if suffix is not None:
                report['dynamic']=evaluate(m,s,suffix,O/'end_state.npz')
                allrows=np.vstack([first,part,suffix]);np.savez_compressed(O/'research_suffix.npz',start=motor(original['ctrl']),goals=allrows)
        report['elapsed_seconds']=time.perf_counter()-began
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        results.append(report);(OUT/'support_release_batch.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='IK'}),flush=True)
if __name__=='__main__':main()
