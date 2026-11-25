"""Replay placement -> saved yaw prefix -> new body hold without state resets."""
import json,time
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import R,CP,OUT,motor,evaluate,inspect_for,restore
from hand_motor_mapping import ticks_to_sim
from cube_contact_material_policy_20261029 import apply_material

def initial(m,opened,up,seed):
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed;d.qvel[:]=0
    cube=m.geom('cube').id;hand,support,check=inspect_for(m);center=seed[:3].copy()
    def depth(height):
        d.qpos[-7:-4]=center+up*height;mujoco.mj_forward(m,d)
        return max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand),default=0.)
    if depth(.04)>1e-5:raise RuntimeError('Upper seating endpoint not free')
    previous=.04;lower=None
    for height in np.arange(.04,-.025,-.0005):
        if depth(float(height))>1e-5:lower=float(height);upper=previous;break
        previous=float(height)
    if lower is None:raise RuntimeError('No support on bounded vertical line')
    for _ in range(24):
        mid=(lower+upper)/2
        if depth(mid)>1e-5:lower=mid
        else:upper=mid
    d.qpos[-7:-4]=center+up*(upper+.0001);d.qvel[:]=0;mujoco.mj_forward(m,d)
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.);external=False
    for step in range(2001):
        if step:mujoco.mj_step(m,d)
        su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
            elif other in support and c.dist<.0004:external=True
    passed=worst['self_mm']<=1e-6 and worst['support_mm']<=.15 and worst['cube_mm']<=1 and worst['tracking_rad']<=.18 and not external
    z={k:getattr(d,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};z.update(time=d.time,up=up,initial_cube_rotation=d.xmat[m.body('cube').id].reshape(3,3).copy(),initial_cube_center=d.qpos[-7:-4].copy())
    return z,dict(passed=passed,worst=worst,external_cube_contact=external,vertical_seating_shift_mm=(upper+.0001)*1000)

def main():
    old=np.load(R/'right_hand_rotation_20261104/raised_cradle_160_yaw0/candidate.npz');suffix=np.load(OUT/'lower_pair_m20_t16/research_suffix.npz');cp=np.load(CP/'closest_state.npz')
    assert np.array_equal(old['goals'][1367],suffix['start'])
    values=np.vstack([old['goals'][:1368],suffix['goals']]);phases=np.r_[old['phases'][:1368],['stage_8_body_regrasp']*(len(suffix['goals'])-67),['stage_9_body_hold']*67]
    np.savez_compressed(OUT/'body_hold_research_candidate.npz',start=old['start'],goals=values,phases=phases,forward_end_sample=len(values))
    seed=np.array(json.loads((R/'right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json').read_bytes())['cube_qpos']);cases=[]
    for gain in (1.,.5,1.5):
      for friction in (.3,.2,.5):
        began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));hand,support,check=inspect_for(m)
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain;apply_material(m,friction,hand,m.geom('cube').id)
        z,placement=initial(m,old['start'],cp['up'],seed)
        row=dict(gain_scale=gain,friction=friction,initial_placement=placement)
        if placement['passed']:
            path=OUT/f'full_replay_g{gain:g}_f{friction:g}.npz';row['dynamic']=evaluate(m,z,values,path)
        else:row['dynamic']=None
        row['elapsed_seconds']=time.perf_counter()-began;cases.append(row)
        report=dict(status='continuous_placement_yaw_body_hold_research',book_front_mm=-10.,goals=len(values),seconds=len(values)*.075,cases=cases,all_initial_placements_passed=all(c['initial_placement']['passed'] for c in cases),all_dynamic_safety_passed=all(c['dynamic'] and c['dynamic']['safety_passed'] for c in cases),hardware_execution_allowed=False,devices_opened=False,physical90_verified=False)
        (OUT/'full_replay_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='dynamic'}|{'dynamic':None if row['dynamic'] is None else {k:v for k,v in row['dynamic'].items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
