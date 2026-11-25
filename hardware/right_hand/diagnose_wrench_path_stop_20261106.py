"""Inspect a saved model stop without executing or modifying a path."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from hand_motor_mapping import SIM_TO_MOTOR_IDS,SIM_JOINT_NAMES

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--case',required=True);a=ap.parse_args();folder=(ROOT/'quasistatic_wrench_path'/a.case).resolve()
    if not folder.is_relative_to((ROOT/'quasistatic_wrench_path').resolve()):raise ValueError('case outside research output')
    report=json.loads((folder/'review.json').read_bytes());source=R/report['source_state'];model=source.parent/'model.mjb';assert hashlib.sha256(model.read_bytes()).hexdigest()==report['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));z=np.load(folder/'end_state.npz');d=restore(m,z);hand,support,check=inspect_for(m);su,sp=check(d);_,_,lo,hi=native(m);cube=m.geom('cube').id;contacts=[]
    for i,c in enumerate(d.contact):
        x,y=int(c.geom1),int(c.geom2)
        if cube not in (x,y):continue
        other=y if x==cube else x;force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force);contacts.append(dict(other=m.geom(other).name,distance_mm=float(c.dist*1000),in_hand=int(m.geom_bodyid[other]) in hand,external_support=other in support,normal_N=float(force[0])))
    ranges=[]
    for i,q in enumerate(d.qpos[:16]):
        if q<lo[i]-1e-7 or q>hi[i]+1e-7:ranges.append(dict(joint=SIM_JOINT_NAMES[i],motor=SIM_TO_MOTOR_IDS[i],actual_rad=float(q),native_lower_rad=float(lo[i]),native_upper_rad=float(hi[i]),goal_rad=float(d.ctrl[i])))
    result=dict(status='saved_stop_geometric_diagnostic_only',case=a.case,source_review_sha256=hashlib.sha256((folder/'review.json').read_bytes()).hexdigest(),reported_worst=report['worst'],current_self_penetrations_mm={str(k):float(v) for k,v in sp.items() if v>1e-7},current_hand_support_penetrations_mm={str(k):float(v) for k,v in su.items() if v>1e-7},native_actual_range_violations=ranges,cube_contacts=contacts,last_path_samples=[{k:v for k,v in t.items() if k!='allocation'} for t in report['trace'][-3:]],joint_torque_Nm=d.actuator_force.tolist(),hardware_execution_allowed=False,devices_opened=False)
    (folder/'stop_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
