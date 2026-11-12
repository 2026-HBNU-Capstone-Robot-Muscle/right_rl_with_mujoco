"""Hold/load sensitivity using each torque-capped replay's own final state."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--cap',type=float,choices=(.15,.23),required=True);a=ap.parse_args()
    folder=ROOT/f'torque_cap_sensitivity/cap{a.cap:g}';raw=json.loads((folder/'review.json').read_bytes());assert raw['task_passed']
    model=folder/'model.mjb';source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(model));z=np.load(source)
    assert np.all(m.actuator_forcelimited) and np.all(m.actuator_forcerange[:,0]>=-a.cap) and np.all(m.actuator_forcerange[:,1]<=a.cap)
    out=ROOT/f'capped_grip_load/cap{a.cap:g}';out.mkdir(parents=True,exist_ok=True);cases=[]
    for torque in (0.,-.01,.01):
        began=time.perf_counter();row=evaluate(m,z,torque,out/f'torque{torque:+g}.npz');row['elapsed_seconds']=time.perf_counter()-began;cases.append(row)
        report=dict(status='nominal_grip_load_under_stricter_model_torque_cap',hypothetical_joint_torque_cap_Nm=a.cap,source_state=str(source.relative_to(R)),source_state_sha256=sha(source),source_model=str(model.relative_to(R)),source_model_sha256=sha(model),source_replay_report_sha256=sha(folder/'review.json'),cases=cases,real450mA_to_joint_torque_not_calibrated=True,rigid_cube_hypothetical_load_not_articulated_layer_or_left_hand=True,native_geometry_tracking_and_contact_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(cap=a.cap,torque=torque,safety=row['safety_passed'],task=row['load_filter_passed'],reasons=row['reasons'],yaw=row['max_yaw_change_deg'],shift=row['max_center_shift_mm'])),flush=True)
if __name__=='__main__':main()
