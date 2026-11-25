"""Read the reached model endpoint and joint headroom without device access."""
import json
import argparse
import numpy as np
from native_cycle import OUT, assets, restore, Guard, SIM_TO_MOTOR_IDS, TICK, motor
from directed_contact_state import forwarded_state

def main():
    base=OUT/'next_cycle/handoff_refinement_20261110'
    p=argparse.ArgumentParser();p.add_argument('--study',default='robust_mu025_declared_face_feedback_v5');p.add_argument('--case',default='force0.8');args=p.parse_args()
    root=(base/args.study/args.case).resolve()
    if not root.is_relative_to(base.resolve()):raise ValueError('Local model endpoint required')
    r=json.loads((root/'review.json').read_text())
    print(json.dumps({k:v for k,v in r.items() if k!='trace'}))
    m,initial,*_=assets('active_ccd50');m.geom_friction[:,0]=.25
    z=dict(np.load(root/'end_state.npz',allow_pickle=False));d=restore(m,z);g=Guard(m)
    rows=[]
    for i,mid in enumerate(SIM_TO_MOTOR_IDS):
        rows.append(dict(id=int(mid),q_deg=float(np.degrees(d.qpos[i])),command_ticks=int(motor(d.ctrl)[mid-1]),lower_room_ticks=float((d.qpos[i]-g.qlo[i])/TICK),upper_room_ticks=float((g.qhi[i]-d.qpos[i])/TICK),tracking_ticks=float((d.ctrl[i]-d.qpos[i])/TICK)))
    print(json.dumps(dict(joints=rows,cube_center=d.qpos[-7:-4].tolist())))

if __name__=='__main__':main()
