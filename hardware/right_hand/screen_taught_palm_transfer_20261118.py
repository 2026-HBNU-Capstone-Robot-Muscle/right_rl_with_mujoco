"""Offline hand-geometry screen of taught transfer and latest empty preparation."""
from pathlib import Path
import argparse,json
import mujoco,numpy as np
import leap_measured_grip_hold_20261117 as pose
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=('taught_palm_transfer','tip_first_lower_seat'),default='taught_palm_transfer');args=ap.parse_args()
    model=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    data=mujoco.MjData(model);_,_,check=inspect_for(model)
    folder=BASE/args.variant
    z=np.load(folder/'candidate.npz',allow_pickle=False)
    refs=[json.loads((BASE/(n+'_reference.json')).read_bytes()) for n in ('middle_clear','cube_pushed','thumb_wrist_clear')]
    if args.variant=='tip_first_lower_seat':refs.append(json.loads((BASE/'index_middle_lower_seat_reference.json').read_bytes()))
    def geometry(q):
        data.qpos[:16]=ticks_to_sim(np.asarray(q));mujoco.mj_fwdPosition(model,data)
        support,own=check(data);return support,own
    baseline=[{},{}]
    for q in [z['goals'][1416],pose.reference()[2],*[r['actual_ticks'] for r in refs]]:
        for totals,values in zip(baseline,geometry(q)):
            for k,v in values.items():totals[k]=max(totals.get(k,0.),float(v))
    paths={'transfer':z['goals'][1417:],'empty_preparation':pose.path(refs[-1]['actual_ticks'],z['start'])}
    reports={}
    for name,rows in paths.items():
        worst=[{},{}];extra=[0.,0.];where=[None,None]
        for i,q in enumerate(rows):
            for j,values in enumerate(geometry(q)):
                for k,v in values.items():
                    worst[j][k]=max(worst[j].get(k,0.),float(v));delta=float(v)-baseline[j].get(k,0.)
                    if delta>extra[j]:extra[j]=delta;where[j]=dict(row=i,pair=k,absolute_mm=float(v))
        reports[name]=dict(goals=len(rows),new_support_over_reference_baseline_mm=extra[0],
            new_self_over_reference_baseline_mm=extra[1],worst_new_support=where[0],worst_new_self=where[1],
            support_worst_mm=worst[0],self_worst_mm=worst[1],hand_geometry_only=True,
            cube_must_be_absent=name=='empty_preparation',loaded_cube_retention_verified=False)
    report=dict(kind='taught_transfer_hand_geometry_screen',paths=reports,
        physical_execution=False,reference_overlap_is_not_zero=True,recorded_pose_baseline=baseline)
    (folder/'hand_geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:{field:v for field,v in r.items() if field not in ('support_worst_mm','self_worst_mm')} for k,r in reports.items()}))

if __name__=='__main__':main()
