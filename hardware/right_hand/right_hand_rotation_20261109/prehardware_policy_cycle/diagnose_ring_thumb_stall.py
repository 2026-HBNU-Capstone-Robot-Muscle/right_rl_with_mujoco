"""Inspect an immutable executed ring/thumbnail turn snapshot, not a device."""
from pathlib import Path
import argparse
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,SIM_TO_MOTOR_IDS,motor,save_json,sha
from whole_hand_support import sample_model


def main():
    p=argparse.ArgumentParser();p.add_argument('--source');p.add_argument('--output');args=p.parse_args()
    folder=OUT/'next_cycle/handoff_refinement_20261110/ring_thumb_turn_checkpoint';path=folder/'state.npz' if not args.source else (OUT/args.source).resolve()
    dest=folder/'diagnosis.json' if not args.output else (OUT/args.output).resolve()
    if not path.resolve().is_relative_to(OUT.resolve()) or not dest.resolve().is_relative_to(OUT.resolve()):raise ValueError('Local research path required')
    model,_,_,_,_=assets('active_ccd50');d=restore(model,dict(np.load(path,allow_pickle=False)));g=Guard(model)
    checks=[]
    for i,mid in enumerate(SIM_TO_MOTOR_IDS):
        checks.append(dict(id=int(mid),q_deg=float(np.degrees(d.qpos[i])),goal_deg=float(np.degrees(d.ctrl[i])),lower_deg=float(np.degrees(g.qlo[i])),upper_deg=float(np.degrees(g.qhi[i])),
            lower_room_deg=float(np.degrees(d.qpos[i]-g.qlo[i])),upper_room_deg=float(np.degrees(g.qhi[i]-d.qpos[i]))))
    loads=sample_model(model,d,g.hand,model.geom('cube').id);r=dict(status='executed_snapshot_joint_and_contact_diagnosis',source_sha256=sha(path),joint_positions=checks,whole_hand_support=loads,guard=g.check(d),guard_details=g.last,devices_opened=0)
    save_json(dest,r);print(json.dumps(dict(nearest_bounds=sorted(checks,key=lambda q:min(q['lower_room_deg'],q['upper_room_deg']))[:6],contacts=loads['by_geom'],guard=g.last)),flush=True)


if __name__=='__main__':main()
