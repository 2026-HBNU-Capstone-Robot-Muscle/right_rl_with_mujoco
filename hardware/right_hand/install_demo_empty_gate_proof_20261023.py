"""Validate native output before selecting it as offline regression evidence."""
import argparse
import hashlib
import json
from pathlib import Path
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--suffix',default='')
    parser.add_argument('--latest-fixture',default='slow_grasp_demo_after_open_fixture')
    args=parser.parse_args();reviews=[]
    for name,outname in (
        ('slow_grasp_demo_fixture','native_screen_empty_start_gate'+args.suffix+'.json'),
        (args.latest_fixture,'native_screen'+args.suffix+'.json')):
        folder=ROOT/'right_hand_rotation_20261023'/name
        path=folder/outname;plan=SlowGraspDemoPlan(folder/'fresh_off_request.json')
        proof=validate_proof(plan,path)
        if name=='slow_grasp_demo_fixture':(folder/'native_screen.json').write_bytes(path.read_bytes())
        reviews.append(dict(fixture=name,accepted=proof['accepted'],static_cases=len(proof['static_screens']),
            dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
            proof_path=str(path.relative_to(ROOT)),proof_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (ROOT/'right_hand_rotation_20261023/demo_empty_start_native_review.json').write_text(
        json.dumps(dict(fixtures=reviews,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print(json.dumps(reviews,indent=2))


if __name__=='__main__':main()
