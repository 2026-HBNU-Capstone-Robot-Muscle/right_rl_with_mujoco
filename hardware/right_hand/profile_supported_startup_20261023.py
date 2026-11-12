"""Profile pure planning from a saved request. No motor/camera construction."""
import argparse
import json
from pathlib import Path
import time
from supported_cube_rotation_plan import SupportedCubeRotationPlan
from leap_thumb_prefix_empty import Bus
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'cube_supported_rotation_trial_20261023_033108_297056/fresh_off_request.json'

def timed(fn):
    start=time.perf_counter();value=fn();return value,(time.perf_counter()-start)*1000

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--label',default='before');args=parser.parse_args()
    p,ctor=timed(lambda:SupportedCubeRotationPlan(SOURCE));raw=p.source_position();rows=[]
    for _ in range(5):
        b=object.__new__(Bus);b.plan_module=p
        _,bind=timed(lambda:b.bind(raw));b.pending.pop(0)
        _,adopt=timed(lambda:b.adopt(raw));goals,plan=timed(lambda:p.plan(raw))
        rows.append(dict(bind_ms=bind,adopt_ms=adopt,second_plan_ms=plan,post_enable_compute_ms=adopt+plan,goals=len(goals)))
    record=dict(constructor_off_ms=ctor,iterations=rows,hardware_commands_sent=0,real_camera_or_serial_opened=False)
    target=ROOT/'right_hand_rotation_20261023'/f'startup_profile_{args.label}.json'
    target.write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps(record,indent=2))

if __name__=='__main__':main()
