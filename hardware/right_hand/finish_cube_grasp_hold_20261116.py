"""Bind existing recovery evidence only to the identical new prefix."""
import json
from pathlib import Path
import cube_grasp_hold_20261116 as c
source=c.ROOT/'right_hand_rotation_20261116/final_grip_attempt_review/escape_verified_v2/empty_recovery_screen/review.json'
r=c.make_review(c.Plan(c.REQUEST),json.loads(source.read_bytes()))
c.PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
c.validate_review(c.Plan(c.REQUEST),c.PROOF)
print(json.dumps(dict(status=r['status'],goals=len(c.Plan(c.REQUEST).plan(c.Plan(c.REQUEST).source_position())),
    long_hold_safety_cases=len(r['loaded_hold_review']['cases']),long_hold_task_verified=r['long_hold_task_verified'],
    physical_grasp_verified=False,devices_opened=False)))
