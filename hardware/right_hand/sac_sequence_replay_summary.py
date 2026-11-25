"""Compact summary of the offline sequence replay."""

import json
from pathlib import Path

path = Path(__file__).resolve().parent / 'sac_sequence_replay_sim_result.json'
report = json.loads(path.read_text())
rows = report['candidate_results']
print('candidates:', len(rows), 'changed face:', sum(r['face_changed'] for r in rows))
print('max stage-end tracking rad:', max(r['max_stage_end_tracking_error_rad'] for r in rows))
print('median stage-end tracking rad:', sorted(r['max_stage_end_tracking_error_rad'] for r in rows)[len(rows)//2])
print('best candidate stage-end:', [(r['stage'], r['face_up'],
                                    round(r['max_joint_tracking_error_rad'], 3),
                                    r['cube_contact_count'])
                                   for r in rows[0]['stage_endpoints']])
