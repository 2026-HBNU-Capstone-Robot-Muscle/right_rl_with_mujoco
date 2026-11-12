import json
from pathlib import Path
OUT=Path(__file__).resolve().parent/'right_hand_rotation_20261024/actual_loaded_review_171607'
for name in ('actual_loaded_contact_hypotheses.json','actual_loaded_finish_candidates_v2.json','actual_loaded_stalled_finish_candidates_v3.json'):
    r=json.loads((OUT/name).read_bytes());print(name,'FRICTION',r.get('original_cube_friction'),r.get('diagnostic_cube_friction'))
    for c in r['cases']:
        print(c['initial_orientation'],c['offset_xy_mm'],c['hypothetical_finish_extra_root_ticks'],round(c['final_rotation_from_settled_deg'],1),round(c['final_target_error_deg'],1),c['external_cube_contact'],round(c['maximum_no_hand_contact_s'],2))
