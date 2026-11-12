import json
from pathlib import Path
R=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff'
d=json.loads((R/'review.json').read_bytes());v=d['dynamic'];print(json.dumps(dict(yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],orientation=d['orientation_error_deg'],contacts=v['final_contact_N'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],max_seam_qpos_error=max(c['maximum_qpos_error'] for c in d['source_checkpoint_state_comparisons']),load_cases=[{k:c[k] for k in ('hypothetical_reaction_torque_nm','max_yaw_change_deg','max_center_shift_mm','max_tilt_change_deg','minimum_separating_gap_1ms_mm','reasons')} for c in d.get('hypothetical_load_cases',[])])),flush=True)
