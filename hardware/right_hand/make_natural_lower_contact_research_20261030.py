from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_lower_body_regrasp_20261030.py').read_text(encoding='utf-8')
s=s.replace("O=R/'right_hand_rotation_20261030/lower_body_regrasp';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'","O=R/'right_hand_rotation_20261030/natural_grasp_yaw/lower_body_contact';CP=R/'right_hand_rotation_20261030/natural_grasp_yaw/checkpoint'")
s=s.replace("forward_checkpoint.mjb","grasp_checkpoint.mjb").replace("forward_checkpoint.npz","grasp_checkpoint.npz").replace("R/'right_hand_rotation_20261030/single_forward/candidate.npz'","R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz'")
s=s.replace("status='post_turn_lower_body_regrasp_geometry_only'","status='post_natural_grasp_lower_body_contact_geometry_only'")
(R/'propose_natural_lower_contact_20261030.py').write_text(s,encoding='utf-8');print('New geometry source targets the current grasp, not the old yaw posture.')
