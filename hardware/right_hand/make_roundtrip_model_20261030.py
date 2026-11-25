"""Derive a separate two-leg diagnostic; preserve the original frozen model."""
from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_material_checked_reverse_cube_20261029.py').read_text(encoding='utf-8')
changes=[
('"""Whole loaded grasp/yaw/open physics review; no robot or camera access."""','"""Continuous forward/inverse loaded review; no robot or camera access."""'),
("q0=d.qpos[-4:].copy();R0=Rotation.from_quat(q0[[1,2,3,0]]);q1=(R0*Rotation.from_rotvec(np.array([0.,-1.,0.])*np.pi/2)).as_quat();d.qpos[-4:]=q1[[3,0,1,2]];d.qvel[:]=0","d.qvel[:]=0"),
("whole_turn=None", "whole_turn=None;forward_whole_turn=None;inverse_delta=None;forward_end=int(data['forward_end_sample'])"),
("                for height in np.arange(upper,-.025,-.0005):", "                if hand_depth(upper)>1e-5:raise RuntimeError('Upper seating endpoint already intersects hand; reject vertical seating')\n                for height in np.arange(upper,-.025,-.0005):"),
("if stopped:break\n            R=", "if stopped:break\n            R="),
("turn_tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(yawB@np.array([0.,-1,0])),-1,1))))", "turn_tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(yawB@np.array([0.,-1,0])),-1,1))))\n                    if sample<=forward_end:forward_whole_turn=whole_turn\n                    elif forward_whole_turn is not None:inverse_delta=whole_turn-forward_whole_turn"),
("row=dict(material_audit=material", "row=dict(vertical_seating_line_upper_verified_free=bool(args.seat_up),forward_from_placement_deg=forward_whole_turn,inverse_from_forward_hold_deg=inverse_delta,material_audit=material"),
("cube_initial_local_green_axis_turn_deg=None if args.cube_seed_json else 90", "cube_initial_local_green_axis_turn_deg=0"),
("status='full_loaded_yaw_release_model_diagnostic'", "model_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),status='continuous_forward_inverse_full_loaded_model_diagnostic'"),
]
for a,b in changes:
 if s.count(a)!=1:raise RuntimeError((a,s.count(a)))
 s=s.replace(a,b)
(R/'screen_yaw_roundtrip_model_20261030.py').write_text(s,encoding='utf-8')
compile(s,'screen_yaw_roundtrip_model_20261030.py','exec')
print('Separate roundtrip model created. Original GREEN-up / ORANGE-wristward orientation retained.')
