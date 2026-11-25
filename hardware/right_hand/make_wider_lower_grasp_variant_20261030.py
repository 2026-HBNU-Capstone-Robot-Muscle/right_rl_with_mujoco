from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_lower_three_contact_grasp_20261030.py').read_text(encoding='utf-8')
replacements={"O=R/'right_hand_rotation_20261030/lower_three_contact_grasp';CP=O/'initial_checkpoint'":"O=R/'right_hand_rotation_20261030/lower_three_contact_grasp/wider_center';CP=R/'right_hand_rotation_20261030/lower_three_contact_grasp/initial_checkpoint'",
"O.mkdir(exist_ok=True)":"O.mkdir(parents=True,exist_ok=True)","[-.008,-.008,0.]":"[-.03,-.03,0.]","[.008,.008,.003]":"[.03,.03,.003]","out.extend(np.array(row['normal_residual'])*8)":"out.extend(np.array(row['normal_residual'])*0)","max_nfev=220":"max_nfev=300"}
for a,b in replacements.items():assert a in s,a;s=s.replace(a,b)
s=s.replace("virtual_cube_used_only_in_geometry_design=True,","virtual_cube_used_only_in_geometry_design=True,contact_normal_alignment_is_soft_design_preference_disabled=True,center_search_planar_mm=30,")
(R/'propose_wider_lower_grasp_20261030.py').write_text(s,encoding='utf-8');print('Created wider-centre design hypothesis; original failed searches preserved.')
