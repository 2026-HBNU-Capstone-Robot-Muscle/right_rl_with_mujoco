from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8');anchor='parser.add_argument(\'--candidate\',required=True);';assert anchor in s;s=s.replace(anchor,"parser.add_argument('--tip-friction',type=float,choices=(.7,1.),required=True);"+anchor)
anchor='material=apply_material(m,friction,hand,cube)';assert anchor in s
s=s.replace(anchor,anchor+"\n            for tip_name in ('if_tip','mf_tip','rf_tip','th_tip'):m.geom_friction[m.geom(tip_name).id,0]=args.tip_friction\n            material.update(hypothetical_tip_friction=args.tip_friction,current_hard_plastic_hardware_not_changed=True,added_pad_thickness_not_modelled=True,hardware_admission_prohibited=True)")
(R/'screen_tip_material_hypothesis_20261104.py').write_text(s,encoding='utf-8');print('Separate material hypothesis; never admitted as current-hardware evidence.')
