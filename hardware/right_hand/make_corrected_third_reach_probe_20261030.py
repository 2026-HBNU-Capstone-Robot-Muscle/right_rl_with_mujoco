from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'search_third_finger_reach_20261030.py').read_text(encoding='utf-8')
s=s.replace("cp=0.\n     for c in d.contact:","cp=0.;third_cp=0.\n     for c in d.contact:")
s=s.replace("if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))", "if cube in (int(c.geom1),int(c.geom2)):\n       depth=max(0.,-float(c.dist)*1000);cp=max(cp,depth);other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)\n       if m.body(int(m.geom_bodyid[other])).name.startswith(finger+'_'):third_cp=max(third_cp,depth)")
s=s.replace("return err,float(top),max(sp.values(),default=0.),max(su.values(),default=0.),cp,q", "return err,float(top),max(sp.values(),default=0.),max(su.values(),default=0.),third_cp,q")
s=s.replace("popsize=9,maxiter=120,tol=.0001", "popsize=14,maxiter=220,tol=.00001")
s=s.replace("'third_finger_reach_review.json'","'third_finger_reach_v2_review.json'")
s=s.replace("cube_initial_center=C.tolist(),", "cube_initial_center=C.tolist(),third_finger_only_cube_penetration_used=True,fixed_other_fingers_not_certified_as_whole_grasp=True,earlier_v1_fixed_other_contact_penalty_not_used=True,")
(R/'search_corrected_third_finger_reach_20261030.py').write_text(s,encoding='utf-8');print('Created isolated third-finger reach probe; v1 whole-grasp inference rejected.')
