from pathlib import Path
R=Path(__file__).resolve().parent
s=(R/'propose_natural_cradle_side_grasp_20261030.py').read_text(encoding='utf-8')
needle="pd=s['ctrl']-s['qpos'][:16];hand=";assert s.count(needle)==1;s=s.replace(needle,"pd=s['ctrl']-s['qpos'][:16];pd[np.setdiff1d(np.arange(16),active)]=0.;hand=")
s=s.replace("axis=max((0,2),key=lambda i:abs(B[:,i]@np.array([0,1,0])))","axis=0")
s=s.replace("folder=BASE/f'side_grasp_height_{int(height*1000)}'","folder=BASE/f'diagonal_grasp_height_{int(height*1000)}'")
s=s.replace("BASE/'side_grasp_geometry_comparison.json'","BASE/'diagonal_grasp_geometry_comparison.json'")
s=s.replace("status='natural_cradle_new_side_contact_grasp_research'","status='natural_cradle_diagonal_side_contact_grasp_research'")
s=s.replace("if passed:\n   close=", "if passed:\n   assert np.array_equal(goal[np.array([4,5,6,7,12,13,14,15])],graspq[np.array([4,5,6,7,12,13,14,15])]),'Frozen fingers changed'\n   close=")
(R/'propose_natural_cradle_diagonal_grasp_20261030.py').write_text(s,encoding='utf-8');print('Prepared opposing diagonal-face grasp with exact fixed index/ring goals.')
