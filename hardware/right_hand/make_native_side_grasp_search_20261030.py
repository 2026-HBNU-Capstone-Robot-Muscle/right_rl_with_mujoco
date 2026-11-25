from pathlib import Path
R=Path(__file__).resolve().parent
s=(R/'propose_natural_cradle_side_grasp_20261030.py').read_text(encoding='utf-8')
s=s.replace("lower=np.maximum(ql[active]-pd[active],guess[active]-.35);upper=np.minimum(qh[active]-pd[active],guess[active]+.35)","lower=ql[active]-pd[active];upper=qh[active]-pd[active]")
s=s.replace("folder=BASE/f'side_grasp_height_{int(height*1000)}'","folder=BASE/f'native_side_grasp_height_{int(height*1000)}'")
s=s.replace("BASE/'side_grasp_geometry_comparison.json'","BASE/'native_side_grasp_geometry_comparison.json'")
s=s.replace("status='natural_cradle_new_side_contact_grasp_research'","status='natural_cradle_native_range_side_contact_grasp_research'")
(R/'propose_natural_cradle_native_side_grasp_20261030.py').write_text(s,encoding='utf-8')
print('Native motor ranges unchanged; removed only the research trust region around the old unsuitable grasp.')
