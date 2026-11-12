from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_natural_cradle_forward_20261030.py').read_text(encoding='utf-8')
s=s.replace("O=BASE/'retargeted_forward'","O=BASE/'retargeted_grasp_native'").replace("end=int(source['forward_end_sample']);goals=[]","end=int(np.sum(np.isin(source['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])));goals=[]")
s=s.replace("status='natural_recess_forward_task_frame_retarget_research_only'","status='natural_recess_grasp_prefix_native_ranges_research'")
s=s.replace("assert np.max", "assert np.max")
(R/'propose_natural_grasp_prefix_20261030.py').write_text(s,encoding='utf-8');print('Grasp prefix only; no failed yaw section is reused.')
