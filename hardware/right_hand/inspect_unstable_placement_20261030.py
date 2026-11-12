import json
from pathlib import Path
R=Path(__file__).resolve().parent;out=R/'right_hand_rotation_20261030/stable_support_research';out.mkdir(exist_ok=True);rows=[]
for name in ('placement_alignment_capture_20261030_100707_371180','placement_alignment_capture_20261030_101046_586473'):
 f=R/name;r=json.loads((f/'result.json').read_bytes());frames=json.loads((f/'camera/frames.json').read_bytes())['frames'];phases={}
 for frame in frames:phases[frame['phase']]=phases.get(frame['phase'],0)+1
 row={k:r.get(k) for k in ('status','error','phase','all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_removal_confirmed','cube_present','placement_photo','prepared_empty_photo','post_placement_goal_writes')};row.update(folder=name,frames=len(frames),phases=phases,last_frame=frames[-1] if frames else None,sample_keys=list(r.get('samples',[{}])[0]),last_off_reference_valid=r.get('final_off_reference_valid'))
 # Save representative PLACE frames, including the final portion before exit.
 placed=[v for v in frames if v['phase']=='place_cube'];selected=placed[::max(1,len(placed)//8)][:8]+placed[-1:];row['placement_frames']=[str(f/'camera'/v['image']) for v in selected];rows.append(row)
(out/'latest_capture_review.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(rows,ensure_ascii=False,indent=2))
