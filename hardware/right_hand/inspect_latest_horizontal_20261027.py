"""Read the pasted operator log and bounded saved-result summaries, no devices."""
import json
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parent
attachment=Path('C:/Users/김준서/.codex/attachments/94c01f5c-e867-4fa5-a975-b2558ed25ce4/Pasted text.txt')
if attachment.is_file():
 print('PASTED LOG\n'+attachment.read_text(encoding='utf-8-sig')[:22000])
for folder in sorted(ROOT.glob('horizontal_quarter_trial_20261027_*'))[-2:]:
 r=json.loads((folder/'result.json').read_bytes());samples=r.get('samples',[])
 out={k:r.get(k) for k in ('status','error','failed_phase','current_start_screen_accepted','all_torque_off_verified','settings_restored','cube_present','cube_placement_confirmed','cube_removal_confirmed','off_preload_confirmation','off_preload_snapshots','enable_snapshots','fixed_hold_fault','progress_fault','current_start_screen','final_off_reference_valid') if k in r}
 # Keep only the fields needed to compare OFF and enable readings.
 for name in ('off_preload_snapshots','enable_snapshots'):
  if name in out:out[name]=[{k:s[k] for k in ('position','goal','torque','error','watchdog','voltage') if k in s} for s in out[name]]
 out.update(folder=folder.name,samples=len(samples),phases=dict(Counter(s.get('phase') for s in samples)),write_events=len(r.get('write_events',[])))
 if samples:out['last_sample']=samples[-1]
 camera=r.get('camera_recording',{});out['camera']={k:camera.get(k) for k in ('status','frame_count','image_signal_usable_frames','image_signal_blank_frames','error')}
 print(json.dumps(out,ensure_ascii=False))
