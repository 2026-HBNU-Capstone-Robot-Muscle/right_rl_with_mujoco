"""Add passive contact/state snapshots to a separate device-free replay."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
source=ROOT/'screen_material_checked_reverse_cube_20261029.py';text=source.read_text(encoding='utf-8')
needle='from cube_contact_material_policy_20261029 import apply_material, cube_contact_friction'
assert text.count(needle)==1
text=text.replace(needle,needle+'\nfrom inverse_contact_checkpoint_20261029 import save_checkpoint')
needle="phases=np.asarray([phase_map[str(p)] for p in data['phases']]);"
assert text.count(needle)==1
text=text.replace(needle,needle+"last_phase_indices={str(p):int(np.flatnonzero(phases==p)[-1])+1 for p in set(phases)};")
needle="                if stopped:break\n"
assert text.count(needle)==1
text=text.replace(needle,"                if phase in ('grasp_hold','yaw_hold','remove_hold') and sample==last_phase_indices[phase] and not stopped:save_checkpoint(m,d,out,str(phase),sample)\n"+needle)
(ROOT/'screen_inverse_contact_replay_20261029.py').write_text(text,encoding='utf-8')
print('diagnostic replay built; parent sources unchanged')
