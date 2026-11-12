"""Build a separate research harness; do not mutate a running/reviewed model."""
import hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parent
source=ROOT/'screen_reverse_loaded_cube_20261029.py'
text=source.read_text(encoding='utf-8')
needle="from sac_cube_model import set_cube_physics"
assert text.count(needle)==1
text=text.replace(needle,needle+'\nfrom cube_contact_material_policy_20261029 import apply_material, cube_contact_friction')
needle="m.geom_friction[cube,0]=friction"
assert text.count(needle)==1
text=text.replace(needle,"material=apply_material(m,friction,hand,cube)")
needle="B=d.xmat[m.body('cube').id].reshape(3,3).copy();"
assert text.count(needle)==1
text=text.replace(needle,"material['effective_contacts_after_settle']=cube_contact_friction(m,d,hand,cube);"+needle)
needle="row=dict(seating_height_shift_mm="
assert text.count(needle)==1
text=text.replace(needle,"row=dict(material_audit=material,seating_height_shift_mm=")
needle="result=dict(simulation_timestep_s="
assert text.count(needle)==1
text=text.replace(needle,"result=dict(material_policy='both_contacting_surfaces_v1_not_physical_identification',parent_model_sha256='"+
    hashlib.sha256(source.read_bytes()).hexdigest()+"',simulation_timestep_s=")
target=ROOT/'screen_material_checked_reverse_cube_20261029.py'
target.write_text(text,encoding='utf-8')
print(target.name,hashlib.sha256(target.read_bytes()).hexdigest())
