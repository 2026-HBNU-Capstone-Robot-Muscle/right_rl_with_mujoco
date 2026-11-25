"""Read existing workspace assets and exported model references only."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parent
scene=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v4_rewired/proxy_identity/scene.xml'
tree=ET.fromstring(scene.read_bytes())
meshes=[dict(v.attrib) for v in tree.iter('mesh')]
hits=[]
for pattern in ('*distal*','*.stl','*mesh*.npz'):
    for path in ROOT.rglob(pattern):
        if path.is_file() and str(path) not in hits:
            hits.append(str(path))
report=dict(mesh_references=meshes[:16],existing_mesh_files=hits[:80],total_mesh_files=len(hits),devices_opened=False,new_dependencies_installed=False)
(ROOT/'right_hand_rotation_20261027/power_branch_20261027/saved_mesh_inventory.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
