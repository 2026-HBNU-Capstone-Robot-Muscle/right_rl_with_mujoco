from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'search_cube_rotation_ik_20261022.py').read_text(encoding='utf-8')
assert source.count("OUT/'rigid_yaw_ik_seeds.json'")==1
source=source.replace("OUT/'rigid_yaw_ik_seeds.json'","OUT/'rigid_yaw_lift_ik_seeds.json'")
path=root/'search_cube_rotation_lift_20261022.py'
path.write_text(source,encoding='utf-8')
compile(source,str(path),'exec')
print(path.name)
