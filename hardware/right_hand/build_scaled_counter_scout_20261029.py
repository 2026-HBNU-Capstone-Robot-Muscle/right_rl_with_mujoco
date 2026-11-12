from pathlib import Path
root=Path(__file__).resolve().parent;source=(root/'scout_counter_yaw_paths_20261029.py').read_text(encoding='utf-8')
changes={
    "counter_yaw_contact_scout'":"counter_yaw_scaled_scout'",
    "experiments=[('thumb_axial',[2]),('middle_side',[9]),('thumb_and_middle_side',[2,9])]":"experiments=[('middle_side07',[9]),('both07',[2,9])]",
    "turn=yaw.copy();turn[:,indices]=2*base[indices]-turn[:,indices]":"turn=yaw.copy();turn[:,indices]=2*base[indices]-turn[:,indices]\n        if 9 in indices:turn[:,9]=np.rint(base[9]-.7*(yaw[:,9]-base[9])).astype(int)"
}
for old,new in changes.items():
    assert source.count(old)==1;source=source.replace(old,new)
path=root/'scout_scaled_counter_yaw_20261029.py'
if path.exists():raise RuntimeError('Refuse to overwrite scaled scout')
path.write_text(source,encoding='utf-8')
