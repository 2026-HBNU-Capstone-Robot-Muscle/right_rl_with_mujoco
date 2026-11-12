from pathlib import Path
root=Path(__file__).resolve().parent
for p in root.glob('*.py'):
    if p.stat().st_size>120000:continue
    if not any(x in p.name for x in ('vision','pose','anchor','camera','marker','replay')):continue
    text=p.read_text(encoding='utf-8',errors='replace')
    lines=[line for line in text.splitlines() if any(k in line for k in ('camera_matrix','calib_path','calib_file','intrinsics','cam_yaml','calib.npz','camera.npz'))]
    if lines:print(p.name,'\n'.join(lines[:4]))
for p in root.parent.glob('work/Leap_Test/**/camera*.y*ml'):print('CAMERA_YAML',p)
for p in root.parent.glob('work/Leap_Test/**/calib*.npz'):print('CALIB_NPZ',p)
