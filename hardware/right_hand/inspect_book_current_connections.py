"""Inventory USB serial ports and saved camera-only frame. No motor writes."""
import json
from pathlib import Path
from serial.tools import list_ports
root=Path(__file__).resolve().parent
print(json.dumps([dict(device=p.device,description=p.description,hwid=p.hwid)
    for p in list_ports.comports()],ensure_ascii=False))
pointer=root/'book_camera_confirmation_latest.json'
if pointer.exists():
    folder=Path(json.loads(pointer.read_bytes())['folder'])
    frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
    print(json.dumps(dict(folder=str(folder),last_image=str(folder/'camera'/frames[-1]['image']))))
r=json.loads((root/'book_cube_grasp_trial_20261024_181338_435214/current_start_screen.json').read_bytes())
print(json.dumps(dict(max_initial_support_mm=max(max(s['initial_support_contacts_mm'].values(),default=0.) for s in r['static_screens']),
    dynamic_max_tracking=max(s['max_tracking_rad'] for s in r['dynamic_sensitivity']),
    support_dynamic_clear_at=[s['support_escape_complete_at'] for s in r['dynamic_sensitivity']]),ensure_ascii=False))
