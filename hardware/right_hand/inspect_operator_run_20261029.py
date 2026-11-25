"""Bounded record inspection; no device imports or writes."""
import json,hashlib,sys
from pathlib import Path
root=Path(__file__).resolve().parent
for name in sys.argv[1:]:
    folder=(root/name).resolve();folder.relative_to(root);path=folder/'result.json';r=json.loads(path.read_bytes());samples=r.get('samples',[])
    info={key:r.get(key) for key in ('status','error','failed_phase','all_torque_off_verified','settings_restored','hardware_authorized','cube_present',
        'cube_placement_confirmed','cube_removal_confirmed','final_off_reference_valid','accepted_enable_goal','placement_photo','grasp_observation_photo','rotation_observation_photo','camera_recording')}
    info.update(folder=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),samples=len(samples),write_events=len(r.get('write_events',[])),
        phase_counts={p:sum(s['phase']==p for s in samples) for p in dict.fromkeys(s['phase'] for s in samples)},
        minimum_voltage_v=min((min(s['voltage_v']) for s in samples),default=None),
        peak_current_ma=max((max(abs(c) for c in s['current_ma']) for s in samples),default=None),
        maximum_tracking_rad=max((max(s['tracking_rad']) for s in samples),default=None),read_failures=r.get('read_failures',[]),
        last_sample_phase=samples[-1]['phase'] if samples else None,keys=list(r))
    print(json.dumps(info,ensure_ascii=False))
