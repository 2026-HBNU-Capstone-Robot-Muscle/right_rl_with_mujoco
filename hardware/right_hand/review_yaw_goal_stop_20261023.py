"""Stored record of the exact goal mismatch and its preceding read recovery."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_222816_628099'


def main():
    blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob)
    e=r['goal_readback_events'][0];samples=r['samples']
    command=np.array(e['command']);previous=np.array(e['previous_command'])
    assert e['initial_readback']==e['previous_command']
    assert len(e['observations'])==3 and all(x==e['previous_command'] for x in e['observations'])
    assert np.max(abs(command-previous))<=2
    failure_samples=[s for s in samples if s['goal']==e['command']]
    assert len(failure_samples)==4
    report=dict(source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        status='stopped_before_rotation_persistent_exact_previous_goal',
        error=r['error'],failed_phase=r['failed_phase'],
        phase_sample_counts=dict(Counter(s['phase'] for s in samples)),
        issued_command=e['command'],exact_previous_command=e['previous_command'],
        unchanged_old_goal_reads=4,maximum_command_step_ticks=int(np.max(abs(command-previous))),
        confirmation_elapsed_ms=e['elapsed_ms'],confirmation_goal_writes=e['goal_writes'],
        last_read_recovered_immediately_before_missing_goal=r['read_failures'][-1],
        transport_application_failure_suspected=True,physical_communication_root_cause_proven=False,
        preceding_recovered_read_necessarily_caused_missing_goal=False,
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_absolute_current_ma=max(max(abs(x) for x in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        camera_requested_exposure=r['camera_requested_exposure'],
        camera_frames=r['camera_recording']['frame_count'],
        rotation_command_path_completed=False,physical_rotation_verified=False,
        source_images_or_results_modified=False,real_devices_opened=False,
        proposed_runtime_change='After a recovered read, hold goals and require two new complete healthy exact-goal snapshots within the existing 200ms sample budget before advancing.',
        missing_goal_packet_still_causes_stop=True,goal_resend_allowed=False)
    (FOLDER/'goal_stop_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('issued_command','exact_previous_command','phase_sample_counts')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
