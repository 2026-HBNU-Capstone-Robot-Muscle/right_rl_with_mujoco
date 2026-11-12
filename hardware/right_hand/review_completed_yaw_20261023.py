"""Saved frames only. Baseline/turn/hold review, strictly before removal."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_234707_886202'
BASE='stage_4_fixed_grasp_observation'
TURN='stage_5_grasped_small_yaw'
AFTER='stage_6_yaw_fixed_observation'


def load(row):
    a=cv.imdecode(np.frombuffer((FOLDER/'camera'/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    if a is None or a.shape!=(720,1280,3):raise ValueError('Bad saved frame')
    return a


def main():
    blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob)
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    cutoff=r['rotation_evidence_cutoff_camera_monotonic']
    phases={phase:[f for f in frames if f['phase']==phase and f['camera_read_monotonic']<=cutoff]
            for phase in (BASE,TURN,AFTER)}
    assert r['rotation_command_path_completed'] and r['all_torque_off_verified'] and r['settings_restored']
    assert all(phases.values())
    counts=Counter(s['phase'] for s in r['samples'])
    assert [counts[p] for p in (BASE,TURN,AFTER)]==[60,90,40]
    chosen=[phases[BASE][0],phases[BASE][-1],phases[TURN][len(phases[TURN])//2],
            phases[TURN][-1],phases[AFTER][0],phases[AFTER][-1]]
    panels=[]
    for row in chosen:
        panel=cv.resize(load(row),(640,360))
        label=row['phase'].replace('stage_4_fixed_grasp_observation','BASE').replace(TURN,'TURN').replace(AFTER,'AFTER')
        label+=f" {row['camera_read_monotonic']-r['enabled_at']:.3f}s"
        cv.rectangle(panel,(0,0),(640,30),(0,0,0),-1)
        cv.putText(panel,label,(8,22),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        panels.append(panel)
    sheet=np.vstack((np.hstack(panels[:3]),np.hstack(panels[3:])))
    ok,encoded=cv.imencode('.jpg',sheet,[cv.IMWRITE_JPEG_QUALITY,95]);assert ok
    (FOLDER/'completed_yaw_contact_sheet.jpg').write_bytes(encoded.tobytes())
    for phase,name in ((BASE,'baseline_last.jpg'),(AFTER,'after_last.jpg')):
        (FOLDER/name).write_bytes((FOLDER/'camera'/phases[phase][-1]['image']).read_bytes())
    selected=[f for f in frames if phases[BASE][0]['camera_read_monotonic']<=f['camera_read_monotonic']<=cutoff]
    assert all(f['phase']!='remove_cube' for f in selected)
    stamps=np.array([f['camera_read_monotonic'] for f in selected]);timeline=np.arange(stamps[0],stamps[-1],1/15.)
    previous=Path.cwd()
    try:
        os.chdir(FOLDER);name='grasped_small_yaw_before_removal.mp4'
        writer=cv.VideoWriter(name,cv.VideoWriter_fourcc(*'mp4v'),15.,(1280,720));assert writer.isOpened()
        try:
            for stamp in timeline:
                index=max(0,int(np.searchsorted(stamps,stamp,side='right')-1));writer.write(load(selected[index]))
        finally:writer.release()
        decoder=cv.VideoCapture(name)
        try:
            assert decoder.isOpened();n=int(decoder.get(cv.CAP_PROP_FRAME_COUNT));ok,a=decoder.read()
            assert n==len(timeline) and ok and a.shape==(720,1280,3)
        finally:decoder.release()
    finally:os.chdir(previous)
    before=[s for s in r['samples'] if s['phase']==BASE][-20:]
    after=[s for s in r['samples'] if s['phase']==AFTER][-20:]
    review=dict(source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        status='all_small_turn_commands_and_after_hold_completed_user_Q_at_removal',
        main_status=r['status'],main_error=r.get('error'),failed_phase=r.get('failed_phase'),
        command_counts=dict(Counter(s['phase'] for s in r['samples'])),
        before_removal_frame_counts={p:len(v) for p,v in phases.items()},cutoff_monotonic=cutoff,
        actual_joint_change_ticks=(np.median([s['actual'] for s in after],axis=0)-np.median([s['actual'] for s in before],axis=0)).tolist(),
        goal_change_ticks=(np.array(after[-1]['goal'])-np.array(before[-1]['goal'])).tolist(),
        read_failures=r.get('read_failures',[]),read_recovery_confirmations=r.get('read_recovery_confirmation_events',[]),
        goal_readback_events=r.get('goal_readback_events',[]),
        minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),
        maximum_absolute_current_ma=max(abs(x) for s in r['samples'] for x in s['current_ma']),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        camera_recording=r['camera_recording'],camera_restored=r['camera_exposure_restoration_verified'],
        full_turn_commands_complete=True,physical_rotation_verified=False,live_SAC=False,
        video='grasped_small_yaw_before_removal.mp4',duration_s=len(timeline)/15.,
        contact_sheet='completed_yaw_contact_sheet.jpg',source_images_modified=False,
        removal_or_OFF_movement_used_as_success_evidence=False,real_devices_opened=False,
        visual_verdict='pending_before_and_after_review')
    (FOLDER/'completed_yaw_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(review,ensure_ascii=False,indent=2))
    print('FRAME_EXAMPLE',chosen[1])


if __name__=='__main__':main()
