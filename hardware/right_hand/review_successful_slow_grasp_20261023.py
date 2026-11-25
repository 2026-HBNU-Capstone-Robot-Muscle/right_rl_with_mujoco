"""Recorded ON grasp evidence and same-frame direct marker/encoder diagnostics."""
import hashlib
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from cube_marker_readiness import make_detector,inspect_frame
from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions
from analyze_readonly_direct_pose_20261023 import analyze_grid

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'
HOLD_PHASES={'stage_4_fixed_grasp_observation','stage_2_fixed_hold','grasp_observation'}


def main():
    blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob);samples=r['samples']
    assert r['status']=='slow_grasp_demo_complete' and r['all_torque_off_verified'] and r['settings_restored']
    assert r['cube_placement_confirmed'] and r['cube_removal_confirmed']
    hold=[s for s in samples if s['phase']=='stage_4_fixed_grasp_observation']
    assert len(hold)==60 and len({tuple(s['goal']) for s in hold})==1
    phases={}
    for s in samples:phases[s['phase']]=phases.get(s['phase'],0)+1
    empty=json.loads((ROOT/'supported_yaw_empty_20261023_030552_675209/result.json').read_bytes())
    a=[s for s in samples if s['phase']=='stage_3_loaded_approach']
    b=[s for s in empty['samples'] if s['phase']=='stage_3_grasp_rehearsal_EMPTY']
    assert len(a)==len(b)==322 and np.array_equal([s['goal'] for s in a],[s['goal'] for s in b])
    load=[]
    for i in range(16):
        currents=np.array([s['current_ma'][i] for s in a[-30:]])
        prior=np.array([s['current_ma'][i] for s in b[-30:]])
        load.append(dict(motor_id=i+1,tail_loaded_median_ma=float(np.median(currents)),
            prior_empty_median_ma=float(np.median(prior)),signed_difference_ma=float(np.median(currents-prior))))
    evidence=dict(status='slow_grasp_command_and_hold_complete',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),phases=phases,
        hold_same_goal_samples=60,hold_sample_span_s=hold[-1]['t']-hold[0]['t'],
        hold_median_cycle_s=float(np.median(np.diff([s['t'] for s in hold]))),
        hold_actual_peak_to_peak_ticks=np.ptp([s['actual'] for s in hold],axis=0).tolist(),
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_absolute_current_ma=max(max(abs(x) for x in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        hardware_errors_observed=any(any(s['error']) for s in samples),
        all_torque_off_verified=True,settings_restored=True,camera_frames=r['camera_recording']['frame_count'],
        exact_goal_empty_comparison_samples=322,current_comparison=load,
        contact_forces_verified=False,physical_grasp_strength_or_lift_verified=False,
        physical_cube_rotation_verified=False,live_sac=False,assistant_opened_real_devices=False)
    (FOLDER/'completion_review.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    reg=candidate();Rwm=np.array(reg['hand_to_world_candidate']);gm=np.array(reg['grasp_site_in_marker_m_candidate'])
    detector=make_detector();timestamps=np.array([s['t'] for s in samples]);rows=[];hypotheses=[];rejected=[]
    recording=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    selected=[f for f in recording if f['phase'] in HOLD_PHASES]
    for f in selected:
        image_blob=(FOLDER/'camera'/f['image']).read_bytes()
        frame=cv.imdecode(np.frombuffer(image_blob,np.uint8),cv.IMREAD_COLOR)
        quality=inspect_frame(frame,detector);stamp=f['camera_read_monotonic']-r['enabled_at']
        index=int(np.searchsorted(timestamps,stamp));reason=None
        if not 0<index<len(samples):reason='no_bracketing_readings'
        if reason is None:
            before,after=samples[index-1],samples[index]
            gap=max(stamp-before['t'],after['t']-stamp)
            drift=int(np.max(abs(np.array(after['actual'])-before['actual'])))
            if gap>.12 or drift>4:reason='encoder_pair_gap_or_motion'
            elif any(s['phase'] not in HOLD_PHASES or s['goal']!=hold[-1]['goal'] or
                     s['torque']!=[1]*16 or any(s['error']) or s['watchdog']!=[25]*16
                     for s in (before,after)):reason='not_healthy_fixed_ON_hold'
        if quality['ids'].count(40)!=1 or quality['ids'].count(5)!=1:
            reason=reason or 'palm40_and_cube5_not_directly_decoded'
        if reason:
            rejected.append(dict(image=f['image'],reason=reason,decoded_ids=quality['ids']));continue
        corners={str(int(i)):np.asarray(c).reshape(4,2).tolist()
                 for i,c in zip(detector.last_ids.ravel(),detector.last_corners)}
        poses40=marker_solutions([np.array(corners['40'])],[40],40,.0282,K,dist)
        poses5=marker_solutions([np.array(corners['5'])],[5],5,.0107,K,dist)
        nearest=min((before,after),key=lambda s:abs(s['t']-stamp))
        row=dict(image='camera/'+f['image'],image_sha256=hashlib.sha256(image_blob).hexdigest(),
            camera_read_monotonic=f['camera_read_monotonic'],phase=f['phase'],marker_corners=corners,
            marker_minimum_edge_px=quality['minimum_edge_px'],
            meets_24px_readonly_capture_guidance=quality['readable_pair'],
            before=before,after=after,maximum_encoder_gap_s=gap,endpoint_drift_ticks=drift,
            measured_motor_ticks=nearest['actual'],nearest_encoder_dt_s=nearest['t']-stamp)
        try:grid,_=analyze_grid(frame,corners['5'],K,dist)
        except ValueError as exc:grid=dict(error=str(exc))
        row['grid_diagnostic']=grid;rows.append(row)
        for (ip,(Rp,tp,ep)),(ic,(Rc,tc,ec)) in itertools.product(enumerate(poses40),enumerate(poses5)):
            Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
            center=tc-Rc[:,2]*.0275
            hypotheses.append(dict(image=row['image'],source='direct_ON_ID40_and_ID5',palm_branch=ip,cube_branch=ic,
                marker_errors_px=[ep,ec],cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),
                cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist(),
                world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                measured_motor_ticks=nearest['actual'],nearest_encoder_dt_s=nearest['t']-stamp))
        if 'grid_rotation_camera' in grid:
            Rg=np.array(grid['grid_rotation_camera']);tg=np.array(grid['grid_plane_center_camera_m'])
            normal=Rg[:,2].copy()
            if normal@tg>0:normal*=-1;Rg=Rg@np.diag([1.,-1.,-1.])
            center=tg-normal*.0275
            for ip,(Rp,tp,ep) in enumerate(poses40):
                Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
                hypotheses.append(dict(image=row['image'],source='direct_ON_ID40_and_solved_grid_diagnostic',
                    palm_branch=ip,cube_branch=None,marker_errors_px=[ep,grid['grid_pnp_mean_error_px']],
                    cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),
                    cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rg).tolist(),
                    world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                    measured_motor_ticks=nearest['actual'],nearest_encoder_dt_s=nearest['t']-stamp))
    result=dict(status='same_run_ON_encoder_marker_diagnostic',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),selected_fixed_hold_frames=len(selected),
        directly_paired_ON_frames=len(rows),rows=rows,rejected=rejected,hypotheses=hypotheses,
        marker_sizes_mm=dict(palm40=28.2,cube5=10.7),cube_side_mm=55,
        marker_to_model_candidate=reg,marker_to_model_registration_verified=False,
        old_OFF_angles_used=False,hardware_execution_allowed=False,assistant_opened_real_devices=False,
        limitations=['Camera driver-read times are not synchronized exposure times.',
            'Both planar IPPE pose branches are retained.',
            'Direct top markers are about 20px on their short edge; diagnostic only, below the 24px capture guidance.',
            'The equal green tile grid is diagnostic and may be occluded.',
            'Current differences between separate runs are not calibrated contact forces.'])
    (FOLDER/'on_paired_pose_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in evidence.items() if k!='current_comparison'},indent=2))
    print('DIRECT ON PAIRS',len(rows),'/',len(selected),'hypotheses',len(hypotheses))
    print('GRID SUCCESSES',sum('grid_rotation_camera' in row['grid_diagnostic'] for row in rows))
    print('LOAD COMPARISON',json.dumps(load))


if __name__=='__main__':main()
