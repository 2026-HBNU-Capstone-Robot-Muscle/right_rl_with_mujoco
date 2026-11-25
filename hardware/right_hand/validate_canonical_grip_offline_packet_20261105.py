"""Validate source bindings and local media in the handoff packet, no devices."""
import ast,hashlib,json,subprocess,math
import numpy as np
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import unquote,urlsplit
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
class Links(HTMLParser):
    def __init__(self):super().__init__();self.links=[]
    def handle_starttag(self,tag,attrs):
        for name,value in attrs:
            if name in ('href','src') and value:self.links.append(value)
def main():
    packet=json.loads((O/'offline_review.json').read_bytes());checks=[]
    candidate=R/packet['current_offline_candidate'];assert sha(candidate)==packet['candidate_sha256'];checks.append('candidate hash')
    for kind in ('source_documents_sha256','implementation_sha256'):
        for p,h in packet[kind].items():assert sha(R/p)==h,p
        checks.append(kind)
    assert packet['hardware_execution_allowed'] is False and packet['devices_opened'] is False and packet['assistant_motor_writes']==0;checks.append('offline scope')
    assert packet['physical_target90_verified'] is False and packet['new_physical_grip_verified'] is False;checks.append('no false physical completion')
    assert packet['full_model_matrix']['cases']==27 and packet['hand_only']['cases']==9 and packet['initial_placement_sensitivity']['cases']==27;checks.append('complete matrix sizes')
    assert packet['initial_placement_sensitivity']['task_passed']==25 and len(packet['initial_placement_sensitivity']['failed_cases'])==2;checks.append('placement failures retained')
    assert packet['hypothetical_reaction']['loaded_cases']==18 and packet['hypothetical_reaction']['loaded_filter_passed']==14;checks.append('reaction failures retained')
    assert packet['face_hold_hypotheses']['cases']==6 and packet['face_hold_hypotheses']['independent_initial_states_not_reorientation_paths'];checks.append('face holds distinguished from rotation paths')
    assert packet['articulated_upper_layer_hypotheses']['cases']==6 and packet['articulated_upper_layer_hypotheses']['passed']==6 and packet['articulated_upper_layer_hypotheses']['not_whole_cube_reorientation90'];checks.append('upper layer load result distinguished from whole cube90')
    assert packet['new_valid_whole_cube90_path'] is True;current=packet['current_nominal_task_candidate'];raw=json.loads((O/current['source']).read_bytes());v=raw['dynamic'];tail=v['settled_tail'];z=np.load(candidate)
    assert raw['quarter_and_handoff_passed'] and raw['reasons']==[] and v['safety_passed'] and v['completed_goals']==v['total_goals']==6166
    assert raw['all_cube_dynamics_continuous_no_resets'] and raw['all_goal_seams_exact'] and raw['orientation_error_deg']<5 and 85<v['final_yaw_from_placement_deg']<95
    assert tail['minimum_separating_plane_gap_1ms_mm']>.3 and tail['certificate']['queries']==4050 and tail['separation_certificate_bad_steps_1ms']==0 and tail['certificate_unqueried_steps_after_first_failure']==0 and tail['opposed_lower_body_contact_fraction']>.8
    assert len(z['goals'])==6166 and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2 and np.array_equal(z['goals'][-134:],np.tile(z['goals'][-1],(134,1)))
    assert all(x['maximum_qpos_error']==x['maximum_qvel_error']==x['maximum_ctrl_error']==0 for x in raw['source_checkpoint_state_comparisons'])
    assert current['nominal_cases']==1 and current['exact90_not_proved'] and not current['new_robustness_matrix_not_run'] and current['full_combined_robustness_matrix_not_run'] and not packet['next_rotation_from_new_grip_verified'];checks.append('new nominal quarter and handoff full replay distinguished from exact90 and real completion')
    sensitivity=current['one_factor_sensitivity'];assert sensitivity['candidate_sha256']==packet['candidate_sha256'] and sensitivity['total_additional_cases']==14 and sensitivity['one_factor_only_not_combined_parameter_matrix'] and sensitivity['physical_repeatability_not_verified'];assert [g['cases'] for g in sensitivity['groups']]==[6,4,4];checks.append('new candidate14 one-factor probes not old27 case matrix')
    oldsha=packet['historical_63deg_reference']['candidate_sha256'];assert oldsha!=packet['candidate_sha256']
    assert all(packet[k]['candidate_sha256']==oldsha and packet[k]['historical_candidate_only'] and packet[k]['does_not_validate_current6166_goal_candidate'] for k in ('full_model_matrix','hand_only','initial_placement_sensitivity','hypothetical_reaction'));checks.append('historical robustness counts not transferred to new candidate')
    loads=raw['hypothetical_load_cases'];assert len(loads)==5 and sum(c['load_filter_passed'] for c in loads)==4 and not raw['all_nominal_hypothetical_loads_passed'];assert next(c for c in loads if c['hypothetical_reaction_torque_nm']==-.01)['max_yaw_change_deg']>3;checks.append('new candidate negative high-load failure retained')
    temporal=packet['temporal_two_hand_protocol'];assert temporal['synthetic_cycle_only'] and temporal['new_nominal_model_admission']['nominal_model_handoff_admitted'] and not temporal['actual_loaded_motion_connection_implemented'];checks.append('nominal protocol admission not an implemented left motion')
    near=packet['near_quarter_full_initial_replay'];assert near['initial_placement_through_fixed_hold_replayed'] and near['free_cube_not_reset'] and near['safety_passed'] and near['completed']==near['total_goals']==2628
    assert 85<near['whole_cube_yaw_deg']<95 and near['fixed_hold_seconds']==10.05 and not near['common_grip_passed'] and 'required_lower_contact_presence' in near['reasons'];checks.append('near quarter full replay separated from final grip completion')
    assert packet['torque_capped_full_path']['cases']==2 and packet['torque_capped_full_path']['task_passed']==2 and packet['torque_capped_full_path']['real450mA_to_torque_not_calibrated'];checks.append('model torque hypotheses not real current conversions')
    partial=packet['postturn_partial_progress'];assert 85<partial['yaw_deg']<95 and partial['tilt_deg']<1 and partial['contact_fraction']>.8 and partial['fixed_goal_hold_seconds']==10.05 and not partial['complete'] and partial['checkpoint_continuation_not_full_initial_replay'];checks.append('upright countergrip separated from upper sweep completion')
    links=[]
    for page in (O/'offline_review.html',O/'evidence_thumb12/index.html',O/'articulated_upper_layer/evidence/index.html',O/'near_quarter_full_replay/evidence/index.html',O/'countergrip_progress/index.html',O/'temporal_handoff_review.html',O/'continuous_handoff/evidence/index.html'):
        parser=Links();parser.feed(page.read_text(encoding='utf-8'))
        for value in parser.links:
            parts=urlsplit(value)
            if parts.scheme or not parts.path:continue
            target=(page.parent/unquote(parts.path)).resolve();assert target.is_relative_to(R.resolve()) and target.is_file(),str(target);links.append(str(target.relative_to(R)))
    checks.append('local HTML links')
    manifest=json.loads((O/'evidence_thumb12/manifest.json').read_bytes());videos=[]
    for video in manifest['videos']:
        p=O/'evidence_thumb12'/video['file'];probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(p)]));stream=next(s for s in probe['streams'] if s['codec_type']=='video');seconds=float(probe['format']['duration']);assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720);assert abs(seconds-video['video_seconds'])<.2
        videos.append(dict(file=video['file'],codec=stream['codec_name'],seconds=seconds,frames=int(stream['nb_frames'])))
    checks.append('video duration and codec')
    layer_manifest=json.loads((O/'articulated_upper_layer/evidence/manifest.json').read_bytes());video=layer_manifest['video'];p=O/'articulated_upper_layer/evidence'/video['file'];assert sha(p)==video['sha256'];assert sha(R/layer_manifest['source_review'])==layer_manifest['source_review_sha256']
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(p)]));stream=next(s for s in probe['streams'] if s['codec_type']=='video');seconds=float(probe['format']['duration']);assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720) and abs(seconds-video['seconds'])<.2
    videos.append(dict(file='articulated_upper_layer/evidence/'+video['file'],codec=stream['codec_name'],seconds=seconds,frames=int(stream['nb_frames'])));checks.append('upper layer video source hash and duration')
    near_manifest=json.loads((O/'near_quarter_full_replay/evidence/manifest.json').read_bytes());assert sha(R/near_manifest['source_review'])==near_manifest['source_review_sha256'] and near_manifest['whole90_and_common_grip_passed'] is False
    for video in near_manifest['videos']:
        p=O/'near_quarter_full_replay/evidence'/video['file'];assert sha(p)==video['sha256'];probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(p)]));stream=next(s for s in probe['streams'] if s['codec_type']=='video');seconds=float(probe['format']['duration']);assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720) and abs(seconds-video['seconds'])<.2
        videos.append(dict(file='near_quarter_full_replay/evidence/'+video['file'],codec=stream['codec_name'],seconds=seconds,frames=int(stream['nb_frames'])))
    checks.append('near quarter videos source hashes and duration')
    progress_manifest=json.loads((O/'countergrip_progress/manifest.json').read_bytes());assert progress_manifest['all_shown_cases_are_incomplete'] and not progress_manifest['physical_video']
    for item in progress_manifest['evidence']:
        assert sha(R/item['source_review'])==item['source_review_sha256'] and not item['checkpoint_complete']
        if 'video' not in item:continue
        video=item['video'];p=O/'countergrip_progress'/video['file'];assert sha(p)==video['sha256'];probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(p)]));stream=next(s for s in probe['streams'] if s['codec_type']=='video');seconds=float(probe['format']['duration']);assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720) and abs(seconds-video['seconds'])<.2 and int(stream['nb_frames'])==video['frames'];videos.append(dict(file='countergrip_progress/'+video['file'],codec=stream['codec_name'],seconds=seconds,frames=int(stream['nb_frames'])))
    checks.append('partial countergrip media checked without promoting completion')
    cm=json.loads((O/'continuous_handoff/evidence/manifest.json').read_bytes());assert cm['source_review_sha256']==sha(O/'continuous_handoff/review.json') and cm['source_end_state_sha256']==sha(O/'continuous_handoff/end_state.npz') and cm['source_trajectory_sha256']==sha(O/'continuous_handoff/trajectory.npz') and cm['no_pose_interpolation'] and not cm['physical_video']
    trajectory=np.load(O/'continuous_handoff/trajectory.npz');times=trajectory['samples']*float(trajectory['goal_period_s'])
    for video in cm['videos']:
        p=O/'continuous_handoff/evidence'/video['file'];assert sha(p)==video['sha256'];indices=np.asarray(video['source_sample_indices']);assert len(indices)==video['frames'] and np.all(np.diff(indices)>=0) and np.max(indices)<len(times)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(p)]));stream=next(s for s in probe['streams'] if s['codec_type']=='video');seconds=float(probe['format']['duration']);assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720) and abs(seconds-video['seconds'])<.2 and int(stream['nb_frames'])==video['frames'];videos.append(dict(file='continuous_handoff/evidence/'+video['file'],codec=stream['codec_name'],seconds=seconds,frames=int(stream['nb_frames'])))
    checks.append('continuous replay videos bind actual saved sample times without pose interpolation')
    scripts=[p for p in R.glob('*20261105.py') if p.name.startswith(('build_canonical','validate_canonical','probe_forward_yaw','bounded_translating','search_free_forward','run_v3_grip','measure_saved_forward','correlate_forward','evaluate_canonical','search_canonical','screen_canonical','render_canonical','summarize_canonical','build_v3','build_offline_two_hand'))]
    scripts.extend(p for p in R.glob('*20261105.py') if p.name.startswith(('research_common_grip','propose_sliding','evaluate_sliding','research_workspace','review_wide','build_articulated','evaluate_articulated','render_articulated','validate_articulated','summarize_offline','review_common_grip','cube_task_frame','offline_fractional','validate_green_grip')))
    scripts=list(set(scripts+[R/p for p in packet['implementation_sha256']]))
    for p in scripts:ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
    checks.append('research script syntax')
    report=dict(status='offline_packet_sources_and_media_verified',checks=checks,source_bindings=len(packet['source_documents_sha256'])+len(packet['implementation_sha256']),checked_local_links=len(links),syntax_checked_scripts=len(scripts),videos=videos,devices_opened=False,hardware_execution_allowed=False)
    (O/'packet_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
