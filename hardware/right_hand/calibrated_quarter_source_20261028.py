"""Bind the shorter route to raw saved encoder/image evidence, no devices."""
import json,hashlib,math
import numpy as np
from horizontal_quarter_plan_20261027 import ROOT,LOADED_PATH as ORIGINAL
from calibrated_quarter_plan_20261028 import DATA,LOADED_PATH
def review():
    meta=json.loads((DATA/'calibration_source.json').read_bytes())
    trace=json.loads((ROOT/meta['trace_source']).read_bytes());seen=trace['closest_observed_targets']['90']
    raw_path=ROOT/meta['observed_source'];raw=json.loads(raw_path.read_bytes())
    if raw.get('status')!='horizontal_quarter_observation_complete' or raw.get('all_torque_off_verified') is not True or raw.get('settings_restored') is not True or raw.get('cube_removal_confirmed') is not True:raise RuntimeError('Missing completed raw physical observation')
    angle=seen.get('angle_deg');count=seen.get('yaw_goals_already_observed')
    if count!=638 or not isinstance(angle,(int,float)) or not math.isfinite(angle) or abs(angle-90)>2:raise RuntimeError('Missing observed90-degree prefix')
    source=ROOT/meta['source']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=meta['source_sha256']:raise RuntimeError('Original candidate source changed')
    full=np.load(ORIGINAL);candidate=np.load(DATA/'candidate.npz');loaded=np.load(LOADED_PATH)
    original_yaw=full['goals'][full['phases']=='yaw'];short=original_yaw[:count]
    if not np.array_equal(short,candidate['goals']) or not np.array_equal(short,loaded['goals'][loaded['phases']=='yaw']):raise RuntimeError('638 original goals not preserved exactly')
    if not np.array_equal(candidate['start'],full['grasp_goal']) or not np.array_equal(loaded['start'],full['start']):raise RuntimeError('Grasp or OPEN join changed')
    for phase in ('grasp','grasp_hold'):
        if not np.array_equal(loaded['goals'][loaded['phases']==phase],full['goals'][full['phases']==phase]):raise RuntimeError('Reached grasp changed')
    expected_phases=['grasp','grasp_hold','yaw','yaw_hold','release','remove_hold']
    phases=[str(p) for i,p in enumerate(loaded['phases']) if not i or p!=loaded['phases'][i-1]]
    if phases!=expected_phases:raise RuntimeError('Unexpected calibration phase order')
    for phase,end in (('yaw_hold',short[-1]),('remove_hold',full['start'])):
        if not np.array_equal(loaded['goals'][loaded['phases']==phase],np.repeat(end[None,:],40,axis=0)):raise RuntimeError('Fixed hold changed')
    count_release=max(1,int(np.ceil(max(abs(full['start']-short[-1]))/2)))
    release=np.rint([short[-1]+(full['start']-short[-1])*i/count_release for i in range(1,count_release+1)]).astype(int)
    if not np.array_equal(loaded['goals'][loaded['phases']=='release'],release):raise RuntimeError('Release differs from reviewed interpolation')
    if seen['command_ticks']!=short[-1].tolist() or raw['samples'][seen['sample_index']]['goal']!=short[-1].tolist():raise RuntimeError('Observed image/encoder command does not match prefix endpoint')
    image=ROOT/meta['observed_source'].split('/')[0]/'camera'/seen['image']
    if not image.is_file():raise RuntimeError('Raw observed frame missing')
    return dict(kind='one_observed_run_timing_calibration',requested_angle_deg=90,kept_original_yaw_goals=638,original_yaw_goals=len(original_yaw),
        source_run=meta['observed_source'],source_result_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        observed_camera_angle_deg=angle,observed_image=image.relative_to(ROOT).as_posix(),observed_image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
        yaw_endpoint_ticks=short[-1].tolist(),actual_shortened_angle_verified=False,certified_robot_frame_angle=False,
        nominal_model_is_not_a90_degree_prediction=True,placement_variation_compensated=False,
        no_live_policy=True,health_limits_unchanged=True)
