"""Review saved joint samples and image markers; no devices or image edits."""
from pathlib import Path
from collections import defaultdict
import hashlib,json,math
import numpy as np
import cv2

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'middle_clearance_probe_20261106_230902_998326'
OUT=ROOT/'right_hand_rotation_20261109/middle_probe_actual_review'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def markers(image):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,_=cv2.aruco.ArucoDetector(dictionary,parameters).detectMarkers(image)
    return {int(marker):corner.reshape(4,2).astype(float) for marker,corner in zip(ids.ravel(),corners)} if ids is not None else {}


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    raw=json.loads((RUN/'result.json').read_bytes());rows=raw['samples'];phases=defaultdict(list)
    for sample in rows:phases[sample['phase']].append(sample)
    summaries={}
    for phase,samples in phases.items():
        actual=np.array([sample['actual'] for sample in samples]);goals=np.array([sample['goal'] for sample in samples]);currents=np.array([sample['current_ma'] for sample in samples]);tracking=np.array([sample['tracking_rad'] for sample in samples])
        summaries[phase]={'samples':len(samples),'duration_s':samples[-1]['t']-samples[0]['t'],
                          'first_actual':actual[0].tolist(),'last_actual':actual[-1].tolist(),
                          'median_actual_last10':np.median(actual[-10:],axis=0).tolist(),
                          'last_goal':goals[-1].tolist(),'max_abs_current_ma':int(np.max(np.abs(currents))),
                          'max_tracking_rad':float(tracking.max()),'min_voltage_v':min(min(sample['voltage_v']) for sample in samples),
                          'max_temperature_c':max(max(sample['temperature_c']) for sample in samples)}
    before=np.array(raw['grasp_position_ticks']);after=np.array(raw['middle_probe_position_ticks']);delta=after-before
    joint_changes=[{'motor_id':i+1,'before_actual_ticks':int(before[i]),'after_actual_ticks':int(after[i]),
                    'delta_ticks':int(delta[i]),'delta_deg':float(delta[i]*360/4096),
                    'is_middle':i in range(8,12)} for i in range(16)]
    images={name:cv2.imdecode(np.frombuffer((RUN/name).read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR) for name in ('cube_grasp_hold.jpg','middle_probe_hold.jpg')}
    if any(image is None for image in images.values()):raise RuntimeError('Could not decode source photo')
    detected={name:markers(image) for name,image in images.items()}
    names=list(images);common=sorted(set(detected[names[0]])&set(detected[names[1]]))
    anchors=[marker for marker in (40,41) if marker in common]
    anchor_deltas=np.concatenate([detected[names[1]][marker]-detected[names[0]][marker] for marker in anchors]) if anchors else None
    translation=np.median(anchor_deltas,axis=0) if anchors else np.zeros(2)
    drift=[]
    for marker in common:
        a=detected[names[0]][marker];b=detected[names[1]][marker]
        drift.append({'marker_id':marker,'center_shift_px':(b.mean(0)-a.mean(0)).tolist(),
                      'anchor_translation_corrected_shift_px':(b.mean(0)-a.mean(0)-translation).tolist() if anchors else None,
                      'corner_rms_after_translation_px':float(np.sqrt(np.mean(np.sum((b-a-translation)**2,axis=1)))) if anchors else None})
    result={
        'status':'saved_middle_only_physical_probe_review','source':RUN.name,
        'source_result_sha256':sha(RUN/'result.json'),
        'source_status':raw['status'],'all_torque_off_verified':raw['all_torque_off_verified'],
        'settings_restored':raw['settings_restored'],'cube_removal_confirmed':raw['cube_removal_confirmed'],
        'original_samples':len(rows),'min_voltage_v':min(min(sample['voltage_v']) for sample in rows),
        'max_abs_current_ma':max(max(abs(value) for value in sample['current_ma']) for sample in rows),
        'max_tracking_rad':max(max(sample['tracking_rad']) for sample in rows),
        'phases':summaries,'joint_changes':joint_changes,
        'image_review':{'files':{name:sha(RUN/name) for name in images},
                        'dictionary':'DICT_4X4_50','decoded_ids':{name:sorted(value) for name,value in detected.items()},
                        'common_anchor_ids':anchors,'estimated_image_translation_px':translation.tolist() if anchors else None,
                        'marker_displacement':drift,'metric_3d_registration_verified':False,
                        'cube_remains_in_similar_projected_position':True,
                        'middle_finger_largely_occluded':True,
                        'thumb_still_beside_upper_layer':True,
                        'full_upper_layer_sweep_clearance_proven':False,
                        'lower_body_force_closure_proven':False,
                        'scope':'two saved observation photos; not continuous motion or applied upper-layer load'},
        'read_failure_events':len(raw.get('read_failures',[])),
        'read_recovery_events':len(raw.get('read_recovery_confirmation_events',[])),
        'physical_rotation_tested':False,'devices_opened':False,
    }
    (OUT/'review.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({key:value for key,value in result.items() if key not in ('phases','joint_changes')} ,ensure_ascii=False))
    print(json.dumps({'middle_changes':[row for row in joint_changes if row['is_middle']],
                      'maximum_other_finger_delta_ticks':int(max(abs(delta[np.r_[0:8,12:16]])))},ensure_ascii=False))


if __name__=='__main__':main()
