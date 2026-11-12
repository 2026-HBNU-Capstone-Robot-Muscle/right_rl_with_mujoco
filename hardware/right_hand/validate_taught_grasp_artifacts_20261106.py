"""Check saved teaching provenance, motor mapping and derived path integrity."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/taught_upper_clearance'
PROTECTED={
'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
'screen_yaw_roundtrip_model_20261030.py':'c9097f13bce11a1cb457e7670b0e916fd1e71c6fce72c175baa874e9e4b72e10',
'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    checks=[]
    for name,expected in PROTECTED.items():assert sha(R/name)==expected,name
    checks.append('six protected source/configuration files unchanged')
    teach=json.loads((R/'manual_grasp_teach_20261106_183247_578628/result.json').read_bytes());review=json.loads((R/'right_hand_rotation_20261106/manual_grasp/middle_teach_review.json').read_bytes())
    assert review['middle_tip_to_root_ids']==[9,11,10,12] and review['middle_root_angle_not_discarded']
    assert review['middle_all_four_saved_angles_are_approximate_shape_evidence'] and not review['instruction_to_repeat_OFF_teaching']
    assert review['source_sha256']==sha(R/review['source']) and review['photo_sha256']==sha(R/review['photo'])
    assert [teach['corrected_position_ticks'][i-1] for i in [9,11,10,12]]==[3362,2523,1770,1983]
    checks.append('four taught middle joints, source photograph and user clarification preserved')
    lo,hi=limits();base=np.load(O/'baseline_path.npz');source=np.load(R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz');assert np.array_equal(base['goals'][:860],source['goals'][:860]);assert np.all(base['goals'][-134:]==base['goals'][-1])
    checks.append('baseline uses original grasp and a fixed 10.05-second hold')
    paths=sorted(O.glob('middle*/proposed_path.npz'))+sorted((O/'sliding_contact').glob('*/*_path.npz'))+sorted(O.glob('dynamic_refinement*/candidate.npz'))+sorted((O/'contact_following').glob('*/candidate.npz'))
    fixed=np.array([4,5,6,7,12,13,14,15])
    for path in paths:
        z=np.load(path);rows=np.asarray(z['goals']);start=z['start'];assert rows.shape[1:]==(16,) and np.isfinite(rows).all(),str(path)
        assert np.all(rows>=lo) and np.all(rows<=hi),str(path)
        assert max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())<=2,str(path)
        assert np.all(rows[:,fixed]==start[fixed]),str(path)
        assert len(rows)>=134 and np.all(rows[-134:]==rows[-1]),str(path)
    checks.append(f'{len(paths)} research paths preserve native bounds, 2-tick steps, index/ring goals and fixed final holds')
    transfer_paths=sorted((O/'index_contact_transfer').glob('*/*_path.npz'))
    for path in transfer_paths:
        z=np.load(path);rows=z['goals'];start=z['start'];assert np.all(rows>=lo) and np.all(rows<=hi),str(path)
        assert max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())<=2,str(path)
        assert np.all(rows[:,12:16]==start[12:16]) and np.all(rows[-134:]==rows[-1]),str(path)
    checks.append(f'{len(transfer_paths)} separately labelled index-support paths preserve ring goals and all original native/step bounds')
    fresh=O/'current_recovery_215211/screen/review.json'
    if fresh.exists():
        r=json.loads(fresh.read_bytes());assert len(r['static_cases'])==201 and len(r['dynamic_cases'])==9 and all(c['passed'] for c in r['static_cases']+r['dynamic_cases']) and r['all_passed']
        checks.append('fresh stationary OFF recovery passed 201 static and 9 dynamic cases')
    for name in ['research_taught_grasp_20261106.py','research_taught_contact_slide_20261106.py','refine_taught_grasp_dynamic_20261106.py','follow_taught_lower_contacts_20261106.py','render_taught_grasp_20261106.py']:
        tree=ast.parse((R/name).read_text(encoding='utf-8'))
        imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]+[a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
        assert not any(any(bad in n for bad in ('dynamixel','serial','camera','cv2')) for n in imports),name
    checks.append('new research/render scripts contain no direct motor or camera imports')
    report=dict(status='taught_grasp_artifact_integrity_passed',checks=checks,research_paths_checked=len(paths)+len(transfer_paths),original_middle_thumb_paths_checked=len(paths),index_support_paths_checked=len(transfer_paths),devices_opened=False,hardware_execution_allowed=False,physical_success=False)
    (O/'validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
