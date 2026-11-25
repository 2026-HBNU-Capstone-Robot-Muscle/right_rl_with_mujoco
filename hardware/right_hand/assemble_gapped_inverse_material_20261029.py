"""Bind all 27 completed finite cases to the exact loaded tape and cube seed."""
import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    cases=[];sources=[];candidate=OUT/'candidate.npz';seed=OUT/'cube_seed.json';z=np.load(candidate)
    phase_map={'stage_5_loaded_grasp':'grasp','stage_6_grasp_hold':'grasp_hold',
        'stage_7_horizontal_yaw':'yaw','stage_8_horizontal_hold':'yaw_hold',
        'stage_9_loaded_open':'release','stage_10_remove_hold':'remove_hold'}
    for front in (-15,-10,-5):
        path=OUT/f'material_checked_front_{front}/review.json';r=json.loads(path.read_bytes())
        assert r['all_passed'] and r['candidate_sha256']==sha(candidate) and r['cube_seed_sha256']==sha(seed)
        assert r['material_policy']=='both_contacting_surfaces_v1_not_physical_identification'
        assert r['simulation_timestep_s']==.001 and r['initial_placement_also_checked']
        assert r['finite_stiffness_contact_forces_preserved'] and not r['hardware_execution_allowed'] and not r['devices_opened']
        generated=path.parent/'loaded_path.npz';g=np.load(generated)
        assert r['loaded_path_sha256']==sha(generated)
        assert np.array_equal(g['start'],z['start']) and np.array_equal(g['goals'],z['goals'])
        assert g['phases'].tolist()==[phase_map[str(p)] for p in z['phases']]
        sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
        cases.extend(r['cases'])
    assert len(cases)==27
    assert {(r['book_front_mm'],r['gain_scale'],r['friction']) for r in cases}=={
        (f,g,mu) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    limits={'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1.,
        'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,
        'center_translation_mm':80.,'longest_no_hand_contact_seconds':.25}
    for row in cases:
        assert row['passed'] and row['stop'] is None and row['initial_placement_passed']
        assert row['final_cube_supported_by_hand'] and not row['external_cube_contact']
        assert all(math.isfinite(row[k]) and 0<=row[k]<=v for k,v in limits.items())
        assert row['placed_to_turn_deg']<=-15 and row['yaw_tilt_change_deg']<=15
        probe=row['initial_placement_probe']
        assert probe['max_self_mm']<=1e-6 and probe['max_support_mm']<=.15 and probe['max_cube_mm']<=1 and probe['max_tracking_rad']<=.18 and not probe['external_cube_contact']
        material=row['material_audit'];assert material['requested_effective_sliding_coefficient']==row['friction']
        assert material['effective_contacts_after_settle']
        assert all(np.allclose(c['sliding'],row['friction'],rtol=0,atol=1e-12)
            for c in material['effective_contacts_after_settle'])
    nominal=next(r for r in cases if (r['book_front_mm'],r['gain_scale'],r['friction'])==(-10.,1.,.3))
    report=dict(status='gapped_inverse_27_case_material_review',all_passed=True,cases=cases,sources=sources,
        candidate_sha256=sha(candidate),cube_seed_sha256=sha(seed),
        model_source_sha256=sha(ROOT/'screen_material_checked_reverse_cube_20261029.py'),
        material_policy_sha256=sha(ROOT/'cube_contact_material_policy_20261029.py'),
        nominal_placed_to_turn_deg=nominal['placed_to_turn_deg'],
        placed_to_turn_range_deg=[min(r['placed_to_turn_deg'] for r in cases),max(r['placed_to_turn_deg'] for r in cases)],
        actual_contact_material_varied=True,physical_material_identified=False,
        physics_acceptance_bounds_unchanged=True,physical_inverse_cube_rotation_verified=False,
        inverse_90_ready=False,initial_cube_pose_not_registered=True,
        devices_opened=False,hardware_execution_allowed=False)
    (OUT/'inverse_material_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('cases','sources')}))
if __name__=='__main__':main()
