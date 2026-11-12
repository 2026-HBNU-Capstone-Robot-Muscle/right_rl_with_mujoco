"""Check the new cube hypothesis and its recorded load experiments offline."""
import hashlib,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import R
from build_articulated_upper_layer_20261105 import OUT,SOURCE,BODY_FIELDS,GEOM_FIELDS,JOINT_FIELDS,ACT_FIELDS,inertia
from research_rotation_body_hold_20261104 import inspect_for

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    manifest=json.loads((OUT/'model_review.json').read_bytes());assert manifest['source_model_sha256']==sha(SOURCE);assert manifest['model_sha256']==sha(OUT/'model.mjb')
    old=mujoco.MjModel.from_binary_path(str(SOURCE));m=mujoco.MjModel.from_binary_path(str(OUT/'model.mjb'));hand,support,_=inspect_for(old);checks=[]
    for ob in hand:
        nb=m.body(old.body(ob).name).id
        for key in BODY_FIELDS:assert np.array_equal(getattr(old,key)[ob],getattr(m,key)[nb]),key
        og=[g for g in range(old.ngeom) if int(old.geom_bodyid[g])==ob];ng=[g for g in range(m.ngeom) if int(m.geom_bodyid[g])==nb];assert len(og)==len(ng)
        for a,b in zip(og,ng):
            for key in GEOM_FIELDS:assert np.array_equal(getattr(old,key)[a],getattr(m,key)[b]),key
    for a in range(16):
        b=m.joint(old.joint(a).name).id
        for key in JOINT_FIELDS:assert np.array_equal(getattr(old,key)[a],getattr(m,key)[b]),key
    for key in ACT_FIELDS:assert np.array_equal(getattr(old,key),getattr(m,key)),key
    checks.append('right_hand_bodies_geometry_actuators_native_bounds_equal_to_source')
    total=np.zeros((3,3));mass=0.;weighted_center=np.zeros(3)
    for name,center in (('cube',np.array([0.,.055/6,0.])),('upper_layer',np.array([0.,-.055/3,0.]))):
        b=m.body(name).id;value=float(m.body_mass[b]);mass+=value;weighted_center+=value*center;total+=np.diag(m.body_inertia[b])+value*((center@center)*np.eye(3)-np.outer(center,center))
    assert abs(mass-.1)<1e-12 and np.linalg.norm(weighted_center)<1e-12
    assert np.allclose(total,np.diag(inertia(.1,np.repeat(.0275,3))),atol=1e-12,rtol=0)
    checks.append('two_body_mass_center_and_combined_uniform_cube_inertia')
    for axis in range(3):
        intervals=[]
        for name,offset in (('cube',np.zeros(3)),('upper_layer_collision',np.array([0.,-.055/3,0.]))):
            g=m.geom(name).id;pos=offset+m.geom_pos[g];intervals.append((pos[axis]-m.geom_size[g,axis],pos[axis]+m.geom_size[g,axis]))
        assert abs(min(x[0] for x in intervals)+.0275)<1e-12 and abs(max(x[1] for x in intervals)-.0275)<1e-12
    j=m.joint('upper_layer_axis').id;assert np.allclose(m.jnt_axis[j],[0,-1,0]) and not m.jnt_limited[j] and all(int(m.actuator_trnid[k,0])!=j for k in range(m.nu))
    checks.append('55mm_envelope_and_no_internal_upper_layer_actuator')
    cases=[]
    for p in sorted(OUT.glob('bearing*/review.json')):
        if '_friction_audit' in p.parent.name:continue
        d=json.loads(p.read_bytes());assert d['model_sha256']==sha(OUT/'model.mjb');assert d['source_state_sha256']==sha(R/d['source_state']);assert d['right_hand_targets_constant'] and d['external_torque_not_an_internal_hinge_motor'];assert d['hardware_execution_allowed'] is False and d['devices_opened'] is False
        cases.append(dict(source=str(p.relative_to(R)),sha256=sha(p),bearing_parameter_nm=d['bearing_frictionloss_hypothesis_nm'],direction=d['direction'],passed=d['load_filter_passed'],safety=d['original_hand_dynamic_safety_passed'],top_angle_deg=d['upper_final_angle_deg'],body_yaw_deg=d['maximum_lower_body_yaw_change_deg'],body_shift_mm=d['maximum_lower_body_shift_mm'],maximum_external_layer_torque_nm=d['maximum_external_layer_torque_nm'],gap_mm=d['minimum_upper_sweep_gap_1ms_mm'],reasons=d['reasons']))
    checks.append('recorded_case_bindings_and_offline_scope')
    report=dict(status='articulated_cube_structure_and_load_evidence_checked',checks=checks,model_sha256=sha(OUT/'model.mjb'),cases=cases,passed=sum(c['passed'] for c in cases),total_cases=len(cases),load_hypotheses_not_actual_left_hand_validation=True,hardware_execution_allowed=False,devices_opened=False)
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
