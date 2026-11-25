"""Boundary checks for packaged no-output proposals, acknowledgements and hold."""
from pathlib import Path
import sys
import json
import hashlib
import ast
import numpy as np

OUT=Path(__file__).resolve().parent


def verify():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';bundle=root/'shadow_bundle';sys.path.insert(0,str(bundle));from whole_cycle_shadow_runtime import WholeCycleShadowController
    data=np.load(root/'encoder_pose_feedback_v1_complete_bundle/nominal/observer_fixture.npz',allow_pickle=False);tests=[]
    def sample(i):
        s={name:data[name][i] for name in ('measured_ticks','commanded_ticks','palm_position','cube_position','cube_quaternion')};s.update(sequence=int(data['sequence'][i]),sample_time=float(data['sample_time'][i]),pose_time=float(data['pose_time'][i]),frame_id='leap_sim_world',palm_point='policy_palm_position',observation_origin='simulator_fixture');return s
    def rejects(name,call):
        try:call()
        except ValueError:tests.append(name);return
        raise AssertionError(name+' was accepted')
    c=WholeCycleShadowController(bundle);rejects('Unverified physical registration blocks first proposal',lambda:c.propose(sample(0)));assert c.pending is None and c.phase==0
    c=WholeCycleShadowController(bundle,observation_mode='simulator_fixture');first=sample(0)
    rejects('Simulation origin must be explicit',lambda:c.propose(dict(first,observation_origin='camera')))
    rejects('Stale cube pose rejected before warming',lambda:c.propose(dict(first,pose_time=-.151)))
    rejects('Wrong coordinate frame rejected',lambda:c.propose(dict(first,frame_id='camera')))
    rejects('Wrong initial command rejected',lambda:c.propose(dict(first,commanded_ticks=np.asarray(first['commanded_ticks'])+1)))
    assert c.propose(first)['state']=='warming'
    rejects('Duplicate sequence rejected',lambda:c.propose(first))
    rejects('Too short control interval rejected',lambda:c.propose(dict(sample(1),sample_time=.05,pose_time=.05)))
    assert c.propose(sample(1))['state']=='warming'
    proposed=c.propose(sample(2));assert c.phase==0 and proposed['motor_commands_sent']==0
    rejects('Pending proposal blocks a second proposal',lambda:c.propose(sample(3)))
    rejects('Changed acknowledgement rejected',lambda:c.commit_shadow(proposed['sequence'],np.asarray(proposed['goal_ticks'])+1))
    assert c.phase==0 and c.pending is not None;c.commit_shadow(proposed['sequence'],proposed['goal_ticks']);assert c.phase==1;tests.append('Phase advances only on exact acknowledgement')
    rejects('No pending goal cannot be acknowledged again',lambda:c.commit_shadow(proposed['sequence'],proposed['goal_ticks']))
    # Synthetic terminal boundary; full native completion parity is audited
    # independently by replay_whole_cycle_shadow_fixture.py.
    c.phase=5089;c.last_goal=c.goals[-1].copy();c.baseline=c.last_goal.copy();last=dict(sample(3),measured_ticks=c.last_goal.copy(),commanded_ticks=c.last_goal.copy());hold=c.propose(last);assert np.array_equal(hold['goal_ticks'],c.last_goal);c.commit_shadow(hold['sequence'],hold['goal_ticks']);assert c.phase==5089
    completion=c.completion();assert completion['final_goal_is_kept'] and not completion['automatic_open'] and not completion['automatic_torque_off'] and not completion['next_rotation_or_left_hand_authorized'];tests.append('Terminal hold retains last command with no OPEN/OFF or next-hand authorization')
    forbidden={'serial','cv2','dynamixel_sdk','subprocess','socket','requests'};imports=set()
    for path in bundle.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if isinstance(node,ast.Import):imports.update(alias.name.split('.')[0] for alias in node.names)
            if isinstance(node,ast.ImportFrom) and node.module:imports.add(node.module.split('.')[0])
    assert not forbidden&imports;tests.append('Packaged controller sources import no serial, motor, camera or network interface')
    report=dict(status='passed',tests=tests,bundle_metadata_sha256=hashlib.sha256((bundle/'bundle.json').read_bytes()).hexdigest(),terminal_boundary_is_synthetic_not_native_success=True,motor_commands_sent=0,physical_registration_verified=False,devices_opened=0,hardware_execution_allowed=False)
    (root/'runtime_gate_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');return report


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
