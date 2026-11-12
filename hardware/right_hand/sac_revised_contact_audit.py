"""Offline contact consistency of the photographed revised start; no devices."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_support_separation import SupportSeparation

ROOT = Path(__file__).resolve().parent

def main():
    audit = json.loads((ROOT/'sac_revised_placement_audit_result.json').read_text())
    reset = json.loads((ROOT/'sac_supported_reset_result.json').read_text())
    qpos = np.asarray(reset['candidates'][0]['qpos'], float)
    cfg = registry.get_default_config('LeapCubeFaceUp'); cfg.impl = 'jax'
    model, manifest = build(registry.load('LeapCubeFaceUp', config=cfg))
    sep = SupportSeparation(model)
    support = {model.geom(n).id for n in ('support_glass', 'support_wood')}
    joints = np.asarray(audit['measured_joint_angles_sim_order'])
    reg = audit['registration']
    variants = [('trained_reset', qpos.copy())]
    measured = qpos.copy(); measured[:16] = joints
    variants.append(('measured_joints_training_cube', measured.copy()))
    measured[-7:-4] = reg['cube_center_in_sim_world_candidate_m']
    measured[-4:] = reg['cube_quat_world_candidate']
    variants.append(('measured_joints_image_cube_provisional', measured.copy()))
    rows = []
    for label, initial in variants:
        data = mujoco.MjData(model); data.qpos[:] = initial
        data.ctrl[:] = joints if label != 'trained_reset' else reset['candidates'][0]['ctrl']
        mujoco.mj_forward(model, data)
        contacts = []
        for c in data.contact:
            names = [model.geom(c.geom1).name, model.geom(c.geom2).name]
            if c.dist < 0:
                contacts.append(dict(pair=names, penetration_mm=-float(c.dist)*1000,
                                     involves_support=c.geom1 in support or c.geom2 in support))
        clearance = min(sep.lower_bound(data, g, s) for g in sep.vertices for s in support)*1000
        peak_penetration = max([c['penetration_mm'] for c in contacts] or [0])
        start = data.qpos[-7:].copy()
        max_shift = 0.; max_joint_shift = 0.
        # Diagnostic zero-motion hold, never used to certify/adjust registration.
        for _ in range(round(2/model.opt.timestep)):
            mujoco.mj_step(model, data)
            max_shift = max(max_shift, float(np.linalg.norm(data.qpos[-7:-4]-start[:3])*1000))
            max_joint_shift = max(max_joint_shift, float(np.max(abs(data.qpos[:16]-initial[:16]))))
        final_q = data.qpos[-4:]/np.linalg.norm(data.qpos[-4:])
        start_q = start[-4:]/np.linalg.norm(start[-4:])
        rows.append(dict(case=label, initial_penetrating_contacts=contacts,
                         max_initial_penetration_mm=peak_penetration,
                         initial_housing_clearance_lower_bound_mm=clearance,
                         hold_seconds=2, max_cube_center_shift_mm=max_shift,
                         max_joint_shift_rad=max_joint_shift,
                         final_cube_rotation_from_initial_deg=float(np.degrees(2*np.arccos(np.clip(abs(final_q@start_q),0,1)))),
                         initial_cube_qpos=start.tolist(), final_cube_qpos=data.qpos[-7:].tolist()))
    report = dict(status='offline_contact_consistency_only', source=audit['source'],
                  motor_commands_sent=0, hardware_authorized=False, cases=rows,
                  limitations=['Image pose and marker-to-model registration are provisional.',
                               'Contact consistency cannot validate pose or actual support clearance.',
                               'Native fixed-goal hold is not a SAC policy evaluation.'])
    (ROOT/'sac_revised_contact_audit_result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(rows, indent=2))

if __name__ == '__main__': main()
