"""New joint hypotheses only: retain or hold the thumb while the middle sweeps back.

No devices, robot commands, model limit changes, or collision-bound changes.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from calibrated_quarter_plan_20261028 import LOADED_PATH, OPEN_PATH
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261029/counter_supported_thumb'

def interp(a, b):
    n = max(1, int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1, n+1)]).astype(int)

def main():
    OUT.mkdir(exist_ok=True)
    original = np.load(LOADED_PATH)
    opened = np.load(OPEN_PATH)['open_goal'].astype(int)
    grasp = original['goals'][original['phases']=='grasp'].astype(int)
    yaw = original['goals'][original['phases']=='yaw'].astype(int)
    base = grasp[-1]
    cfg = registry.get_default_config('LeapCubeFaceUp'); cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model, _ = build(env, OUT/'geometry.xml', book_projection_mm=-10.)
    correct_model(model, opened)
    hand = {model.body('palm').id}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in hand:
            hand.add(body)
    support = {i for i in range(model.ngeom) if (model.geom(i).name or '').startswith(
        ('support_', 'leap_mount_collision')) or model.geom(i).name=='floor'}
    inspect = cached_contacts(model, hand, support)
    lo, hi = limits()
    mapping = dict(grasp='stage_5_loaded_grasp', grasp_hold='stage_6_grasp_hold',
        yaw='stage_7_horizontal_yaw', yaw_hold='stage_8_horizontal_hold',
        release='stage_9_loaded_open', remove_hold='stage_10_remove_hold')
    reports = []
    for middle in (.4, .7):
      for thumb in (1., 0.):
       for ring in (-128, -192):
        name = f'middle{middle:g}_thumb{thumb:g}_ring{ring}'
        turn = yaw.copy()
        turn[:,9] = np.rint(base[9]-middle*(yaw[:,9]-base[9])).astype(int)
        turn[:,2] = np.rint(base[2]+thumb*(yaw[:,2]-base[2])).astype(int)
        custom = opened.copy(); custom[13] += ring
        pieces = [('grasp', grasp), ('grasp_hold', np.repeat(base[None,:],40,axis=0)),
            ('yaw',turn), ('yaw_hold',np.repeat(turn[-1][None,:],40,axis=0)),
            ('release',interp(turn[-1],opened)),
            ('remove_hold',np.repeat(opened[None,:],40,axis=0))]
        goals = np.vstack([q for _,q in pieces]); goals[:,13] += ring
        phases = np.asarray([mapping[stage] for stage,qs in pieces for _ in qs])
        values = np.vstack([custom,goals])
        valid = bool(np.all(values>=lo) and np.all(values<=hi) and
            np.max(abs(np.diff(values,axis=0)))<=2)
        folder = OUT/name; folder.mkdir(exist_ok=True)
        path = folder/'candidate.npz'
        np.savez_compressed(path,start=custom,goals=goals,phases=phases,
            empty_ring_approach=interp(opened,custom))
        row = dict(name=name, candidate=path.relative_to(ROOT).as_posix(),
            candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            native_and_two_tick_bounds=valid,goals=len(goals),
            geometry_passed=False)
        if valid:
            data = mujoco.MjData(model); data.qpos[-7:] = [0,0,10,1,0,0,0]
            peak_self = peak_support = 0.; first = None; worst = None
            approach = interp(opened, custom)
            for i,q in enumerate(np.vstack([approach,goals])):
                data.qpos[:16] = ticks_to_sim(q); mujoco.mj_forward(model,data)
                su,sp = inspect(data); sp = classify(sp)
                peak_self = max(peak_self,max(sp.values(),default=0.))
                peak_support = max(peak_support,max(su.values(),default=0.))
                if peak_self>1e-6 or peak_support>.15:
                    first = i; worst = max(sp,key=sp.get) if sp else None; break
            row.update(geometry_passed=first is None,static_max_self_mm=peak_self,
                static_max_support_mm=peak_support,first_rejected_goal=first,
                worst_self_pair=worst)
        reports.append(row); print(json.dumps(row),flush=True)
        (OUT/'geometry_review.json').write_text(json.dumps(dict(cases=reports,
            hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
    # Audit the effective model material before interpreting any friction sweep.
    audit = []
    cube = model.geom('cube').id
    for coefficient in (.2,.3,.5,.8,1.):
        model.geom_friction[cube,0] = coefficient
        audit.append(dict(requested_cube_sliding=coefficient,
            cube_priority=int(model.geom_priority[cube]),
            tips={name:dict(sliding=float(model.geom_friction[model.geom(name).id,0]),
                priority=int(model.geom_priority[model.geom(name).id]))
                for name in ('if_tip','mf_tip','rf_tip','th_tip')}))
    (OUT/'material_parameters.json').write_text(json.dumps(audit,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__': main()
