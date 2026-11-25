"""Offline review of the user's seven recorded poses; never imports motor code.

The original is parsed with AST only. This review preserves its raw ID order,
native limits, and current runtime code. Extra mesh hulls are diagnostic only.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim, hw_to_sim, metadata, TICK
from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261024' / 'original_sequence_review'


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')


def load_source():
    archive_path = ROOT / 'sequence_flip_poses.json'
    archive = json.loads(archive_path.read_bytes())
    source = Path('/mnt/c/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
    blob = source.read_bytes()
    if hashlib.sha256(blob).hexdigest() != archive['original_sha256']:
        raise ValueError('Original source differs from preserved archive')
    tree = ast.parse(blob.decode('utf-8-sig'))
    expr = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'SEQUENCE' for t in n.targets))
    names, angles = [], []
    for row in expr.elts:
        label, call = row.elts
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Attribute) or call.func.attr != 'array':
            raise ValueError('Unexpected original pose expression')
        names.append(ast.literal_eval(label))
        angles.append(ast.literal_eval(call.args[0]))
    angles = np.asarray(angles, dtype=float)
    ticks = np.rint(angles / TICK).astype(int)
    if angles.shape != (7, 16) or not np.array_equal(ticks, [p['raw_ticks'] for p in archive['poses']]):
        raise ValueError('AST source and archived raw ID poses disagree')
    return names, angles, dict(original_source_windows=archive['original_source'],
        original_sha256=archive['original_sha256'], archive_sha256=hashlib.sha256(archive_path.read_bytes()).hexdigest(),
        original_imported_or_executed=False, raw_id_values_unchanged=True)


def make_models(env):
    m, manifest = build_book(env, OUT / 'book_scene.xml', book_projection_mm=-10.,
        book_ylo_mm=-500., book_yhi_mm=500., support_projection_mm=50.)
    # The earlier runtime builder does not add middle/ring housing self tests.
    assets = copy.deepcopy(dict(env._model_assets))
    added = []
    for name, blob in list(assets.items()):
        if not str(name).endswith('.xml'):
            continue
        xml = ET.fromstring(blob)
        for body in xml.iter('body'):
            bname = body.get('name', '')
            middle, ring = bname.startswith('mf_'), bname.startswith('rf_')
            if not (middle or ring):
                continue
            for i, geom in enumerate(list(body.findall('geom'))):
                if geom.get('mesh') is None:
                    continue
                proxy = copy.deepcopy(geom)
                proxy.attrib.pop('class', None)
                pname = f'original_review_mf_rf_{bname}_{i}'
                proxy.attrib.update(name=pname, type='mesh', density='0', mass='0', group='4', rgba='0 0 0 0',
                    contype='64' if middle else '128', conaffinity='128' if middle else '64')
                body.append(proxy)
                added.append(pname)
        assets[name] = ET.tostring(xml, encoding='utf-8')
    wrapped = SimpleNamespace(mj_model=env.mj_model, xml_path=env.xml_path, _model_assets=assets)
    diagnostic, _ = build_book(wrapped, OUT / 'book_housing_diagnostic_scene.xml', book_projection_mm=-10.,
        book_ylo_mm=-500., book_yhi_mm=500., support_projection_mm=50.)
    if len(added)!=10 or any(diagnostic.geom_type[diagnostic.geom(n).id]!=mujoco.mjtGeom.mjGEOM_MESH for n in added):
        raise RuntimeError('Middle/ring housing proxy must preserve mesh type')
    set_cube_physics(m, side_m=.055, mass_kg=.1)
    set_cube_physics(diagnostic, side_m=.055, mass_kg=.1)
    return m, diagnostic, manifest, added


def hand_contacts(m, d, diagnostic=False):
    out = []
    for c in d.contact:
        if c.dist >= -1e-7:
            continue
        a, b = int(c.geom1), int(c.geom2)
        na, nb = m.geom(a).name, m.geom(b).name
        ba, bb = m.body(int(m.geom_bodyid[a])).name, m.body(int(m.geom_bodyid[b])).name
        is_hand = lambda name: name == 'palm' or name.startswith(('th_', 'if_', 'mf_', 'rf_'))
        kind = None
        if na.startswith('original_review_mf_rf_') and nb.startswith('original_review_mf_rf_'):
            kind = 'diagnostic_middle_ring_convex_hulls'
        elif is_hand(ba) and is_hand(bb):
            kind = 'native_or_existing_self_geometry'
        elif (na.startswith('support_') and is_hand(bb)) or (nb.startswith('support_') and is_hand(ba)):
            kind = 'support_geometry'
        if kind:
            out.append(dict(kind=kind, pair='|'.join(sorted([na, nb])), model_overlap_mm=-float(c.dist)*1000))
    return out


def audit(m, dm, names, hw, source, manifest, added):
    q = hw_to_sim(hw)
    d, dd = mujoco.MjData(m), mujoco.MjData(dm)
    per_pose, maxima, transitions = [], {}, []
    for index, angles in enumerate(q):
        d.qpos[:16] = angles
        d.qpos[-7:] = [0, 0, 2, 1, 0, 0, 0]
        mujoco.mj_forward(m, d)
        lo, hi = m.jnt_range[:16].T
        errors = np.maximum(lo-angles, angles-hi)
        violations = [dict(joint=metadata()['sim_joint_names'][j], motor_id=metadata()['motor_ids_in_sim_order'][j],
            angle_rad=float(angles[j]), range_rad=[float(lo[j]), float(hi[j])]) for j in np.flatnonzero(errors > 1e-9)]
        per_pose.append(dict(stage=index, name=names[index], native_range_violations=violations,
            native_and_existing_contacts=hand_contacts(m, d),
            tip_world_m={name:d.site(name).xpos.tolist() for name in ('if_tip','mf_tip','rf_tip','th_tip')
                         if mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name)>=0}))
    for stage in range(1, 7):
        delta = (hw[stage]-hw[stage-1])/TICK
        transitions.append(dict(from_stage=stage-1,to_stage=stage,source_duration_s=2.1,
            max_source_step_ticks=float(np.max(abs(delta))/70),
            moving_over_100_ticks_motor_ids=(np.flatnonzero(abs(delta)>100)+1).tolist(),
            max_joint_speed_rad_s=float(np.max(abs(hw[stage]-hw[stage-1]))/2.1)))
        for k in range(71):
            dd.qpos[:16] = q[stage-1]+(q[stage]-q[stage-1])*k/70
            dd.qpos[-7:] = [0,0,2,1,0,0,0]
            mujoco.mj_forward(dm, dd)
            for c in hand_contacts(dm, dd):
                key=c['kind']+':'+c['pair']
                if c['model_overlap_mm'] > maxima.get(key, {}).get('model_overlap_mm',0):
                    maxima[key] = dict(**c,stage=stage,fraction=k/70)
    mujoco.mj_resetData(m,d)
    mujoco.mj_forward(m,d)
    palm=m.body('palm').id
    result=dict(source=source,mapping=metadata(),original_pose_count=7,
        native_range_violation_count=sum(len(p['native_range_violations']) for p in per_pose),
        poses=per_pose,transitions=transitions,whole_path_model_contact_maxima=list(maxima.values()),
        support=manifest,gravity_m_s2=m.opt.gravity.tolist(),palm_origin_world_m=d.xpos[palm].tolist(),
        palm_rotation_world=d.xmat[palm].reshape(3,3).tolist(),diagnostic_middle_ring_hulls=added,
        all_joint_ranges_and_raw_angles_preserved=True,hardware_commands_sent=0,
        physical_rotation_verified=False,physical_contact_depth_measured=False,
        model_contacts_are_geometry_diagnostics_not_hardware_certification=True)
    save(OUT/'source_and_path_audit.json',result)
    print('AUDIT',json.dumps(dict(poses=7,native_limit_violations=result['native_range_violation_count'],
        max_model_overlap_by_kind={kind:max([c['model_overlap_mm'] for c in maxima.values() if c['kind']==kind] or [0])
            for kind in ('native_or_existing_self_geometry','support_geometry','diagnostic_middle_ring_convex_hulls')},
        gravity=result['gravity_m_s2'],palm_origin=result['palm_origin_world_m'],palm_rotation=result['palm_rotation_world'])),flush=True)
    return result


def render_poses(m, names, hw):
    from PIL import Image, ImageDraw
    renderer=mujoco.Renderer(m,height=480,width=640)
    d=mujoco.MjData(m)
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[.07,0,.035]
    camera.distance=.37
    camera.azimuth=145
    camera.elevation=-28
    option=mujoco.MjvOption()
    option.geomgroup[4]=0
    frames=[]
    for i in range(7):
        d.qpos[:16]=hw_to_sim(hw[i]);d.qpos[-7:]=[0,0,2,1,0,0,0]
        mujoco.mj_forward(m,d)
        renderer.update_scene(d,camera=camera,scene_option=option)
        frame=Image.fromarray(renderer.render())
        ImageDraw.Draw(frame).text((12,12),f'Original recorded pose {i} - corrected motor mapping',fill='white')
        frame.save(OUT/f'pose_{i}.jpg')
        frames.append(frame)
    sheet=Image.new('RGB',(640*3,480*3),(25,25,25))
    for i,frame in enumerate(frames):sheet.paste(frame,((i%3)*640,(i//3)*480))
    sheet.save(OUT/'original_seven_poses.jpg')
    renderer.close()


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    names,hw,source=load_source()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,dm,manifest,added=make_models(env)
    audit(m,dm,names,hw,source,manifest,added)
    render_poses(m,names,hw)


if __name__=='__main__':
    main()
