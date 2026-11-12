"""Offline sensitivity to a newly observed book below the fixed palm.

Book thickness and front position are unmeasured hypotheses. The saved motor
angles predate the book observation. This is never a current-start certificate.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
PHOTO=ROOT/'webcam_support_view_20261024_053217_646258'/'frame_3.png'
SOURCE=ROOT/'pose_pair_readonly_20261024_045449_430810'/'result.json'
OUT=ROOT/'right_hand_rotation_20261024'/'book_support_sensitivity'
OUT.mkdir(parents=True,exist_ok=True)
source_bytes=SOURCE.read_bytes();saved=json.loads(source_bytes)
assert saved['status']=='photo_and_readings_saved'
assert saved['servo_write_commands_sent']==0 and saved['torque_changes_requested']==0
q=np.array([next(v['position_ticks_signed'] for v in saved['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
baseline=env.mj_model;bd=mujoco.MjData(baseline);mujoco.mj_forward(baseline,bd)
palm=baseline.body('palm').id;origin=bd.xpos[palm].copy();rotation=bd.xmat[palm].reshape(3,3).copy()
quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())

def examine(model):
    d=mujoco.MjData(model)
    d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(model,d)
    support={model.geom(name).id for name in ('support_glass','support_wood','support_book')}
    hand={model.body('palm').id}
    for b in range(model.nbody):
        if int(model.body_parentid[b]) in hand:hand.add(b)
    pairs={};housing=0.
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if c.dist>=0:continue
        if not ((a in support and int(model.geom_bodyid[b]) in hand)
                or (b in support and int(model.geom_bodyid[a]) in hand)):continue
        key='|'.join(sorted((model.geom(a).name,model.geom(b).name)))
        overlap=-float(c.dist)*1000
        pairs[key]=max(pairs.get(key,0.),overlap)
        if '_support_proxy' in key:housing=max(housing,overlap)
    return dict(support_contacts_mm=pairs,maximum_support_model_overlap_mm=max(pairs.values(),default=0.),
                maximum_finger_housing_model_overlap_mm=housing)

records=[]
for glass_projection in (34.,50.):
    for thickness in (10.,20.,30.):
        for book_projection in (-20.,0.,20.):
            lower=np.array([-130.,-125.,11.3])/1000
            upper=np.array([3.+book_projection,65.,11.3+thickness])/1000
            center=origin+rotation@((lower+upper)/2)
            tree=ET.fromstring(Path(env.xml_path).read_text())
            ET.SubElement(tree.find('worldbody'),'geom',dict(name='support_book',type='box',
                size=' '.join(map(str,(upper-lower)/2)),pos=' '.join(map(str,center)),
                quat=' '.join(map(str,quat)),contype='4',conaffinity='11',
                friction='0.6 0.005 0.0001',rgba='0.85 0.85 0.75 1',group='0'))
            scene=OUT/'hypothesis_scene.xml';scene.write_text(ET.tostring(tree,encoding='unicode'),encoding='utf-8')
            wrapped=SimpleNamespace(mj_model=baseline,xml_path=scene,_model_assets=env._model_assets)
            model,manifest=build(wrapped,projection_mm=glass_projection,bottom_reference_mm=11.3+thickness)
            assert np.array_equal(model.jnt_range,baseline.jnt_range)
            assert np.array_equal(model.actuator_ctrlrange,baseline.actuator_ctrlrange)
            result=examine(model)
            record=dict(glass_projection_mm_assumed=glass_projection,book_thickness_mm_assumed=thickness,
                book_projection_from_fixed_front_mm_assumed=book_projection,**result)
            records.append(record)
        print(f'Offline hypotheses: glass {glass_projection:g}mm / book {thickness:g}mm complete.',flush=True)
report=dict(status='offline_unmeasured_book_sensitivity_complete',
    source_encoders=str(SOURCE.relative_to(ROOT)),source_encoder_sha256=hashlib.sha256(source_bytes).hexdigest(),
    source_photo=str(PHOTO.relative_to(ROOT)),source_photo_sha256=hashlib.sha256(PHOTO.read_bytes()).hexdigest(),
    visual_observation='Fixed hand body is above a white book; original rigid support is below the book.',
    book_dimensions_measured=False,encoder_pose_paired_with_book_photo=False,
    records=records,native_ranges_unchanged=True,hardware_commands_sent=0,real_devices_opened=False,
    hardware_execution_allowed=False,physical_contact_verified=False,
    limitations=['The 18 parameter combinations are hypotheses, not a measured or exhaustive uncertainty bound.',
        'Saved encoders were recorded before the book was observed and may not equal current physical angles.',
        'Support widths, rear lengths, friction and book compression are unmeasured.',
        'Book is included as a solid obstacle; lowering glass alone would omit possible new contacts.',
        'No path or dynamics validation, current-start approval, or physical failure-cause inference is provided.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
summary=dict(status=report['status'],hypotheses=len(records),
    housing_model_overlap_mm_range=[min(v['maximum_finger_housing_model_overlap_mm'] for v in records),
                                    max(v['maximum_finger_housing_model_overlap_mm'] for v in records)],
    complete_support_model_overlap_mm_range=[min(v['maximum_support_model_overlap_mm'] for v in records),
                                             max(v['maximum_support_model_overlap_mm'] for v in records)],
    zero_support_overlap_hypotheses=sum(v['maximum_support_model_overlap_mm']<=1e-6 for v in records),
    output=str(OUT/'result.json'),hardware_execution_allowed=False)
print(json.dumps(summary,indent=2),flush=True)
