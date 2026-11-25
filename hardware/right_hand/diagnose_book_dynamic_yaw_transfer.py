"""Measured replay and missing middle/ring housing diagnostic, no devices.

Added visual-mesh hulls are scoped to the diagnostic middle/ring pair. Their
convex overlap is not asserted to be a physical penetration measurement.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_dynamic_yaw_trial_20261024_222132_806567'
BASE='stage_4_book_grasp_hold';TURN='stage_5_book_small_clockwise';AFTER='stage_6_book_yaw_hold'


def main():
    blob=(FOLDER/'result.json').read_bytes();trial=json.loads(blob)
    before=[s for s in trial['samples'] if s['phase']==BASE][-20:]
    initial=np.median([s['actual'] for s in before],axis=0);base=np.array(before[-1]['goal'])
    path=[s for s in trial['samples'] if s['phase'] in (TURN,AFTER,'stage_2_fixed_hold')]
    times=np.r_[0.,[s['t']-path[0]['t']+.05 for s in path]]
    actual=np.vstack([initial,[s['actual'] for s in path]])
    assumed=np.vstack([initial,[np.array(s['goal'])+initial-base for s in path]])
    closure=np.vstack([actual[:27],np.tile(actual[26],(len(actual)-27,1))])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    # Preserve existing runtime checker/model files. Add cross-finger housings
    # only to this separate scene; no tolerance or actual stop is changed.
    assets=copy.deepcopy(dict(env._model_assets));added=[]
    for name,value in list(assets.items()):
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(value);changed=False
        for body in xml.iter('body'):
            bname=body.get('name','');middle=bname.startswith('mf_');ring=bname.startswith('rf_')
            if not (middle or ring):continue
            for i,geom in enumerate(list(body.findall('geom'))):
                if geom.get('mesh') is None:continue
                proxy=copy.deepcopy(geom);proxy.attrib.pop('class',None)
                pname=f'diagnostic_mf_rf_{bname}_{i}'
                proxy.attrib.update(name=pname,type='mesh',density='0',mass='0',group='4',rgba='0 0 0 0',
                    contype='64' if middle else '128',conaffinity='128' if middle else '64')
                body.append(proxy);added.append(pname);changed=True
        if changed:assets[name]=ET.tostring(xml,encoding='utf-8')
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets)
    dm,manifest=build_book(wrapped,FOLDER/'middle_ring_diagnostic_scene.xml',book_projection_mm=-10.,
        book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.,additional_self_hulls=False)
    diagnostic_ids={dm.geom(n).id for n in added};dcube=dm.geom('cube').id
    dm.geom_contype[dcube]=dm.geom_conaffinity[dcube]=0;dd=mujoco.MjData(dm)
    maxima={};contacts=[]
    for index,s in enumerate(trial['samples']):
        if index%3 and s['phase'] not in (BASE,TURN,AFTER):continue
        dd.qpos[:16]=ticks_to_sim(s['actual']);dd.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(dm,dd)
        found=[]
        for c in dd.contact:
            a,b=int(c.geom1),int(c.geom2)
            if a not in diagnostic_ids or b not in diagnostic_ids or c.dist>=0:continue
            key='|'.join(sorted((dm.geom(a).name,dm.geom(b).name)));depth=-float(c.dist)*1000
            found.append(dict(pair=key,depth_mm=depth))
            if depth>maxima.get(key,{}).get('depth_mm',0):
                maxima[key]=dict(depth_mm=depth,phase=s['phase'],sample_index=index,position_world_m=c.pos.tolist())
        if found:contacts.append(dict(sample=index,phase=s['phase'],contacts=found))
    print('MIDDLE_RING_DIAGNOSTIC',json.dumps(dict(hulls=len(added),contact_samples=len(contacts),maxima=maxima)),flush=True)
    m,_=build_book(env,FOLDER/'measured_replay_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    prior=json.loads((ROOT/'right_hand_rotation_20261024/book_yaw_measured_response/relative_candidate_replay.json').read_bytes())
    center=np.array(prior['selected_placement_hypothesis']['center_m']);R=np.array(prior['cube_rotation_world_hypothesis'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    runs=[]
    def simulate(trace,friction):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
        d.qpos[:16]=ticks_to_sim(initial);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=np.r_[center,quat];mujoco.mj_forward(m,d)
        for _ in range(round(2/m.opt.timestep)):
            d.qpos[:16]=ticks_to_sim(initial);d.qvel[:16]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_step(m,d)
        start=d.qpos[-7:].copy();R0=d.xmat[body].reshape(3,3).copy();elapsed=0.
        while elapsed<=times[-1]+1.:
            q=np.array([np.interp(elapsed,times,trace[:,i]) for i in range(16)])
            nxt=np.array([np.interp(elapsed+m.opt.timestep,times,trace[:,i]) for i in range(16)])
            angles=ticks_to_sim(q);d.qpos[:16]=angles;d.ctrl[:]=angles
            d.qvel[:16]=(ticks_to_sim(nxt)-angles)/m.opt.timestep;mujoco.mj_step(m,d);elapsed+=m.opt.timestep
        rv=Rotation.from_matrix(d.xmat[body].reshape(3,3)@R0.T).as_rotvec();shift=(d.qpos[-7:-4]-start[:3])*1000
        cube_contacts=sorted({m.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name
            for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003})
        return dict(yaw_deg=float(np.degrees(rv@up)),tilt_deg=float(np.degrees(np.linalg.norm(rv-up*(rv@up)))),
            lateral_shift_mm=float(np.linalg.norm(shift-up*(shift@up))),final_cube_contacts=cube_contacts,
            finite=bool(np.isfinite(d.qpos).all()))
    for friction in (.4,.8,1.2):
        row=dict(friction=friction,constant_response_assumption=simulate(assumed,friction),
            measured_hand_trace=simulate(actual,friction),measured_closure_then_hold=simulate(closure,friction))
        runs.append(row);print('MEASURED_RESPONSE_REPLAY',json.dumps(row),flush=True)
    output=dict(source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        initial_measured_ticks=initial.tolist(),constant_response_end_error_ticks=(actual[-1]-assumed[-1]).tolist(),
        middle_ring_housing_hulls_added_only_for_diagnostic=added,middle_ring_housing_model_contacts=maxima,
        middle_ring_housing_contact_sample_count=len(contacts),middle_ring_contact_samples=contacts,
        existing_runtime_housing_proxies_cover_middle_thumb_not_middle_ring=True,
        physical_middle_ring_contact_verified=False,convex_hull_overlap_not_measured_physical_depth=True,
        cube_replay=runs,marker_registration_verified=False,cube_friction_measured=False,
        hand_trace_is_prescribed_not_actuator_prediction=True,hardware_commands_sent=0,
        physical_target_rotation_verified=False)
    (FOLDER/'transfer_diagnostic.json').write_text(json.dumps(output,indent=2),encoding='utf-8')


if __name__=='__main__':main()
