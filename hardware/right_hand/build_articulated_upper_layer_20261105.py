"""Isolated two-body cube hypothesis for offline upper-layer load research.

Right-hand geometry, masses, actuator gains and bounds are copied back by name
and verified against the selected source model. Only the cube is replaced by
two uniform boxes with total mass .1kg and an uncalibrated axial bearing.
No robot runner imports or device calls are present.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import copy,hashlib,json,xml.etree.ElementTree as ET
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from cube_contact_material_policy_20261029 import apply_material
from research_rotation_body_hold_20261104 import inspect_for
from dynamic_common_grip_search_20261105 import ROOT,R

OUT=ROOT/'articulated_upper_layer'
SOURCE=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3/model.mjb'
BODY_FIELDS=('body_pos','body_quat','body_ipos','body_iquat','body_mass','body_inertia','body_gravcomp')
GEOM_FIELDS=('geom_type','geom_pos','geom_quat','geom_size','geom_friction','geom_solref','geom_solimp','geom_contype','geom_conaffinity','geom_condim','geom_priority','geom_margin','geom_gap','geom_solmix','geom_group')
JOINT_FIELDS=('jnt_pos','jnt_axis','jnt_range','jnt_limited','jnt_margin','jnt_stiffness')
ACT_FIELDS=('actuator_gainprm','actuator_biasprm','actuator_dynprm','actuator_gear','actuator_ctrlrange','actuator_forcerange','actuator_ctrllimited','actuator_forcelimited')

def numbers(x):return ' '.join(format(float(v),'.17g') for v in x)
def inertia(mass,half):return mass/3*np.array([half[1]**2+half[2]**2,half[0]**2+half[2]**2,half[0]**2+half[1]**2])

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    source=mujoco.MjModel.from_binary_path(str(SOURCE))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    original=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    model,_=build(env,OUT/'builder_scene.xml',book_projection_mm=-10.)
    correct_model(model,np.array(original['start_ticks']));set_cube_physics(model,.055,.1)
    model.opt.timestep=.001;model.opt.gravity[:]=source.opt.gravity
    positive=model.geom_solref[:,0]>0;model.geom_solref[positive,0]=np.minimum(model.geom_solref[positive,0],.004)
    hand,_,_=inspect_for(model);apply_material(model,.3,hand,model.geom('cube').id)
    serialized=OUT/'source_expanded.xml';mujoco.mj_saveLastXML(str(serialized),model)
    root=ET.fromstring(serialized.read_bytes());body=next(b for b in root.iter('body') if b.get('name')=='cube')
    for element in list(body):
        if element.tag=='geom' and element.get('name')!='cube':body.remove(element)
    lower_half=np.array([.0275,.055/3,.0275]);top_half=np.array([.0275,.055/6,.0275])
    inertial=body.find('inertial')
    if inertial is None:inertial=ET.SubElement(body,'inertial')
    inertial.attrib.clear();inertial.attrib.update(pos=numbers([0,.055/6,0]),mass=numbers([.1*2/3]),diaginertia=numbers(inertia(.1*2/3,lower_half)))
    geom=next(g for g in body.findall('geom') if g.get('name')=='cube')
    geom.attrib.pop('mesh',None);geom.attrib.update(type='box',pos=numbers([0,.055/6,0]),size=numbers(lower_half),mass='0',density='0')
    child=ET.SubElement(body,'body',dict(name='upper_layer',pos=numbers([0,-.055/3,0])))
    ET.SubElement(child,'inertial',dict(pos='0 0 0',mass=numbers([.1/3]),diaginertia=numbers(inertia(.1/3,top_half))))
    ET.SubElement(child,'joint',dict(name='upper_layer_axis',type='hinge',axis='0 -1 0',limited='false',damping='.0001',frictionloss='.003'))
    upper=copy.deepcopy(geom);upper.attrib.update(name='upper_layer_collision',pos='0 0 0',size=numbers(top_half),rgba='.95 .65 .1 1');child.append(upper)
    # Old keyframes have no value for the added hinge and are not used here.
    keyframe=root.find('keyframe')
    if keyframe is not None:root.remove(keyframe)
    xml=ET.tostring(root,encoding='unicode');(OUT/'articulated_scene.xml').write_text(xml,encoding='utf-8')
    m=mujoco.MjModel.from_xml_string(xml,assets=dict(env._model_assets))
    source_hand,source_support,_=inspect_for(source);verified=0
    for old in sorted(source_hand):
        name=source.body(old).name;new=m.body(name).id
        for field in BODY_FIELDS:getattr(m,field)[new]=getattr(source,field)[old]
        og=[g for g in range(source.ngeom) if int(source.geom_bodyid[g])==old]
        ng=[g for g in range(m.ngeom) if int(m.geom_bodyid[g])==new]
        if len(og)!=len(ng):raise ValueError('right-hand geometry count changed: '+name)
        for a,b in zip(og,ng):
            if source.geom(a).name!=m.geom(b).name:raise ValueError('right-hand geometry order changed')
            for field in GEOM_FIELDS:getattr(m,field)[b]=getattr(source,field)[a]
            if int(source.geom_type[a])==mujoco.mjtGeom.mjGEOM_MESH:
                sm,tm=int(source.geom_dataid[a]),int(m.geom_dataid[b]);sa,sn=int(source.mesh_vertadr[sm]),int(source.mesh_vertnum[sm]);ta,tn=int(m.mesh_vertadr[tm]),int(m.mesh_vertnum[tm])
                if sn!=tn or not np.allclose(source.mesh_vert[sa:sa+sn],m.mesh_vert[ta:ta+tn],atol=1e-9,rtol=0):raise ValueError('source mesh hull vertices changed')
        verified+=len(og)
    for old in source_support:
        name=source.geom(old).name;new=m.geom(name).id
        for field in GEOM_FIELDS:getattr(m,field)[new]=getattr(source,field)[old]
    for old in range(16):
        new=m.joint(source.joint(old).name).id
        for field in JOINT_FIELDS:getattr(m,field)[new]=getattr(source,field)[old]
        od=int(source.jnt_dofadr[old]);nd=int(m.jnt_dofadr[new])
        for field in ('dof_damping','dof_armature','dof_frictionloss'):getattr(m,field)[nd]=getattr(source,field)[od]
    for field in ACT_FIELDS:getattr(m,field)[:]=getattr(source,field)
    cube=m.geom('cube').id;top=m.geom('upper_layer_collision').id;oldcube=source.geom('cube').id
    for field in ('geom_friction','geom_solref','geom_solimp','geom_contype','geom_conaffinity','geom_condim','geom_priority','geom_margin','geom_gap','geom_solmix'):
        getattr(m,field)[cube]=getattr(source,field)[oldcube];getattr(m,field)[top]=getattr(source,field)[oldcube]
    mujoco.mj_setConst(m,mujoco.MjData(m))
    for old in source_hand:
        new=m.body(source.body(old).name).id
        for field in BODY_FIELDS:assert np.array_equal(getattr(m,field)[new],getattr(source,field)[old]),field
    for field in ACT_FIELDS:assert np.array_equal(getattr(m,field),getattr(source,field)),field
    assert m.nu==source.nu==16 and m.nq==source.nq+1 and m.nv==source.nv+1
    cb=m.body('cube').id;tb=m.body('upper_layer').id
    assert abs(m.body_mass[cb]+m.body_mass[tb]-.1)<1e-12
    out=OUT/'model.mjb';mujoco.mj_saveModel(m,str(out))
    report=dict(status='isolated_articulated_upper_layer_model_hypothesis',source_model=str(SOURCE.relative_to(R)),source_model_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),model_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),right_hand_geoms_verified=verified,right_hand_masses_actuators_geometry_and_native_bounds_preserved=True,cube_side_mm=55,total_cube_mass_kg=.1,two_uniform_bodies=True,uncalibrated_layer_bearing=True,not_a_full_rubik_mechanism=True,actual_left_hand_geometry_absent=True,hardware_execution_allowed=False,devices_opened=False)
    (OUT/'model_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
