"""Model support accounting for every hand body; no device access.

Support is assessed together with the caller's unchanged pose, drift, clearance,
duration and native guards. A positive contact force alone is not a stable grasp
or a test of resistance to the other hand's future load.
"""
import numpy as np


CONTRACT = dict(
    version='whole_hand_support_v1',
    hand_membership='palm body and all descendants, by body ID',
    loaded_geometry_normal_force_N=.2,
    loaded_geometry_threshold_source='Existing per-geometry force threshold retained for diagnostics only',
    minimum_upward_hand_load_N=.2,
    support_threshold_source='Explicit research criterion for whole-hand upward resultant using the existing0.2N force scale; not a hardware load calibration',
    upward_resultant_required=True,
    external_loaded_contact_allowed=False,
    final_window_samples=4050,
    required_supported_sample_fraction_strictly_above=.8,
    pose_drift_duration_clearance_and_native_guards_are_separate_required_gates=True,
    weight_support_ratio_is_diagnostic_not_a_new_tuned_acceptance_threshold=True,
    is_force_closure_or_external_load_capacity_certificate=False,
    hardware_execution_allowed=False)


def force_on_cube(local_force, frame, cube_is_geom1):
    """MuJoCo contact frame rows are world axes; force acts on geom2."""
    local_force=np.asarray(local_force,dtype=float)
    frame=np.asarray(frame,dtype=float).reshape(3,3)
    return (-1. if cube_is_geom1 else 1.)*(frame.T@local_force)


def summarize_contacts(contacts, hand_bodies, gravity, weight_N):
    gravity=np.asarray(gravity,dtype=float)
    if gravity.shape!=(3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity)<=0:
        raise ValueError('Support requires a finite nonzero gravity vector')
    if not np.isfinite(weight_N) or weight_N<=0:
        raise ValueError('Support requires positive cube weight')
    up=-gravity/np.linalg.norm(gravity)
    hand_bodies=set(hand_bodies);by_geom={};external=[]
    for contact in contacts:
        force=np.asarray(contact['force_world_N'],dtype=float)
        normal=float(contact['normal_N'])
        if force.shape!=(3,) or not np.isfinite(force).all() or not np.isfinite(normal) or normal<0:
            raise ValueError('Invalid contact force')
        if int(contact['other_body_id']) not in hand_bodies:
            if normal>1e-9:
                external.append(dict(geom=contact['geom'],normal_N=normal,force_world_N=force.tolist()))
            continue
        name=contact['geom']
        entry=by_geom.setdefault(name,dict(normal_N=0.,force_world_N=np.zeros(3)))
        entry['normal_N']+=normal;entry['force_world_N']+=force
    total=sum((entry['force_world_N'] for entry in by_geom.values()),np.zeros(3))
    loaded=[name for name,entry in by_geom.items() if entry['normal_N']>CONTRACT['loaded_geometry_normal_force_N']]
    normal_sum=sum(entry['normal_N'] for entry in by_geom.values())
    upward=float(total@up)
    for entry in by_geom.values():
        entry['upward_N']=float(entry['force_world_N']@up)
        entry['force_world_N']=entry['force_world_N'].tolist()
    return dict(
        supported_sample=bool(normal_sum>0 and upward>CONTRACT['minimum_upward_hand_load_N'] and not external),
        hand_normal_load_N=normal_sum,
        contact_hand_geoms=sorted(name for name,entry in by_geom.items() if entry['normal_N']>0),
        loaded_hand_geoms=sorted(loaded),by_geom=by_geom,
        hand_force_world_N=total.tolist(),upward_support_N=upward,
        weight_N=float(weight_N),upward_support_over_weight=upward/float(weight_N),
        external_loaded_contacts=external,
        force_with_gravity_world_N=(total-float(weight_N)*up).tolist())


def sample_model(model,data,hand_bodies,cube):
    """Lazy MuJoCo adapter. No model or state mutation; force direction is explicit."""
    import mujoco
    contacts=[]
    for ci,contact in enumerate(data.contact):
        first,second=int(contact.geom1),int(contact.geom2)
        if cube not in (first,second):continue
        other=second if first==cube else first
        local=np.zeros(6);mujoco.mj_contactForce(model,data,ci,local)
        contacts.append(dict(
            geom=model.geom(other).name or 'geom_'+str(other),
            other_body_id=int(model.geom_bodyid[other]),normal_N=max(0.,float(local[0])),
            force_world_N=force_on_cube(local[:3],contact.frame,first==cube)))
    body=int(model.geom_bodyid[cube])
    return summarize_contacts(contacts,hand_bodies,model.opt.gravity,
                              float(model.body_mass[body]*np.linalg.norm(model.opt.gravity)))


class SupportWindow:
    def __init__(self):
        self.samples=self.supported_samples=self.external_loaded_samples=0
        self.min_ratio=float('inf');self.max_ratio=-float('inf')
        self.loaded_geom_samples={};self.force_sum=np.zeros(3)
        self.contact_geom_samples={}

    def add(self,sample):
        self.samples+=1;self.supported_samples+=int(sample['supported_sample'])
        self.external_loaded_samples+=bool(sample['external_loaded_contacts'])
        self.min_ratio=min(self.min_ratio,sample['upward_support_over_weight'])
        self.max_ratio=max(self.max_ratio,sample['upward_support_over_weight'])
        self.force_sum+=sample['hand_force_world_N']
        for name in sample['loaded_hand_geoms']:
            self.loaded_geom_samples[name]=self.loaded_geom_samples.get(name,0)+1
        for name in sample['contact_hand_geoms']:
            self.contact_geom_samples[name]=self.contact_geom_samples.get(name,0)+1

    def report(self):
        fraction=self.supported_samples/max(1,self.samples)
        passed=(self.samples==CONTRACT['final_window_samples'] and
                fraction>CONTRACT['required_supported_sample_fraction_strictly_above'] and
                self.external_loaded_samples==0)
        return dict(samples=self.samples,supported_samples=self.supported_samples,
                    supported_fraction=fraction,external_loaded_samples=self.external_loaded_samples,
                    loaded_geom_samples=dict(sorted(self.loaded_geom_samples.items())),
                    contact_geom_samples=dict(sorted(self.contact_geom_samples.items())),
                    upward_support_over_weight_min=self.min_ratio if self.samples else None,
                    upward_support_over_weight_max=self.max_ratio if self.samples else None,
                    hand_force_world_N_mean=(self.force_sum/max(1,self.samples)).tolist(),
                    support_window_passed=bool(passed),
                    stable_grasp_requires_separate_pose_drift_clearance_and_native_gates=True)
