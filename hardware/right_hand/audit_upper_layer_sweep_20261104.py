"""Sampled convex sweep with a Hausdorff bound for the angles between samples."""
import json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import CP,OUT,restore
from audit_upper_layer_clearance_20261104 import local_mesh,box_gap

def sweep(v,step=1.):
    vals=[]
    for theta in np.arange(0,90.+step/2,step):
        A=Rotation.from_rotvec(np.array([0.,np.radians(theta),0.])).as_matrix()
        vals.append((float(theta),box_gap(v@A,np.array([0.,-55/3.,0.]),np.array([27.5,55/6.,27.5]))))
    angle,gap=min(vals,key=lambda x:x[1]);bound=float(2*27.5*np.sqrt(2)*np.sin(np.radians(step)/4))
    return dict(minimum_sampled_separating_gap_mm=gap,angle_at_minimum_deg=angle,between_sample_motion_bound_mm=bound,continuous_sweep_separation_proved=bool(gap>bound),sampled_overlap_predicted=bool(gap<0),step_deg=step)

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));cube=m.geom('cube').id;rows=[]
    paths=[('before_lowering',CP/'closest_state.npz')]+[(p.parent.name,p) for p in sorted(OUT.glob('lower_pair_*/end_state.npz'))]
    for name,path in paths:
        d=restore(m,np.load(path));geometries=[]
        for tip in ('mf_tip','th_tip'):
            g=m.geom(tip).id;v=local_mesh(m,d,g,cube)
            geometries.append(dict(geom=tip,**sweep(v)))
        row=dict(name=name,geometry=geometries,both_tips_clear_through_90deg=all(g['continuous_sweep_separation_proved'] for g in geometries));rows.append(row)
        print(json.dumps(row),flush=True)
    report=dict(status='upper_layer_90deg_sweep_CAD_check',cases=rows,physical_clearance_verified=False,whole_hand_clearance_verified=False,independent_layer_dynamics_verified=False,devices_opened=False)
    (OUT/'upper_layer_sweep_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
