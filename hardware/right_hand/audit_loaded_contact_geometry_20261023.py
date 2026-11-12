"""Compare same-run image hypotheses with recorded joint geometry, offline only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import argparse
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--rim',action='store_true');args=parser.parse_args()
    reg=json.loads((FOLDER/('rim_constrained_cube_registration.json' if args.rim else 'conditional_cube_registration.json')).read_bytes())
    trial=json.loads((FOLDER/'result.json').read_bytes())
    samples=trial['samples']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    cube=m.geom('cube').id;up=-np.array(m.opt.gravity);up/=np.linalg.norm(up)
    tips=('if_tip','mf_tip','rf_tip','th_tip');snapshots=[]
    for f in reg['rows']:
        if not f['hypotheses']:continue
        stamp=f['camera_read_monotonic']-f['enabled_at']
        sample=min(samples,key=lambda x:abs(x['t']-stamp))
        actual=np.array(sample['actual']);d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual)
        rows=[]
        for h in f['hypotheses']:
            R=np.array(h['cube_rotation_world']);t=np.array(h['cube_center_world_m'])
            quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
            d.qpos[-7:]=np.r_[t,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
            contacts=[]
            for c in d.contact:
                if cube in (int(c.geom1),int(c.geom2)):
                    other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                    contacts.append(dict(other=m.geom(other).name,overlap_mm=max(0.,-float(c.dist)*1000)))
            distances={n:float(mujoco.mj_geomDistance(m,d,cube,m.geom(n).id,1,None))*1000 for n in tips}
            rows.append(dict(**h,tip_gap_mm=distances,cube_contacts=contacts,
                maximum_cube_overlap_mm=max([c['overlap_mm'] for c in contacts]+[0.])))
        # Image fit is ranked independently, not to force expected contacts.
        rows.sort(key=lambda x:x['error_px'])
        joints={m.joint(i).name:d.xanchor[i].tolist() for i in range(16)}
        boxes=[]
        for i in range(m.ngeom):
            body=int(m.geom_bodyid[i]);name=m.body(body).name
            if int(m.geom_type[i])==6 and (name.startswith(('if_','mf_','rf_','th_')) or name=='palm'):
                signs=np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)])
                vertices=(signs*m.geom_size[i])@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
                boxes.append(dict(body=name,geom=m.geom(i).name,vertices=vertices.tolist()))
        snapshots.append(dict(image=f['image'],phase=f['phase'],sample_t=sample['t'],camera_sample_dt_s=sample['t']-stamp,
            measured_motor_ticks=actual.tolist(),joint_anchors_world=joints,boxes=boxes,hypotheses=rows))
        print(f['image'],'SYNC',round(sample['t']-stamp,4),flush=True)
        for h in rows[:4]:print('FIT',round(h['error_px'],2),'CENTER',np.round(1000*np.array(h['cube_center_world_m']),1).tolist(),
              'GAPS',{k:round(v,1) for k,v in h['tip_gap_mm'].items()},'OVERLAP',round(h['maximum_cube_overlap_mm'],1),flush=True)
    source=json.loads((ROOT/'right_hand_rotation_20261022/supported_yaw_result.json').read_bytes())
    baseline=[dict(reset=i,cube_center_world_m=s['initial_qpos'][-7:-4]) for i,s in enumerate(source['validation'])]
    out=dict(status='recorded_joint_vs_image_geometry_diagnostic',source=FOLDER.name,snapshots=snapshots,
             historical_search_cube_centers=baseline,model_manifest=manifest,registration_verified=False,
             hardware_commands_sent=0,hardware_execution_allowed=False,
             limitations=['A low reprojection error does not verify hand mounting offsets or planar depth.',
                          'Contact distances are model diagnostics, not measured contact forces.',
                          'No hand offsets, physical scales, or native joint/control ranges were fitted.'])
    (FOLDER/('rim_image_joint_contact_audit.json' if args.rim else 'image_joint_contact_audit.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
