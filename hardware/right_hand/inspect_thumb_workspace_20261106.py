"""Inspect accepted workspace proposals and their saved dynamic endpoints."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',choices=('thumb_global_workspace','index_countergrip_workspace','index_lower_regrasp_workspace'),default='thumb_global_workspace');ap.add_argument('--compact',action='store_true');args=ap.parse_args();folder=ROOT/args.group/('p12_seeds6_h-12' if args.group=='index_lower_regrasp_workspace' else 'p12_seeds6');r=json.loads((folder/'review.json').read_bytes());parent=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());m=mujoco.MjModel.from_binary_path(str(R/parent['source_model']));tip=m.geom('th_tip').id;cube=m.geom('cube').id;initial=np.load(R/r['source_state']);results=[]
    for c in r['cases']:
        if not c['static_accepted']:continue
        case=folder/f"{c['mode']}_{c['seed_index']}";proposal=np.load(case/'static_proposal.npz')
        for trial in c['dynamic']:
            if 'result' not in trial:continue
            z=np.load(case/f"force{trial['force_N']:g}_end.npz");d=restore(m,z);v=trial['result'];row=dict(mode=c['mode'],seed=c['seed_index'],force_N=trial['force_N'],center_delta_mm=((z['qpos'][-7:-4]-initial['qpos'][-7:-4])*1000).tolist(),actual_thumb_minus_proposal_rad=(z['qpos'][12:16]-proposal['actual_joint_proposal'][12:16]).tolist(),tracking_rad=(z['ctrl'][12:16]-z['qpos'][12:16]).tolist(),tip_cube_distance_mm=float(mujoco.mj_geomDistance(m,d,tip,cube,.1,None))*1000,final_contact_N=v['final_contact_N'],tail=v['settled_tail'],dynamic_safety=v['safety_passed']);results.append(row)
            if args.compact:print(json.dumps(dict(mode=c['mode'],seed=c['seed_index'],force=trial['force_N'],safety=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],regrasp_yaw=v['regrasp_yaw_change_deg'],tilt=v['final_up_tilt_deg'],shift=v['regrasp_translation_mm'],contacts=v['final_contact_N'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],contact_fraction=v['settled_tail']['opposed_lower_body_contact_fraction'],grip=v['whole_grip_task_passed'])),flush=True)
            else:print(json.dumps(row),flush=True)
    (folder/'endpoint_diagnostic.json').write_text(json.dumps(dict(cases=results,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
