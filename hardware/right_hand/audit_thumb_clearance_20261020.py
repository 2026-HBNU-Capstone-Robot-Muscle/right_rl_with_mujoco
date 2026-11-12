"""Offline sampled clearance review. Does not import a motor interface."""
import os
os.environ.setdefault("MUJOCO_GL","egl")
import hashlib
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS
from offline_housing_proxy_model import build_with_middle_thumb_hulls

ROOT=Path(__file__).resolve().parent
OUT=ROOT/"right_hand_root_success_20261020"

def main(pose_pair=None, output=None):
    # Check signed distance units against simple known geometry.
    test=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="a" type="sphere" size=".01"/><geom name="b" type="sphere" size=".01" pos=".03 0 0"/></worldbody></mujoco>')
    td=mujoco.MjData(test);mujoco.mj_forward(test,td)
    assert abs(mujoco.mj_geomDistance(test,td,0,1,.1,None)-.01)<1e-8
    out=Path(output) if output else OUT
    out.mkdir(exist_ok=True)
    source=Path(pose_pair) if pose_pair else OUT/"source_result.json"
    trial=json.loads(source.read_bytes())
    if pose_pair:
        from leap_pose_pair_readonly import validate_pair
        assert trial['status']=='photo_and_readings_saved'
        assert validate_pair(*trial['snapshots'])['stationary_endpoints']
        for snapshot in trial['snapshots']:
            assert snapshot['joint_mapping']['motor_ids_in_sim_order']==list(SIM_TO_MOTOR_IDS)
        start=np.array([v['position_ticks_signed'] for v in sorted(trial['snapshots'][-1]['motors'],key=lambda v:v['id'])])
        source_kind='Read-only paired photo and encoder endpoints; later posture changes unknown.'
    else:
        start=np.array(trial["final_on_state"]["position"])
        source_kind='Current OFF pose is unknown. Source is final ON telemetry.'
    targetfile=ROOT/"full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json"
    target=np.array(json.loads(targetfile.read_text())["start_ticks"])
    cfg=registry.get_default_config("LeapCubeFaceUp");cfg.impl="jax"
    env=registry.load("LeapCubeFaceUp",config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    palm=m.body("palm").id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ("support_glass","support_wood")}
    names=[m.geom(g).name or "" for g in range(m.ngeom)]
    palms=[g for g,n in enumerate(names) if n.startswith("palm_collision")]
    tip=m.geom("th_tip").id
    middle=[g for g,n in enumerate(names) if n.startswith("mf_") and "_offline_hull_" in n]
    thumbs=[g for g,n in enumerate(names) if n.startswith("th_") and "_offline_hull_" in n]
    pairs=[(tip,g) for g in palms]+[(a,b) for a in middle for b in thumbs]
    def separation(g,h):
        if int(m.geom_type[g])==6 and int(m.geom_type[h])==6:
            # Independent separating-axis lower bound. mj_geomDistance's
            # box query returned zero despite separated closest-point output.
            ra=d.geom_xmat[g].reshape(3,3);rb=d.geom_xmat[h].reshape(3,3)
            axes=np.concatenate((ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)))
            norms=np.linalg.norm(axes,axis=1);axes=axes[norms>1e-10]/norms[norms>1e-10,None]
            gaps=np.abs(axes@(d.geom_xpos[h]-d.geom_xpos[g]))
            gaps-=np.abs(axes@ra)@m.geom_size[g]+np.abs(axes@rb)@m.geom_size[h]
            return max(0.,float(gaps.max()))
        return float(mujoco.mj_geomDistance(m,d,g,h,.05,None))
    rng=np.random.default_rng(20261020)
    offsets=[np.zeros(16,dtype=int),np.full(16,-32),np.full(16,32)]
    offsets += [rng.integers(-32,33,16) for _ in range(32)]
    results={}
    for change in (-16,-32,-64):
        rows=[]
        for sample,offset in enumerate(offsets):
            a=start+offset;b=a.copy();b[:4]=target[:4];b[0]+=change
            n=max(1,int(np.ceil(np.max(abs(b-a))/5)))
            min_gap=.05;nearest=None;self_depth=0.;support_depth=0.
            for k in range(n+1):
                raw=np.rint(a+(b-a)*k/n).astype(int)
                d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
                mujoco.mj_forward(m,d)
                for c in d.contact:
                    g,h=int(c.geom1),int(c.geom2)
                    gh,hh=int(m.geom_bodyid[g]) in hand,int(m.geom_bodyid[h]) in hand
                    if gh and hh:self_depth=max(self_depth,-float(c.dist))
                    if (g in supports and hh) or (h in supports and gh):
                        support_depth=max(support_depth,-float(c.dist))
                for g,h in pairs:
                    gap=separation(g,h)
                    if gap<min_gap:
                        min_gap=float(gap)
                        nearest=dict(pair=[names[g],names[h]],fraction=k/n)
            rows.append(dict(sample=sample,start_offset_ticks=offset.tolist(),samples=n+1,
                min_selected_gap_mm=min_gap*1000,nearest=nearest,
                max_self_overlap_mm=self_depth*1000,max_support_overlap_mm=support_depth*1000))
        results[str(change)]=dict(nominal=rows[0],sampled_starts=len(rows),
            worst_selected_gap_mm=min(x["min_selected_gap_mm"] for x in rows),
            max_self_overlap_mm=max(x["max_self_overlap_mm"] for x in rows),
            max_support_overlap_mm=max(x["max_support_overlap_mm"] for x in rows),rows=rows)
        print(json.dumps({str(change):{k:v for k,v in results[str(change)].items() if k!="rows"} }),flush=True)
    report=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_path=str(source),
        source_kind=source_kind,start_ticks=start.tolist(),
        target_sha256=hashlib.sha256(targetfile.read_bytes()).hexdigest(),model_manifest=manifest,
        selected_pairs=[[names[a],names[b]] for a,b in pairs],results=results,
        contact_observation="User did not observe clearly; physical contact remains unknown.",
        hardware_execution_allowed=False,motor_commands_sent=0,
        limitations=["Sampled static paths; not exhaustive bounds, dynamic validation or physical calibration.",
            "Box pairs use an independent SAT separation lower bound; mesh pairs use MuJoCo convex distance. Minimum is a conservative clearance estimate, not exact geometry calibration.",
            "Clearance distances cover listed pairs only; contact overlap checks cover enabled hand/support pairs.",
            source_kind,
            "ID3 measured start outside native range is retained without wrap, limit changes or calibration claims."])
    (out/"thumb_clearance_sensitivity.json").write_text(json.dumps(report,indent=2),encoding="utf-8")

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('--pose-pair');parser.add_argument('--output')
    args=parser.parse_args()
    main(args.pose_pair,args.output)
