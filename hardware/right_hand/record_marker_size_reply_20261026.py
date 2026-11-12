"""Persist the user's approximate red-face dimension without upgrading precision."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026'
def main():
    path=OUT/'marker_dimension_observations_20261026.json';r=json.loads(path.read_bytes()) if path.exists() else dict(observations=[])
    ident='red2_user_reply_10p5_10p7'
    if not any(v.get('id')==ident for v in r['observations']):r['observations'].append(dict(id=ident,timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),face_id=2,face_name='red',user_reply='10.5?10.7? 그쯤 맞는데',side_mm_approximate_range=[10.5,10.7],precision_not_verified=True,white_margin_excluded_in_question=True))
    r.update(red_marker_nominal_for_sensitivity_mm=10.6,red_marker_size_sensitivity_mm=[10.5,10.6,10.7],prior_photo_estimate_9p8mm_is_not_accepted_physical_size=True,green_marker_previous_user_estimate_mm=10.7,cube_side_user_mm=55.,no_live_configuration_changed=True)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(r,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
