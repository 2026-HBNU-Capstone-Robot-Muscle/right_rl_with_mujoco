"""Prepare source-bound offline evidence; no hardware access."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def write(path,r):path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    source=ROOT/'finish_empty_readonly_20261026_120039_495713/result.json'
    search=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v8_voltage_fresh/prefix_search/review.json'
    review=json.loads(search.read_bytes());state=json.loads(source.read_bytes())
    if not review['greedy_cleared'] or review['cases']!=67 or state['status']!='off_snapshots_saved':raise ValueError('Incomplete offline source')
    waypoints=review['greedy_waypoint_deltas']
    write(ROOT/'confirmed_thumb_palm_contact_20261026.json',dict(
        recorded_at=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        user_answer_ko='닿아 있음',observation='Thumb tip black pad touches the palm in the unchanged current OFF posture.',
        motor_motion_requested=False,simultaneous_photo_registration_verified=False,
        readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    write(ROOT/'confirmed_thumb_escape_prefix_20261026.json',dict(
        source=search.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(search.read_bytes()).hexdigest(),
        thumb_delta_waypoints_ticks=waypoints,static_initial_perturbations=67,
        all_initial_contacts_monotonically_decrease=True,new_contacts_forbidden=True,
        support_contacts_do_not_deepen=True,contact_forces_preserved=True,
        motor_commands_sent=0,physical_escape_tested=False))
    from thumb_clear_root_empty_plan_20261026 import FIXTURE,ThumbRootEmptyPlan
    old=json.loads((search.parent.parent/'request.json').read_bytes());old['plan_version']=ThumbRootEmptyPlan.PLAN_VERSION
    FIXTURE.mkdir(exist_ok=True);write(FIXTURE/'request.json',old)
    print(json.dumps(dict(prefix_goals=len(waypoints),maximum_thumb_travel_ticks=max(abs(v) for q in waypoints for v in q),waypoints=waypoints,motor_commands_sent=0)))
if __name__=='__main__':main()
