"""Build new local assets without editing the old reviewed robot files."""
import json
import numpy as np
from final_grip_pose_plan_20261116 import ROOT, OUT, REFERENCE, REQUEST, CANDIDATE, MANIFEST, MOVE, HOLD, FinalGripPosePlan, sha
from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK
from new_pair_grasp_plan_20261027 import OPEN_PATH
from calibrated_empty_ring_margin_20261028 import interpolate

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if any(p.exists() for p in (REQUEST, CANDIDATE, MANIFEST)):
        raise RuntimeError('Preserve existing final-pose assets')
    target = np.empty(16, dtype=int)
    target[np.asarray(SIM_TO_MOTOR_IDS)-1] = np.rint(np.load(REFERENCE)['qpos'][:16]/TICK+2048).astype(int)
    opened = np.load(OPEN_PATH)['open_goal']
    goals = interpolate(opened, target)
    phases = [MOVE]*len(goals)+[HOLD]*40
    goals = np.vstack([goals, np.tile(target, (40, 1))])
    np.savez_compressed(CANDIDATE, start=opened, goals=goals, phases=np.asarray(phases))
    MANIFEST.write_text(json.dumps(dict(candidate_sha256=sha(CANDIDATE), reference_path=REFERENCE.relative_to(ROOT).as_posix(),
        reference_sha256=sha(REFERENCE), target_source='rounded measured model qpos; NOT pressure ctrl',
        physical_target_ticks=target.tolist(), empty_only=True, cube_grasp_verified=False), indent=2)+'\n', encoding='utf-8')
    src = ROOT/'support_first_trial_20261113_042739_709487/fresh_off_request.json'
    request = json.loads(src.read_bytes())
    request.update(plan_version=FinalGripPosePlan.PLAN_VERSION, source=src.relative_to(ROOT).as_posix(), source_sha256=sha(src), hardware_execution_allowed=False)
    REQUEST.write_text(json.dumps(request, indent=2)+'\n', encoding='utf-8')
    # Separate copy: includes the entire final-pose approach in the old 201/9 screen.
    source = (ROOT/'screen_empty_initial_scope_20261113.py').read_text(encoding='utf-8')
    original = '(ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE)'
    if source.count(original) != 2:
        raise RuntimeError('Upstream screen layout changed')
    source = source.replace(original, '(ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE,MOVE,HOLD)')
    source = source.replace("if __name__=='__main__':main()", '')
    source += '\nfrom final_grip_pose_plan_20261116 import FinalGripPosePlan, MOVE, HOLD\nOfflineRecovery=FinalGripPosePlan\nif __name__=="__main__":main()\n'
    (ROOT/'screen_final_grip_pose_full_20261116.py').write_text(source, encoding='utf-8')
    plan = FinalGripPosePlan(REQUEST)
    rows = plan.plan(plan.source_position())
    print(json.dumps(dict(target=target.tolist(), goals=len(rows), motion_seconds=len(rows)*.075, devices_opened=False)))

if __name__ == '__main__':
    main()
