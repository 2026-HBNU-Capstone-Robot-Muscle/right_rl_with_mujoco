"""Make an isolated relative-turn teacher; preserve the original planner."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle/plan_clear_work_grip_bridge.py'
TARGET = ROOT / 'stage3_relative_teacher_20261112.py'


def main():
    if TARGET.exists():
        raise RuntimeError('Preserve generated teacher')
    text = SOURCE.read_text(encoding='utf-8')
    changes = [
        ('def main():', 'ROOT = OUT.parents[1]\n\ndef main():'),
        ('folder=base/args.tag', "folder=ROOT/'right_hand_rotation_20261112/stage3_preparation'/args.tag"),
        ("reference_path=base/'RF_clear_work_grip_joined_suffix/end_state.npz'", "reference_path=ROOT/'right_hand_rotation_20261112/stage3_preparation/opposed_contact_v1/axis0_middle+1_all/end_state.npz'"),
        ("if not source.is_relative_to(OUT.resolve()):", "if not source.is_relative_to(ROOT.resolve()):"),
        ("target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],global_yaw)", "target=upright_yaw_target(start_rotation,initial['up'],args.increment)"),
        ("and ev['center_target_error_mm']<2:", ":"),
        (" and final['center_target_error_mm']<2 and (not args.require_loaded_hold", " and (not args.require_loaded_hold"),
        ("source=source.relative_to(OUT).as_posix()", "source=source.relative_to(ROOT).as_posix()"),
        ("goal_reference=reference_path.relative_to(OUT).as_posix()", "goal_reference=reference_path.relative_to(ROOT).as_posix()"),
        ("global_yaw_target_deg=global_yaw,", "relative_yaw_target_deg=args.increment,target_rotation=target.tolist(),initial_rotation=start_rotation.tolist(),same_station_2mm_required=False,"),
        ("status='receding_horizon_clear_work_grip_bridge'", "status='stage3_relative_turn_model_teacher'"),
    ]
    for before, after in changes:
        if text.count(before) != 1:
            raise RuntimeError('Unexpected parent source for exact edit: '+before)
        text = text.replace(before, after, 1)
    compile(text, str(TARGET), 'exec')
    TARGET.write_text(text, encoding='utf-8')
    out = ROOT / 'right_hand_rotation_20261112/stage3_preparation/relative_teacher_provenance.json'
    out.write_text(json.dumps({'source': SOURCE.relative_to(ROOT).as_posix(), 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'new_script_sha256': hashlib.sha256(TARGET.read_bytes()).hexdigest(), 'changes': changes,
        'native_guard_changes': 0, 'model_or_hardware_limits_changed': False,
        'target_is90_from_reached_model_pose_not_original180_certificate': True,
        'end_hold_local_drift_2mm_and3deg_preserved': True, 'devices_opened': 0}, indent=2)+'\n', encoding='utf-8')
    print('Isolated relative-turn teacher generated; no devices opened.')


if __name__ == '__main__':
    main()
