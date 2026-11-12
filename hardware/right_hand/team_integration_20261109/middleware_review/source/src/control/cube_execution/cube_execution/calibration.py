"""Fail-closed validation for the physical hardware calibration record."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


_REQUIRED_VERIFIED_FLAGS = (
    ('verified',),
    ('pose', 'verified'),
    ('frames', 'verified'),
    ('motors', 'leap', 'verified'),
    ('motors', 'custom_fingers', 'verified'),
    ('motors', 'custom_wrist', 'verified'),
    ('policies', 'leap', 'verified'),
    ('policies', 'custom_hand', 'verified'),
    ('safety', 'emergency_stop_verified'),
    ('safety', 'stop_behavior_verified'),
    ('safety', 'joint_limits_verified'),
    ('safety', 'workspace_limits_verified'),
    ('safety', 'completion_feedback_verified'),
    ('safety', 'handoff_verified'),
)

_REQUIRED_TEXT_FIELDS = (
    ('verified_by',),
    ('verified_at_utc',),
    ('pose', 'topic'),
    ('pose', 'message_type'),
    ('pose', 'frame_id'),
    ('pose', 'position_unit'),
    ('pose', 'orientation_convention'),
    ('frames', 'robot_base_frame'),
    ('frames', 'cube_frame'),
    ('frames', 'leap_base_frame'),
    ('frames', 'custom_base_frame'),
    ('frames', 'custom_wrist_frame'),
    ('motors', 'leap', 'port'),
    ('motors', 'custom_fingers', 'port'),
    ('motors', 'custom_wrist', 'port'),
    ('policies', 'leap', 'model_path'),
    ('policies', 'leap', 'sha256'),
    ('policies', 'custom_hand', 'model_path'),
    ('policies', 'custom_hand', 'sha256'),
)

_REQUIRED_POSITIVE_NUMBERS = (
    ('pose', 'max_pose_age_ms'),
    ('motors', 'leap', 'baud_rate'),
    ('motors', 'custom_fingers', 'baud_rate'),
    ('motors', 'custom_wrist', 'baud_rate'),
)


def validate_calibration_file(file_path):
    """Return missing/invalid record fields; an empty list means complete."""
    if not file_path:
        return ['calibration_file: 경로가 지정되지 않음']

    path = Path(file_path).expanduser()
    try:
        with path.open(encoding='utf-8') as calibration_file:
            document = json.load(calibration_file)
    except (OSError, json.JSONDecodeError) as exc:
        return [f'calibration_file: 읽을 수 없음 ({exc})']

    if not isinstance(document, dict):
        return ['calibration_file: 최상위 JSON 값은 object여야 함']

    errors = []
    if document.get('schema_version') != 1:
        errors.append('schema_version: 지원 버전 1이 아님')

    for field_path in _REQUIRED_VERIFIED_FLAGS:
        value = _read_path(document, field_path)
        if value is not True:
            errors.append(f'{".".join(field_path)}: true 검증 기록 필요')

    for field_path in _REQUIRED_TEXT_FIELDS:
        value = _read_path(document, field_path)
        if not isinstance(value, str) or not value.strip():
            errors.append(f'{".".join(field_path)}: 비어 있지 않은 값 필요')

    for field_path in _REQUIRED_POSITIVE_NUMBERS:
        value = _read_path(document, field_path)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
            errors.append(f'{".".join(field_path)}: 양수 값 필요')

    for group, expected_model in (
            ('leap', 'Dynamixel XC330-M288-T'),
            ('custom_fingers', 'Dynamixel XL330-M288-T'),
            ('custom_wrist', 'Feetech STS3215')):
        field_path = ('motors', group, 'model')
        if _read_path(document, field_path) != expected_model:
            errors.append(f'{".".join(field_path)}: {expected_model}이어야 함')
        ids = _read_path(document, ('motors', group, 'motor_ids'))
        if (not isinstance(ids, list) or not ids
                or any(isinstance(motor_id, bool) or not isinstance(motor_id, int)
                       or motor_id <= 0 for motor_id in ids)
                or len(set(ids)) != len(ids)):
            errors.append(f'motors.{group}.motor_ids: 중복 없는 양의 정수 목록 필요')
        if _read_path(document, ('motors', group, 'joint_mapping_verified')) is not True:
            errors.append(f'motors.{group}.joint_mapping_verified: true 검증 기록 필요')
        mapping = _read_path(document, ('motors', group, 'joint_mapping'))
        if not isinstance(mapping, list) or not mapping:
            errors.append(f'motors.{group}.joint_mapping: 관절 매핑 목록 필요')
        else:
            mapping_ids = []
            joint_names = []
            for index, entry in enumerate(mapping):
                prefix = f'motors.{group}.joint_mapping[{index}]'
                if not isinstance(entry, dict):
                    errors.append(f'{prefix}: object 필요')
                    continue
                joint_name = entry.get('joint_name')
                motor_id = entry.get('motor_id')
                direction = entry.get('direction')
                zero_offset = entry.get('zero_offset')
                unit = entry.get('unit')
                lower = entry.get('min_position')
                upper = entry.get('max_position')
                if not isinstance(joint_name, str) or not joint_name.strip():
                    errors.append(f'{prefix}.joint_name: 비어 있지 않은 값 필요')
                else:
                    joint_names.append(joint_name)
                if isinstance(motor_id, bool) or not isinstance(motor_id, int):
                    errors.append(f'{prefix}.motor_id: 정수 ID 필요')
                else:
                    mapping_ids.append(motor_id)
                if (isinstance(direction, bool) or not isinstance(direction, int)
                        or direction not in (-1, 1)):
                    errors.append(f'{prefix}.direction: -1 또는 1 필요')
                if not _is_number(zero_offset):
                    errors.append(f'{prefix}.zero_offset: 숫자 값 필요')
                if not isinstance(unit, str) or not unit.strip():
                    errors.append(f'{prefix}.unit: 단위 문자열 필요')
                if (not _is_number(lower) or not _is_number(upper)
                        or lower >= upper):
                    errors.append(f'{prefix}: min_position < max_position 필요')
            if len(set(joint_names)) != len(joint_names):
                errors.append(f'motors.{group}.joint_mapping: joint_name 중복')
            if len(set(mapping_ids)) != len(mapping_ids):
                errors.append(f'motors.{group}.joint_mapping: motor_id 중복')
            if (isinstance(ids, list) and all(isinstance(item, int) and not isinstance(item, bool)
                                               for item in ids)
                    and set(mapping_ids) != set(ids)):
                errors.append(f'motors.{group}.joint_mapping: motor_ids와 일치해야 함')

    for policy in ('leap', 'custom_hand'):
        if _read_path(document, ('policies', policy, 'observation_schema_verified')) is not True:
            errors.append(
                f'policies.{policy}.observation_schema_verified: true 검증 기록 필요')
        model_path = _read_path(document, ('policies', policy, 'model_path'))
        if isinstance(model_path, str) and model_path.strip():
            resolved_model_path = Path(model_path).expanduser()
            if not resolved_model_path.is_absolute():
                resolved_model_path = path.parent / resolved_model_path
            if not resolved_model_path.is_file():
                errors.append(f'policies.{policy}.model_path: 모델 파일을 찾을 수 없음')
            else:
                recorded_hash = _read_path(document, ('policies', policy, 'sha256'))
                if not isinstance(recorded_hash, str) or not re.fullmatch(
                        r'[0-9a-fA-F]{64}', recorded_hash):
                    errors.append(f'policies.{policy}.sha256: SHA-256 64자리 hex 필요')
                elif _sha256_file(resolved_model_path) != recorded_hash.lower():
                    errors.append(f'policies.{policy}.sha256: 모델 파일과 일치하지 않음')

    pose = document.get('pose', {})
    if isinstance(pose, dict) and pose.get('orientation_convention') != 'quaternion_xyzw':
        errors.append('pose.orientation_convention: quaternion_xyzw여야 함')
    if isinstance(pose, dict) and pose.get('position_unit') != 'm':
        errors.append('pose.position_unit: m여야 함')

    if _read_path(document, ('frames', 'transforms_verified')) is not True:
        errors.append('frames.transforms_verified: true 검증 기록 필요')

    return errors


def _is_number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_path(document, field_path):
    value = document
    for key in field_path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Validate a cube manipulation hardware calibration JSON file.')
    parser.add_argument('calibration_file', help='Path to the calibration JSON file')
    parsed = parser.parse_args(args=args)
    errors = validate_calibration_file(parsed.calibration_file)
    if errors:
        print('Calibration is incomplete:')
        for error in errors:
            print(f'- {error}')
        return 1
    print('Calibration record is complete. Hardware drivers still require implementation.')
    return 0
