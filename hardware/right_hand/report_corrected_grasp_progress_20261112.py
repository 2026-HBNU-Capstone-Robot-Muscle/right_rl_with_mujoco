"""Publish concise evidence and continuation requirements without changing a runner."""
from pathlib import Path
import argparse
import json
import hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--publish',action='store_true');args=parser.parse_args()
    sources=['corrected_three_finger_sequence/review.json','reached_three_finger_geometry/review.json',
             'reached_lower_contact_search/review.json','corrected_three_finger_continuous_mu0.3/review.json',
             'corrected_three_finger_continuous_mu0.2/review.json']
    for name in sources:
        path=BASE/name
        if not path.exists():continue
        r=json.loads(path.read_bytes())
        if 'continuous' in name:
            print(json.dumps(dict(path=name,**{k:r[k] for k in ('done','total','stop','continuous_three_finger_supported_hold_passed','loaded_fractions','hold_shift_mm','hold_drift_deg','upper_certificates','original_face5_tilt_deg','final_top')})),flush=True)
        elif name=='reached_lower_contact_search/review.json':
            print(json.dumps(dict(path=name,geometry=[{k:x[k] for k in ('finger','static_lower_contact','gap_mm')} for x in r['rows']],dynamic_trials=[{k:x[k] for k in ('order','stop','temporary_three_finger_hold_passed','loaded_fractions','hold_shift_mm')} for x in r['dynamic_trials']],source_top_face=r['source_top_face'])),flush=True)
    if not args.publish:return
    nominal=json.loads((BASE/'corrected_three_finger_continuous_mu0.3/review.json').read_bytes())
    friction=json.loads((BASE/'corrected_three_finger_continuous_mu0.2/review.json').read_bytes())
    lower=json.loads((BASE/'lower_three_finger_contact_search/review.json').read_bytes())
    refinement=json.loads((BASE/'lower_three_finger_refinement/review.json').read_bytes())
    assert nominal['continuous_three_finger_supported_hold_passed'] and not nominal['stop']
    assert not friction['continuous_three_finger_supported_hold_passed']
    assert nominal['upper_certificates']['nearest_up_face']['first_failure'] is not None
    assert not any(x['temporary_three_finger_hold_passed'] for x in lower['dynamic_trials'])
    assert not refinement['three_finger_lower_clear_hold_passed']
    folder=BASE/'corrected_three_finger_summary';folder.mkdir(exist_ok=True)
    paths=[ROOT/'probe_corrected_three_finger_sequence_20261112.py',ROOT/'fit_reached_three_finger_contacts_20261112.py',
           ROOT/'search_reached_lower_contacts_20261112.py',ROOT/'verify_corrected_three_finger_chain_20261112.py',
           ROOT/'probe_three_finger_top_alignment_20261112.py',ROOT/'refine_lower_three_finger_hold_20261112.py']
    paths += [BASE/x for x in sources]
    paths += [BASE/'lower_three_finger_contact_search/review.json',BASE/'lower_three_finger_refinement/review.json']
    bindings={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    candidate=BASE/'corrected_three_finger_continuous_mu0.3/candidate.npz';z=dict(np.load(candidate,allow_pickle=False))
    assert np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2
    assert hashlib.sha256(candidate.read_bytes()).hexdigest()==nominal['candidate_sha256']
    for p in paths[:6]:(folder/p.name).write_bytes(p.read_bytes())
    report=dict(status='nominal_three_finger_connection_achieved_upper_clearance_and_robustness_pending',
                nominal_continuous_goals=nominal['done'],nominal_supported_hold_samples=nominal['supported_samples'],
                nominal_three_finger_loaded_fractions=nominal['loaded_fractions'],
                nominal_hold_shift_mm=nominal['hold_shift_mm'],nominal_hold_drift_deg=nominal['hold_drift_deg'],
                upper_clearance_passed=False,low_friction_replay_stop=friction['stop'],low_friction_stop_goal=friction['done'],
                lower_regrasp_lost_middle_contact=True,MF_feedback_stop=refinement['stop'],
                MF_feedback_self_pairs_mm=refinement['terminal_guard']['self_pairs_mm'],
                cube_top_changed_from_green5_to_orange3_in_new_nominal_branch=True,
                desired_cube_orientation_and_final_left_alignment_certified=False,
                next_rotation_included=False,full_physical_trial_ready=False,physical_success=False,
                source_bindings=bindings,candidate_sha256=nominal['candidate_sha256'],
                previous_hardware_executable_modified=False,devices_opened=0,new_SAC_training_steps=0,
                native_joint_collision_tracking_and_drive_limits_unchanged=True)
    (folder/'review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    text='''# 2026-11-12 수정 경로 검토\n\n'''
    text+='''현재 사용자가 추가로 연결·촬영·재실행할 일은 없다. 기존 실물 실행파일은 수정하지 않았다.\n\n'''
    text+=f'''중지 윗마디를 먼저 편 뒤 엄지 이송과 하부 받침 이동을 거쳐, 현재 큐브에 맞춰 구한 약지 → 엄지 → 중지 접촉을 연결했다. 명목 마찰 0.3 모델에서는 처음 배치부터 상태 초기화 없이 {nominal['done']}목표를 재생했고, 마지막 4050개 1ms 표본 모두에서 지지와 세 손가락 접촉이 성립했다. 유지 중 최대 이동 {nominal['hold_shift_mm']:.3f}mm, 방향 변화 {nominal['hold_drift_deg']:.3f}도다. 실물 성공 수치가 아니다.\n\n'''
    text+='''전체 준비 완료로 판정할 수 없는 근거:\n\n- 세 손가락 접촉은 성립했지만 현재 윗면의 회전 공간 검사에 실패했다. 분리 평면 하한의 음수를 실제 침투 깊이로 해석하지 않는다.\n- 마찰 0.2 가정에서는 3097번째 목표 중 추종 기준으로 중단했다. 실제 마찰 측정값이 아니다.\n- 접촉을 더 낮춘 두 경로는 지지는 유지했지만 중지 접촉을 잃었다. 이후 중지 압력 보정은 291번째 목표에서 손가락 간섭 기준으로 멈췄다. 기준을 낮춰 통과시키지 않았다.\n- 새 경로에서 윗면이 기존 초록색(5)에서 주황색(3)으로 바뀌었다. 이는 원하는 방향으로 회전했다는 인증이 아니며, 다음 동작의 큐브 자세·작업면을 일관되게 연결해야 한다.\n- 다음 전체 회전은 아직 포함하지 않았다.\n\n다음 수정은 약지와 엄지의 지지를 유지하면서 중지가 아래쪽에 먼저 닿도록 접근 경로를 바꾸고, 엄지 끝마디를 윗단 공간 밖으로 낮추는 것이다. 같은 최종 후보에 대해 마찰 차이와 연속 재생을 다시 검사한 후 실물용 파일을 갱신한다.\n\n모든 작업은 저장 모델에서 수행했다. 실제 모터·카메라 접근, SAC 추가 학습, 정책 증류, 원격 반영은 수행하지 않았다.\n'''
    text=text.replace('이후 중지 압력 보정은 291번째 목표에서 손가락 간섭 기준으로 멈췄다.',
        '이후 중지 압력 보정은 291번째 목표에서 중지 끝마디와 약지 중간마디의 모델 접촉 약 0.000129mm로 중단했다. '
        '매우 작은 접촉이므로 실물의 유의미한 충돌로 확정하지 않는다. 중지가 큐브에 닿지 않은 점이 핵심 미달이며, 접촉 기준을 낮춰도 이 문제가 해결되지는 않는다.')
    (folder/'REVIEW.md').write_text(text,encoding='utf-8')
    target_path=BASE/'FINAL_TARGET.json';target=json.loads(target_path.read_bytes())
    target['latest_model_connection_review']='corrected_three_finger_summary/REVIEW.md'
    target['nominal_continuous_three_finger_hold_passed']=True
    target['upper_clearance_and_low_friction_robustness_passed']=False
    target['complete_three_finger_join_ready']=False
    target['next_rotation_included']=False
    target_path.write_text(json.dumps(target,indent=2,ensure_ascii=False),encoding='utf-8')
    marker='corrected_TH_MF_RF_continuous_evidence_20261112'
    note=f'''\n<!-- {marker} -->\n## 최신: 수정 순서의 세 손가락 연결은 명목 모델 통과, 전체 실물 준비는 미완료\n\n현재 사용자의 추가 연결·촬영·기존 파일 재실행은 필요 없다. 보조자가 중지 윗마디를 먼저 편 순서에서 마지막 엄지·중지·약지 파지 연결을 진행 중이다. 마찰 0.3 명목 모델에서 처음 배치부터 3788목표를 초기화 없이 재현했고 마지막 4050표본 모두 지지 및 세 손가락 접촉을 확인했다. 유지 이동 약 0.033mm/방향 변화 0.142도는 모델 수치다.\n\n현재 윗단 공간은 미통과이며, 마찰 0.2에서는 목표3097 추종 중단이다. 접촉을 낮춘 후속 경로는 중지 접촉을 잃었고 중지 보정은 목표291에서 손가락 간섭 중단이다. 새 명목 경로의 윗면은 초록5에서 주황3으로 바뀌므로 원하는 큐브 방향 및 다음 동작의 작업면도 아직 인증되지 않았다. 다음 회전은 미포함, 새 실물 실행파일 준비 완료가 아니다. 이전 중간 받침 실행 완료와 최종 세 손가락 파지 완료를 구분한다.\n\n근거: `right_hand_rotation_20261112/support_first_transfer/corrected_three_finger_summary/REVIEW.md`, `review.json`. 다음 수정은 중지가 하부에 먼저 닿는 접근과 엄지 윗단 공간 해소를 함께 만족시키는 것이다. 기존 실물 실행파일·모터 제한·중단 기준은 수정하지 않았고, 장치 접근 및 새 SAC 학습은 없었다.\n<!-- /{marker} -->\n'''
    note=note.replace('중지 보정은 목표291에서 손가락 간섭 중단이다.',
        '중지 보정은 목표291에서 모델 손가락 접촉 약0.000129mm로 중단이다. 이 미세 접촉을 실물 충돌로 확정하지 않는다. 중지가 큐브에 닿지 않은 것이 핵심 미달이다.')
    for path in (ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md'):
        doc=path.read_text(encoding='utf-8-sig')
        if marker not in doc:
            first,sep,rest=doc.partition('\n');path.write_text(first+'\n'+note+'\n'+rest,encoding='utf-8')
        else:
            a=doc.index('<!-- '+marker+' -->');b=doc.index('<!-- /'+marker+' -->',a)+len('<!-- /'+marker+' -->')
            path.write_text(doc[:a]+note.strip()+doc[b:],encoding='utf-8')
    print(json.dumps(dict(published=folder.relative_to(ROOT).as_posix(),MF_feedback_self_pairs_mm=report['MF_feedback_self_pairs_mm'],full_physical_trial_ready=False)),flush=True)

if __name__=='__main__':main()
