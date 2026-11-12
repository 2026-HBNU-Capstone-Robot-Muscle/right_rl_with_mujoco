"""Record completed offline review; no hardware imports or settings changes."""
import json
from pathlib import Path
from leap_pose_pair_readonly import summarize_tip_direction
ROOT=Path(__file__).resolve().parent
stamp='## CURRENT 2026-10-18: 끝패드 관측 완료, 중지 ID 순서 가정 재검토'
note='''이번 07:20 읽기 전용 전·후 촬영은 완료됐다. 사진상 끝패드 굽힘 변화와 함께
ID9 +9.492°, ID11 +2.021°, ID12 −1.670°, ID10 +0.044°를 기록했다.
각 사진 전후16개 모터의 변화는 최대1틱이고 토크 OFF다. 기존 `ID12=끝관절`
가정으로 만든 자동 판정은 이번 자료를 설명하지 못한다. 사용자의 과거 ID 설명은
당시 판단 근거로 보존하되, ID와 해부학적 관절의 대응은 이번 차분 관측으로 재검토한다.

중지 ID 순서24가지를 네 저장 자세에 대입해 오프라인 MuJoCo 비교를 완료했다.
영점 오프셋 없이 `[12,10,11,9]`(뿌리→끝)가 기존 `[9,10,11,12]`보다 실제 중지
형태에 가까웠고, 기존 접촉 자세의 끝패드–엄지 하우징 모델 간격은61.42→13.30mm였다.
완전 역순 `[12,11,10,9]`는34.79mm다. 실제 접촉을 재현한 결과는 아니며 후보만
제시한다. ID10/11의 역할과 잔여 형상·등록 오차는 미확정이다. 다른 손가락까지
일괄 역순으로 바꾸지 않았고, 실물 제어 변환·게인·영점도 변경하지 않았다.

다음 사용자 작업: `leap_middle_joint_order_readonly.cmd` 한 창에서 전·후 두 번 SPACE.
첫 저장 뒤 모터 전원을 끄고, 중지 끝에서 두 번째 굽힘 관절을 작게 굽힌다.
끝패드와 바로 아래 모터 하우징을 함께 움직이고 둘 사이 각도는 유지한다.
전원 재연결 후 손을 떼고 두 번째 저장. 구동·쓰기 없는 별도 관절 식별 관측이다.
기존 끝패드 촬영을 다시 할 필요는 없다. 이번 도구의 실제 사용은 아직 미실행이다.

산출물: `middle_direction_review_20261018/README.md`, `analysis.json`,
`motor_order_comparison.jpg`, `overlay_0_1.png`. 원본 사진·엔코더·해시는 보존했다.
진단 요약에서 ID12를 끝관절로 미리 고정하던 부분을 제거했다. 읽기 전용 관련
테스트14개 통과, 새 모드 check-only 성공(장치 접근 없음).

실물 SAC 성공이나 전체 큐브 풀이 완료가 아니다. 현재 후보로 기존 SAC 경로를
실물에 보내지 않는다. 다음 관측 해석→매핑/역변환 검증→간섭을 반영한 경로 재검증이 필요하다.
아래 이전 기록의 ‘최신/다음’ 및 ID9=뿌리·ID12=끝관절 표기는 당시 가정과 이력이다.
현재 판단은 이 절을 우선한다.
'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
    if stamp not in old:p.write_text(stamp+'\n\n'+note+'\n---\n\n'+old,encoding='utf-8')
source=ROOT/'pose_pair_readonly_20261018_072023_698689/result.json'
r=json.loads(source.read_text(encoding='utf-8'))
new=summarize_tip_direction(r['direction_pairs'])
(ROOT/'middle_direction_review_20261018/reviewed_direction_check.json').write_text(json.dumps(dict(
    source=str(source),original_status=r['direction_check']['status'],
    reason_for_reassessment='Original summary assumed ID12 was the anatomical tip before visual comparison.',
    reviewed_summary=new,source_modified=False),indent=2),encoding='utf-8')
print(json.dumps(new,indent=2))
