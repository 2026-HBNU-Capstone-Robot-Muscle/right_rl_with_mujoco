# 현재 가로·세로 회전과 G 복귀

작업 폴더는 `hardware/right_hand`다. 가로·세로 모두 마지막 받침/파지 단계에서 **검지 끝까지 → 검지 유지 → 중지 이동** 순서로 진행한다. 엄지를 먼저 내리는 순서와 같은 확정 G를 유지한다. 가로에는 중지 뿌리 약4도 낮춤과 끝마디 약5.6도 추가 굽힘이 포함돼 있다.

| 용도 | 새 실행파일 |
|---|---|
| 가로 회전 → G 복귀/유지 | `run_current_horizontal.cmd` |
| 세로 회전 → G 복귀/유지 | `run_current_vertical.cmd` |
| A 가로 / B 세로를 같은 토크 세션에서 선택 | `run_current_keyboard.cmd` |

다른 로봇 창을 정상 종료하고 하나만 연다. 처음 SPACE로 G를 준비한다. A/B 파일은 G 확인 안내 뒤 A/B를 받으며, 이동 중 누른 A/B는 예약하지 않는다. SPACE/Q/Esc는 중단/OFF다. 중간 확인 SPACE나500초 전체 제한은 없다. 종료 전 큐브를 받친다.

Windows Python 환경이 없으면 이 폴더에서 `.venv`를 만들고 `requirements-leap.txt`를 설치한다. 설치 환경이나 인증정보를 저장소에 포함하지 않았다. 새cmd는 기존 `.runtime`, `.venv`, PATH의 Python 순서로 사용할 수 있다. 역사적 import 두 파일은 `current_compat`에 해시를 고정해, 원래 PC의 `work/Leap_Test/deploy`가 없어도 불러온다. 실제 모터 출력은 원래의 검사된 PoseBus다.

장치를 열지 않는 확인:

```powershell
python -X utf8 -B current_motion_entry_20261125.py --mode horizontal
python -X utf8 -B current_motion_entry_20261125.py --mode vertical
python -X utf8 -B current_motion_entry_20261125.py --mode keyboard
```

단독 실행은 지금 사용한 별도 속도 실험을 보존한다.75ms,시작/가로 마지막 받침2틱,선택 고속 구간 최대6틱이다. 미들웨어용 `right_hand_rotation_20261125/middleware_current_rotation`은 같은 관절 자세/순서를 기존2틱·75ms로 내보낸 것이다. 미들웨어에6틱 속도를 자동 적용하지 않는다.450mA와 기존 PID/관절/전압/전류/추종·고정관절 보호는 유지한다.

통신 실패나 완전한 읽기의200ms 초과는 마지막 목표를 유지한 채 최대10초 READ만 재시도한다. 늦은 표본 자체로 움직이지 않고, 새 전체16개 정상 표본2개를200ms 안에 확인한 뒤 다음 목표부터 이어간다. 전압 확인 시간 초과·실제 건강/고정관절 이상·토크/워치독 변경은 이 처리로 넘기지 않는다. 자동 ON·워치독 해제·목표 재전송은 없다.

최근 실제 세로 실행은 복귀 도중 읽기203ms로 중단했고 OFF·설정 복구를 확인했다. 이 지연 처리 누락을 수정한 뒤 소프트웨어 검사는 통과했지만, 수정판의 실제 복구 및 ROS/DDS·양손 구동 성공은 아직 확인하지 않았다. 완료 표시는 관절 경로와 G 상태 판정이며 실제 큐브 회전각·접촉/미끄러짐 측정과 구분한다.

원본267개와 과거1배속 파일은 보존했다. 추가 파일/해시는 저장소 루트 `RIGHT_HAND_CURRENT_ROTATIONS_20261125_MANIFEST.json`, 후보와 검사 결과는 각 실행 자료 디렉터리의 manifest/validation 파일에서 확인한다. 원본 파일을 수정하면 검증이 거부된다.
