통신 검사 v2다. 모든 모터의 토크가 이미 OFF인 상태에서 표시등 LED만 켰다 끄고 새 읽기로 전달을 확인한다. 손가락 목표와 토크, 전류, PID, 모드 및 EEPROM은 쓰지 않는다. 끝나면 각 모터의 원래 LED 값을 복원한다.

```powershell
& "C:\Users\김준서\Documents\Codex\2026-09-23\files-mentioned-by-the-user-leap\outputs\leap_bus_echo_off.cmd"
```

큐브를 빼고 다른 모터 및 카메라 프로그램을 닫는다. USB와 모터 전원을 연결한 뒤 터미널에서 SPACE를 한 번 누른다. 손가락 모양을 바꿀 필요는 없다. 처음 15초는 카메라를 닫고 검사하며, 다음 15초는 웹캠을 자동으로 켠다. 카메라 시작 시간이 추가될 수 있다. 표시등이 깜박이는 것은 이번 검사의 정상 동작이다. 마커나 큐브를 보여주거나 추가 SPACE를 누를 필요는 없다. Q와 Esc 및 Ctrl+C로 중단한다.

이전 진단은 토크 OFF에서 Goal Position 값이 고정돼 있을 것으로 가정했다. 실제 저장된 두 실행의 14개 읽기에서 Goal Position은 매번 Present Position과 같았고 1틱씩 달라졌다. 두 번째 실행은 시험 패킷을 보내기도 전에 중단됐다. 이 자료만으로 통신 고장이나 펌웨어의 원인을 확정하지 않는다. 현재 진단은 Goal Position을 관찰만 하고 비교나 쓰기에 사용하지 않는다. 주소 분리를 검사한 SDK 모의 패킷에서도 Goal Position116과 Present Position132가 별도로 읽힘을 확인했다.

공식 제어표에 따라 LED RAM 주소65의 1바이트에만 0 또는 1을 기록한다. 매 쓰기 전에 토크 OFF와 watchdog 및 오류 없음, 전압과 온도를 새로 확인한다. 예상 LED 값이 읽히지 않으면 최대150ms 및 추가3회 읽기로 확인한다. 시험 패킷 재전송으로 오류를 덮지 않는다. 복원 시에도 새 OFF 읽기가 확인된 경우에만 원래 LED 값을 한 번 전송한다. 복원 실패는 결과에 기록한다.

- 공식 LED 설명: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#led65
- 결과: `bus_echo_off_날짜_시간/result.json`
- 최신 결과 복사: `leap_bus_echo_off_result.json`

이 검사는 모터당 1바이트 LED 쓰기와 기존83바이트 전체 상태 읽기로 카메라 사용 전후의 통신을 비교한다. 부하가 걸린 상태의 4바이트 Goal Position 전달을 검증하는 것은 아니다. 기존 회전 프로그램의 목표 확인 및 중단 기준은 그대로 유지한다. 끝나면 했어만 알려주면 저장 파일을 검토한다.
