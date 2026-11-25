"""Review saved voltage telemetry and pause the next motion recommendation."""
import collections,datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'recovered_forward_placement_20261026_041024_485668'
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/undervoltage_0410'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_bytes())
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    OUT.mkdir(exist_ok=True);r=read(SOURCE/'result.json');s=r['samples'];last=s[-1]
    voltage=np.asarray([v['voltage_v'] for v in s]);groups=[]
    for name,lo,hi in (('thumb',0,4),('index',4,8),('middle',8,12),('ring',12,16)):
        groups.append(dict(finger=name,motor_ids=list(range(lo+1,hi+1)),final_min_v=float(voltage[-1,lo:hi].min()),
            final_max_v=float(voltage[-1,lo:hi].max()),whole_run_min_v=float(voltage[:,lo:hi].min())))
    low=np.flatnonzero((voltage<3.7).any(axis=1));prior=np.flatnonzero((voltage[: -1]<3.7).any(axis=1))
    phases=dict(collections.Counter(v['phase'] for v in s))
    events={k:v for k,v in r.items() if 'voltage' in k and k not in ('samples',)}
    review=dict(source=SOURCE.name,source_sha256=sha(SOURCE/'result.json'),error=r['error'],failed_phase=r['failed_phase'],
        phases=phases,last_goal_ticks=r['last_issued_command'],phase_sample_count=phases.get('stage_6_loaded_quarter_reference',0),
        sample_count=len(s),motor_write_events=len(r['write_events']),
        voltage_before_fault_v=s[-2]['voltage_v'],voltage_at_fault_v=last['voltage_v'],groups=groups,
        sub_3_7_v_sample_count=int(len(low)),sub_3_7_v_samples_before_fault=int(len(prior)),
        first_sub_3_7_sample_index=int(low[0]) if len(low) else None,
        last_healthy_to_fault_read_interval_s=last['t']-s[-2]['t'],
        low_voltage_duration_determined=False,duration_reason='Immediate OFF after first3.5V read censors the subsequent loaded-voltage duration.',
        last_inter_motor_voltage_spread_v=float(voltage[-1].max()-voltage[-1].min()),
        max_tracking_rad=max(max(v['tracking_rad']) for v in s),
        peak_motor_current_ma=int(max(np.max(abs(np.asarray(v['current_ma']))) for v in s)),
        motor_current_telemetry_is_not_supply_current=True,
        thumb_release_confirmed=r.get('thumb_mcp_release_measured_verified'),thumb_release_actual=r.get('thumb_mcp_release_actual_ticks'),thumb_release_threshold=r.get('thumb_mcp_release_limit_ticks'),
        cube_placement_confirmed=r.get('cube_placement_confirmed'),cube_removal_confirmed=r.get('cube_removal_confirmed'),cube_present=r.get('cube_present'),
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        physical_cube_rotation_verified=False,camera_recording=r['camera_recording'],voltage_guard_events=events,
        operator_confirmed_power_topology='One3pin cable from power hub supplies all16 motors',
        inference='Voltage loss along the single feed/chain or its connections is likely; supply/connection measurements are still needed to localize it.',
        wiring_fault_localized=False,undervoltage_trip_threshold_changed=False,
        official_motor_voltage_range_v=[3.7,6.0],official_recommended_voltage_v=5.,
        sources=[dict(title='ROBOTIS XC330-M288 specifications',url='https://docs.robotis.com/docs/dxl/model_reference/x_series/xc_series/xc330-m288/'),
            dict(title='ROBOTIS U2D2 Power Hub',url='https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/'),
            dict(title='LEAP Hand assembly electronics',url='https://v1.leaphand.com/assembly')],
        next_action='Remove Cube if still present, turn motor power OFF, then show the power-hub3pin ports and cable to hand. Identify a split-feed arrangement from the actual connections before rewiring. No repeated loaded trial yet.')
    write(OUT/'review.json',review)
    rows=['t_s,phase,'+','.join('id'+str(i)+'_v' for i in range(1,17))]
    rows.extend(str(v['t'])+','+v['phase']+','+','.join(map(str,v['voltage_v'])) for v in s)
    (OUT/'voltage_trace.csv').write_text('\n'.join(rows)+'\n',encoding='utf-8')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;v=read(p);v.update(timestamp=stamp,latest_loaded_undervoltage_review=review,
            next_user_command=None,next_user_command_ready=False,next_action=review['next_action'],
            power_feed_review_required_before_next_loaded_trial=True,physical_ninety_degree_rotation_verified=False)
        write(p,v)
    block='''## CURRENT 2026-10-26: 큐브 적재 회전 중 3.5V 관측 / 전원 경로 확인 필요

- `recovered_forward_placement_20261026_041024_485668`는 수정된 엄지 분리 확인과 빈손 준비, 큐브 배치를 통과하고 적재 기준 회전 중 ID1과 ID2의 3.5V 읽기로 즉시 OFF했다. 같은 읽기에서 엄지 3.5~3.6V, 검지 3.7~3.9V, 중지 4.0~4.3V, 약지 4.5~4.8V였다. 손 내부 전압 차이는 최대 1.3V다. 최대 추종 오차 0.11505rad, 최고 모터 전류 132mA였고 카메라 1,572장은 모두 정상이었다. OFF와 설정 복원을 확인했다. 제거 확인 단계 전에 멈춰 큐브 제거는 아직 프로그램으로 확인되지 않았다.
- 첫 3.5V 읽기 직후 OFF했으므로 부하가 유지됐을 때의 저전압 지속 시간은 알 수 없다. 단발 노이즈였다고 확정하거나 저전압 중단 기준을 낮추지 않았다. 모터 전류 합을 전원장치 입력 전류로 취급하지 않는다. 공식 XC330-M288 입력 범위는 3.7~6.0V이며 권장은 5.0V다.
- 사용자는 전원 허브에서 손 전체로 3핀 케이블 한 개가 연결된다고 확인했다. 전압의 위치 차이를 보면 그 한 공급 경로나 직렬 연결 및 커넥터의 손실이 유력하지만, 고장 위치나 전원장치 출력을 아직 측정해 확정하지 않았다. 같은 5V 공급을 적절한 분기로 나누는 구성을 검토한다. 실제 허브 포트와 손 연결을 보기 전 선을 옮기도록 지시하지 않는다. 공식 LEAP 조립 자료도 손가락별 커스텀 배선 또는 전원 허브 사용을 설명한다.
- 다음 사용자 작업은 큐브가 남아 있으면 제거하고 모터 전원을 끈 뒤, 파란 전원 허브의 3핀 단자들과 손으로 이어지는 케이블이 보이게 사진을 제공하는 것이다. 현재는 적재 실행 명령을 다시 권하지 않는다. 실제 배선 확인 뒤 분기하거나 짧게 할 선을 지정한다. 추가 모터 목표와 토크 쓰기는 하지 않았고 중단 기준과 원본 및 전시 문구는 바꾸지 않았다.
- 근거는 `loaded_finish_review_0057_20261026/undervoltage_0410/review.json`과 `voltage_trace.csv`다. 실물 90도 회전은 미검증이다.

<!-- END LOADED UNDERVOLTAGE CURRENT 20261026 -->

'''
    marker='<!-- END LOADED UNDERVOLTAGE CURRENT 20261026 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-26: 엄지의 20틱 목표 오차 중단을 검증된 분리 위치 확인으로 교체','## PREVIOUS 2026-10-26: 엄지의 20틱 목표 오차 중단을 검증된 분리 위치 확인으로 교체',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps({k:review[k] for k in ('error','phase_sample_count','groups','sub_3_7_v_sample_count','sub_3_7_v_samples_before_fault','last_healthy_to_fault_read_interval_s','thumb_release_confirmed','cube_present')},ensure_ascii=False))
if __name__=='__main__':main()
