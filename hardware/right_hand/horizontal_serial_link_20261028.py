"""Use the user's verified link speed; no motor-register writes."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
CONFIG=ROOT/'hand_serial_link_20261028.json'

def metadata():
    c=json.loads(CONFIG.read_bytes())
    expected=dict(status='all_16_verified',baud=1000000,baud_register_value=3,port='COM7',
        motor_ids=list(range(1,17)),torque_off_verified=True)
    if any(c.get(k)!=v for k,v in expected.items()):raise RuntimeError('Verified1Mbps host-link record missing or changed')
    sources={}
    for name in ('evidence','readonly_check'):
        path=(ROOT/c[name]).resolve();path.relative_to(ROOT)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=c[name+'_sha256']:raise RuntimeError('Host-link evidence changed')
        sources[name]=json.loads(path.read_bytes())
    fresh=sources['evidence'];test=sources['readonly_check'];blocks=test.get('blocks',[])
    if fresh.get('status')!='all16_1m_off_reads_saved' or fresh.get('baud')!=1000000 or fresh.get('motor_writes')!=0 or fresh.get('differences')!=[] or fresh.get('maximum_drift_ticks',999)>2:raise RuntimeError('Fresh1Mbps OFF evidence incomplete')
    if set(fresh.get('individual_states',{}))!={str(m) for m in range(1,17)}:raise RuntimeError('Missing1Mbps motor IDs')
    for states in fresh['individual_states'].values():
        if len(states)!=2 or any(s.get('baud_value')!=3 or any(s.get(str(a))!=0 for a in (64,70,98)) for s in states):raise RuntimeError('Motor link/OFF state not confirmed twice')
    if test.get('status')!='readonly_burst_comparison_complete' or test.get('baud')!=1000000 or test.get('motor_writes')!=0 or test.get('camera_frames',0)<=0 or len(blocks)!=1 or blocks[0].get('group_size')!=1 or blocks[0].get('camera_on') is not True or blocks[0].get('complete')!=1000 or blocks[0].get('errors')!=[] or blocks[0].get('maximum_ms',999)>200:raise RuntimeError('Camera-concurrent1Mbps single-motor READ comparison incomplete')
    return dict(version='verified_user_commissioned_host_link_v1_20261028',port='COM7',baud=1000000,
        expected_baud_register=3,config_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        motor_ids=list(range(1,17)),evidence=c['evidence'],evidence_sha256=c['evidence_sha256'],
        readonly_check=c['readonly_check'],readonly_check_sha256=c['readonly_check_sha256'],
        full_health_reply_group_size=1,link_stability_proven_permanent=False,
        motor_baud_writes_by_motion_program=0,automatic_baud_probe_or_change=False,
        all_existing_motor_health_limits_preserved=True)

def prepare_host(bus):
    policy=metadata()
    if bus.port.getBaudRate()!=policy['baud']:
        if not bus.port.setBaudRate(policy['baud']):raise RuntimeError('Cannot select verified1Mbps host speed; no motor writes')
    bus.report['serial_link_policy']=policy
    return policy
