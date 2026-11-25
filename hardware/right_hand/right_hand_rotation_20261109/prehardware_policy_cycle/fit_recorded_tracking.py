"""Empirical encoder lag fit from archived empty-hand samples, not torque ID."""
from pathlib import Path
import json
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def main():
    source=ROOT/'middle_clearance_probe_20261106_230902_998326/result.json'
    doc=json.loads(source.read_text(encoding='utf-8'));all_samples=doc['samples']
    end=next(i for i,s in enumerate(all_samples) if s['phase']=='place_cube')
    samples=all_samples[:end]
    phases=np.array([s['phase'] for s in samples]);times=np.array([s['t'] for s in samples],float)
    actual=np.asarray([s['actual'] for s in samples],float);goals=np.asarray([s['goal'] for s in samples],float)
    read=np.asarray([s['read_ms'] for s in samples],float)
    rows=[]
    for joint in range(16):
        span=float(np.ptp(goals[:,joint]));best=None
        if span<8 or np.ptp(actual[:,joint])<4:
            rows.append({'motor_id':joint+1,'identified':False,'reason':'insufficient_command_excitation','goal_span_ticks':span});continue
        for delay in (0,1,2):
            k=np.arange(delay,len(samples)-1)
            mask=(phases[k]==phases[k+1])&(phases[k]==phases[k-delay])&((times[k+1]-times[k])>.03)&((times[k+1]-times[k])<.15)
            k=k[mask]
            if len(k)<50:continue
            split=int(len(k)*.75);train=k[:split];test=k[split:]
            x=actual[train,joint];u=goals[train-delay,joint];y=actual[train+1,joint]
            for a in np.linspace(0,.99,200):
                b=float(np.clip(np.mean(y-a*x-(1-a)*u)/max(1-a,1e-6),-64,64))
                pred=a*x+(1-a)*(u+b);mse=float(np.mean((y-pred)**2))
                if best is None or mse<best['train_mse']:
                    pred_test=a*actual[test,joint]+(1-a)*(goals[test-delay,joint]+b)
                    best={'motor_id':joint+1,'identified':True,'goal_span_ticks':span,'sample_delay':delay,
                          'a_per_sample':float(a),'offset_ticks':b,'train_mse':mse,
                          'heldout_rmse_ticks':float(np.sqrt(np.mean((actual[test+1,joint]-pred_test)**2))),
                          'heldout_static_goal_rmse_ticks':float(np.sqrt(np.mean((actual[test+1,joint]-goals[test-delay,joint])**2))),
                          'training_pairs':len(train),'heldout_pairs':len(test)}
        rows.append(best if best is not None else {'motor_id':joint+1,'identified':False,'reason':'not_enough_same_phase_pairs'})
    report={'status':'archived_empty_hand_empirical_tracking_fit','source':str(source.relative_to(ROOT)),
            'empty_hand_sample_count':len(samples),'cut_before_first_cube_placement_record':True,
            'phase_names':list(dict.fromkeys(phases)),'median_read_ms':float(np.median(read)),
            'samples_have_shared_read_timestamp_not_exact_per_motor_measurement_time':True,
            'training_first75percent_and_heldout_last25percent_per_delay':True,'fits':rows,
            'absolute_torque_force_cap_and_cube_friction_identified':False,
            'reason':'Encoder/goal lag alone does not separate torque gain, friction, inertia and unmeasured contact loads.',
            'existing_PID_force_limits_and_physical_settings_changed':False,
            'devices_opened':0,'hardware_execution_allowed':False}
    (HERE/'recorded_tracking_fit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'status':report['status'],'samples':len(samples),'median_read_ms':report['median_read_ms'],
                      'fits':[{'id':r['motor_id'],'identified':r['identified'],'rmse':r.get('heldout_rmse_ticks')} for r in rows],
                      'torque_calibrated':False}),flush=True)


if __name__=='__main__':main()
