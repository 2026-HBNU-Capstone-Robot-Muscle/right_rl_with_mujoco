"""Plot saved fresh initial-placement trials; these are simulated measurements."""
from pathlib import Path
import json
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent


def main():
    old=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/review.json'
    parent=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/review.json'
    paths=[old,parent]+sorted(OUT.glob('next_cycle/staged_model_*ccd50repair*/review.json'))
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True,constrained_layout=True)
    sources=[]
    for path in paths:
        r=json.loads(path.read_text());t=r['trace']
        if path==old:label='Historical CCD10 - first turn/hold pass';style='--'
        elif path==parent:label='CCD50 - parent policy';style='-'
        else:
            actor=Path(r['rotation_repair_actor_path'])
            kind='early-window repair' if 'early_recovery' in str(actor) else ('long-window repair' if 'recovery_long' in str(actor) else 'short-window repair')
            label='CCD50 - '+kind+' '+actor.parent.name.removeprefix('export_').lstrip('0')
            style='-'
        x=np.array([s['step']*.075 for s in t])
        for ax,key in zip(axes,('error_deg','center_error_mm','tilt_deg')):
            ax.plot(x,[s[key] for s in t],style,label=label,linewidth=1.6)
        sources.append(dict(path=path.relative_to(OUT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            label=label,model_kind=r.get('model_kind','active'),completed_goals=r['steps'],first_rotation_and_hold_passed=r['first_rotation_and_hold_passed']))
    for ax,ylabel in zip(axes,('90deg orientation error (deg)','Common-station error (mm)','Up tilt (deg)')):
        ax.set_ylabel(ylabel);ax.grid(alpha=.25)
    axes[0].axhline(5,color='black',linewidth=.7,alpha=.5)
    axes[1].axhline(2,color='black',linewidth=.7,alpha=.5)
    axes[2].axhline(5,color='black',linewidth=.7,alpha=.5)
    axes[0].legend(fontsize=8,loc='upper left');axes[-1].set_xlabel('Simulated elapsed control time (seconds)')
    fig.suptitle('Fresh initial-placement policy comparisons\nHistorical CCD10 pass does not certify CCD50; horizontal limits apply to final hold')
    folder=OUT/'review';fig.savefig(folder/'ccd_policy_comparison.png',dpi=160,bbox_inches='tight',pad_inches=.15);plt.close(fig)
    (folder/'ccd_policy_comparison_sources.json').write_text(json.dumps(dict(
        scientific_plot_of_saved_model_trials=True,sources=sources,
        plotted_samples_are_sparse_control_step_reports_not_every_1ms_state=True,
        devices_opened=0,hardware_execution_allowed=False),indent=2),encoding='utf-8')
    print(json.dumps(dict(model_trials=len(sources),image='review/ccd_policy_comparison.png')))


if __name__=='__main__':main()
