"""Export saved model rotation/release and final-grip comparison figures."""
from pathlib import Path
import json
import hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties,fontManager

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'reposition_then_second_rotation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    font=next((p for p in (Path('C:/Windows/Fonts/malgun.ttf'),Path('/mnt/c/Windows/Fonts/malgun.ttf')) if p.exists()),None)
    if font is not None:fontManager.addfont(str(font));plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus']=False
    sources=[];fig,axes=plt.subplots(1,2,figsize=(13,4.7),layout='constrained')
    for name,label,color in (('robust_mu025_late_TH_feedback_v34','이전 이탈 제어', '#bf4f45'),('robust_mu025_late_TH_normal_release_v35','실제 면의 바깥으로 이탈','#157f83')):
        path=BASE/name/'review.json';r=json.loads(path.read_text(encoding='utf-8'));row=r['rows'][0];trace=row['trace'];sources.append(path)
        axes[0].plot([t['step']*.075 for t in trace],[t['error180_deg'] for t in trace],label=label,color=color)
        if name.endswith('_v35'):
            hold=next(t for t in trace if t['phase']=='hold');axes[0].axvspan(hold['step']*.075,trace[-1]['step']*.075,color=color,alpha=.15,label='4.05초 지지 유지 (윗단 공간 미달)')
    axes[0].axhline(5,color='#666',ls='--',lw=1,label='최종 방향 기준5°');axes[0].set(xlabel='해당 엄지 구간 시작 후 시간 [s]',ylabel='전체180° 목표와의 방향 오차 [°]',ylim=(0,40),title='회전 뒤 이탈 방향을 바꾼 결과');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    path=BASE/'robust_mu025_final_clear_hold_v37/review.json';r=json.loads(path.read_text(encoding='utf-8'));sources.append(path)
    rows=[row for row in r['rows'] if row['label'] in ('unchanged','MF_clear','known_clear_hand')];labels={'unchanged':'그대로 유지','MF_clear':'중지 비우기','known_clear_hand':'기존 전체 파지'}
    axes[1].bar([labels[row['label']] for row in rows],[row['upper_minimum_lower_mm'] for row in rows],color=['#bf4f45','#d8aa4b','#157f83']);axes[1].axhline(.3,color='#666',ls='--',lw=1)
    final_path=BASE/'robust_mu025_final_pose_space_v38/review.json';final=json.loads(final_path.read_text(encoding='utf-8'));sources.append(final_path)
    axes[1].bar(['위치·방향 보정'],[final['final_hold']['min_lower_mm']],color='#39764d');axes[1].text(3,1.1,f"방향 오차{final['final_hold']['final_error_deg']:.1f}°",ha='center',fontsize=9)
    for i,row in enumerate(rows):axes[1].text(i,max(.9,row['upper_minimum_lower_mm']+.2),f"방향 오차{row['final180_error_deg']:.1f}°",ha='center',fontsize=9)
    axes[1].set(ylabel='윗단 분리 증명에서 확인한 하한 [mm]',title='공간을 비우는 동안 방향이 달라짐',ylim=(-7.8,2.4));axes[1].grid(axis='y',alpha=.2)
    fig.suptitle('낮은 마찰0.25 모델의 저장 결과 · 실물 성공 아님',fontsize=14)
    fig.savefig(FOLDER/'late_rotation_progress.png',dpi=160);fig.savefig(FOLDER/'late_rotation_progress.svg');plt.close(fig)
    meta=dict(status='saved_model_scientific_plot',sources_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},negative_certificate_lower_not_a_penetration_measurement=True,conditional_final_hold_passed=final['conditional_bridge_and_hold_passed'],full_initial_placement_replay_reported_separately=True,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'late_rotation_plot.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(images=2,source_reports=len(sources))))

if __name__=='__main__':main()
