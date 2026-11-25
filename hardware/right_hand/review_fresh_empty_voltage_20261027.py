"""Summarize three saved voltage stops; make one saved-video contact sheet."""
import hashlib,json
from pathlib import Path
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parent
def main():
    folders=sorted([p for p in ROOT.glob('fresh_empty_grasp_*') if p.is_dir() and (p/'result.json').exists()],reverse=True)[:3];rows=[]
    for folder in folders:
        path=folder/'result.json';r=json.loads(path.read_bytes());s=r.get('samples',[]);events=r.get('voltage_events',[]);by_phase={}
        for phase in sorted({v['phase'] for v in s}):
            group=[v for v in s if v['phase']==phase];by_phase[phase]=dict(samples=len(group),minimum_voltage_v=min(min(v['voltage_v']) for v in group),root_peak_current_ma=[max(abs(v['current_ma'][i]) for v in group) for i in (7,11)],maximum_tracking_rad=max(max(v['tracking_rad']) for v in group))
        rows.append(dict(source=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),error=r.get('error'),phase=r.get('failed_phase'),all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),by_phase=by_phase,voltage_events=[{k:e.get(k) for k in ('status','t','initial_voltage_v','last_voltage_v','confirmation_reads','healthy_confirmation_count','elapsed_ms')} for e in events],blank_camera_frames=r.get('camera_recording',{}).get('image_signal_blank_frames')))
    out=ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/voltage_stop_review_20261027';out.mkdir(exist_ok=True);(out/'review.json').write_text(json.dumps(dict(trials=rows,assistant_motor_writes=0,voltage_thresholds_unchanged=True,root_target_rate_change_is_a_proposed_mitigation_not_a_verified_electrical_fix=True),indent=2)+'\n',encoding='utf-8')
    folder=folders[0];images=sorted([p for p in (folder/'camera').rglob('*') if p.suffix.lower() in ('.jpg','.png','.jpeg')]);print('Latest saved image count',len(images))
    if images:
        sheet=np.full((310,1440,3),255,np.uint8)
        for i,index in enumerate((0,len(images)//2,len(images)-1)):
            im=cv2.imdecode(np.fromfile(images[index],dtype=np.uint8),cv2.IMREAD_COLOR);sheet[30:300,i*480:(i+1)*480]=cv2.resize(im,(480,270));cv2.putText(sheet,f'saved image {index+1}',(i*480+5,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
        cv2.imencode('.jpg',sheet)[1].tofile(out/'latest_saved_contact_sheet.jpg')
    print(json.dumps([dict(source=v['source'],error=v['error'],by_phase=v['by_phase'],off=v['all_torque_off_verified']) for v in rows],indent=2))
if __name__=='__main__':main()
