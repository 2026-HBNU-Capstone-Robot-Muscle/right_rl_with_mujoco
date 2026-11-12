"""A labelled synthetic one-tick start to exercise fresh recovery, no devices."""
import json
from horizontal_quarter_plan_20261027 import ROOT,REQUEST,OUT
folder=OUT/'synthetic_one_tick_probe';folder.mkdir(exist_ok=True);data=json.loads(REQUEST.read_bytes());data['off_state']['position'][0]+=1;data['origin']='SYNTHETIC_ONE_TICK_NO_DEVICE_TEST';data['synthetic_not_physical_measurement']=True;(folder/'request.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8');print(folder/'request.json')
