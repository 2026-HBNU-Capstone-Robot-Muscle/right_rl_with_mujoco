"""Inventory available actual encoder/command samples; no device reopening."""
from pathlib import Path
import json
import hashlib
from collections import Counter
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def main():
    path=ROOT/'middle_clearance_probe_20261106_230902_998326/result.json';doc=json.loads(path.read_text(encoding='utf-8'))
    structure={}
    for key,value in doc.items():
        if isinstance(value,list):structure[key]={'type':'list','length':len(value),'first_keys':list(value[0]) if value and isinstance(value[0],dict) else None}
        elif isinstance(value,dict):structure[key]={'type':'dict','keys':list(value)[:40]}
    candidates=[]
    def walk(node,label):
        if isinstance(node,dict):
            actual=node.get('actual');goal=node.get('goal',node.get('target'))
            if isinstance(actual,list) and len(actual)==16:
                candidates.append({'source':label,'keys':list(node),'phase':node.get('phase'),
                                   'has_goal16':isinstance(goal,list) and len(goal)==16,
                                   'timestamp':node.get('timestamp',node.get('time',node.get('t',node.get('elapsed_s'))))})
            for key,value in node.items():
                if isinstance(value,(dict,list)):walk(value,label+'/'+key)
        elif isinstance(node,list):
            for i,value in enumerate(node):
                if isinstance(value,(dict,list)):walk(value,label+'/'+str(i))
    walk(doc,'result')
    report={'source':str(path.relative_to(ROOT)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'status':doc.get('status'),'root_structure':structure,'actual16_records':len(candidates),
            'sample_record_schema':candidates[:5]+candidates[-5:],
            'recorded_geometric_calibration_verified':doc.get('joint_mapping',{}).get('geometric_calibration_verified'),
            'no_current_to_torque_conversion_assumed':True,'devices_opened':0,'hardware_execution_allowed':False}
    report['phase_counts']=dict(Counter(s['phase'] for s in doc['samples']))
    (HERE/'actual_tracking_inventory.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('source','actual16_records','phase_counts','recorded_geometric_calibration_verified')}),flush=True)


if __name__=='__main__':main()
