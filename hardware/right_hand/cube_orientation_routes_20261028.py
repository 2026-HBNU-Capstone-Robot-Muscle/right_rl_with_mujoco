"""Exact face-normal planning for the two observed quarter-turn directions.

Logical orientation planning only. This never concatenates motor trajectories,
certifies a physical connection, or treats an observed82-88deg turn as exact90.
"""
import argparse
import json
from collections import deque
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection'
NAMES=('white','yellow','red','orange','blue','green')
NORMALS=np.array(((0,0,1),(0,0,-1),(1,0,0),(-1,0,0),(0,1,0),(0,-1,0)))
UP=np.array([0,0,1]);WRIST=np.array([-1,0,0])
REFERENCE=np.array(((1,0,0),(0,0,1),(0,-1,0)))
MOVES={
    'yaw': np.array(((0,-1,0),(1,0,0),(0,0,1))),
    'pitch': np.array(((0,0,-1),(0,1,0),(1,0,0))),
}

def faces(matrix):
    normals=NORMALS@matrix.T
    return (NAMES[int(np.flatnonzero(np.all(normals==UP,axis=1))[0])],
            NAMES[int(np.flatnonzero(np.all(normals==WRIST,axis=1))[0])])

def routes(start=REFERENCE):
    queue=deque([(start.copy(),[])])
    seen={tuple(start.ravel())}
    result={}
    while queue:
        R,path=queue.popleft()
        assert round(np.linalg.det(R))==1 and np.array_equal(R@R.T,np.eye(3,dtype=int))
        result[faces(R)]=(R,path)
        for name,move in MOVES.items():
            target=move@R;key=tuple(target.ravel())
            if key not in seen:
                seen.add(key);queue.append((target,path+[name]))
    assert len(result)==24
    return result

def validate():
    assert faces(REFERENCE)==('green','orange')
    assert faces(MOVES['pitch']@REFERENCE)==('red','green')
    assert faces(MOVES['yaw']@REFERENCE)==('green','white')
    for move in MOVES.values():
        assert np.array_equal(np.linalg.matrix_power(move,4),np.eye(3,dtype=int))
    all_routes=routes()
    expected={(a,b) for a,n in zip(NAMES,NORMALS) for b,p in zip(NAMES,NORMALS) if n@p==0}
    assert set(all_routes)==expected
    for start,_ in all_routes.values():
        assert len(routes(start))==24
    # The user's earlier right-side-first example has the opposite yaw sign
    # to the observed WHITE-to-wrist primitive. Three observed yaws are its
    # logical inverse; no inverse motor recording is invented here.
    assert faces(MOVES['pitch']@np.linalg.matrix_power(MOVES['yaw'],3)@REFERENCE)[0]=='white'
    return all_routes

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--up',choices=NAMES)
    parser.add_argument('--wrist',choices=NAMES)
    args=parser.parse_args()
    table=validate()
    if bool(args.up)!=bool(args.wrist):parser.error('Give both --up and --wrist')
    if args.up:
        key=(args.up,args.wrist)
        if key not in table:parser.error('Up and wrist faces must be adjacent, not identical or opposite')
        print(json.dumps(dict(up=key[0],wrist=key[1],logical_route=table[key][1],
                              physical_connection_verified=False,hardware_execution_allowed=False)))
        return
    report=dict(status='exact_discrete_orientation_routes_for_observed_directions',
                reference_up='green',reference_wrist='orange',
                logical_quarter_turns_are_idealized=True,
                actual_turn_angle_not_assumed_exact=True,
                observed_yaw_reference_end=dict(up='green',wrist='white'),
                observed_pitch_reference_end=dict(up='red',wrist='green'),
                directions_labelled_by_face_transition_not_ambiguous_left_right=True,
                states=len(table),all24_states_reachable_from_each_state=True,
                maximum_logical_steps=max(len(p) for _,p in table.values()),
                routes=[dict(up=a,wrist=b,moves=p,steps=len(p)) for (a,b),(_,p) in sorted(table.items())],
                requires_vision_confirmation_between_real_moves=True,
                requires_loaded_regrasp_validation=True,
                cube_solving_planner=False,hardware_execution_allowed=False,real_devices_opened=False)
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'orientation_routes.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('status','states','maximum_logical_steps','hardware_execution_allowed')}))

if __name__=='__main__':main()
