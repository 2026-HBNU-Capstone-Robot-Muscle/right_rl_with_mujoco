"""Evaluate the small-turn curriculum without reset between action samples."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
import time
import json
import numpy as np
import jax
from brax.training.agents.sac import checkpoint
from next_turn_env import NextTurn
from learning_env import decode
from export_and_evaluate import Actor
from native_cycle import OUT,save_json,sha


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',default='sac_next_turn5_10k');p.add_argument('--step',type=int)
    p.add_argument('--mode',choices=('trained','zero'),default='trained');a=p.parse_args()
    parent=OUT/a.run;settings=json.loads((parent/'settings.json').read_text())
    env_type=NextTurn
    if settings.get('fractional_tick_intent_accumulated'):
        from fractional_next_env import FractionalNextTurn
        env_type=FractionalNextTurn
    env=env_type(settings['curriculum_increment_deg']);mode=a.mode
    checkpoints=sorted((parent/'checkpoints').glob('[0-9]*'),key=lambda p:int(p.name))
    selected=next(q for q in checkpoints if int(q.name)==a.step) if a.step is not None else checkpoints[-1]
    folder=parent/('evaluation_'+(selected.name if mode=='trained' else 'zero'));folder.mkdir(exist_ok=True)
    if mode=='trained':
        norm,params=checkpoint.load(str(selected));arrays={'mean':np.asarray(norm.mean),'std':np.asarray(norm.std)}
        for i in range(3):
            layer=params['params'][f'hidden_{i}'];arrays[f'w{i}']=np.asarray(layer['kernel']);arrays[f'b{i}']=np.asarray(layer['bias'])
        np.savez_compressed(folder/'actor.npz',**arrays);policy=Actor(folder/'actor.npz')
        rng=np.random.default_rng(8);sample=(arrays['mean']+arrays['std']*rng.normal(size=(128,110))).astype(np.float32)
        jactor=checkpoint.load_policy(str(selected),deterministic=True)
        with jax.default_matmul_precision('highest'):
            expected=np.asarray(jax.jit(jax.vmap(lambda o:jactor(o,jax.random.PRNGKey(0))[0]))(sample))
        parity=float(np.max(np.abs(expected-np.stack([policy.action(o) for o in sample]))))
        if parity>=1e-5:raise AssertionError(parity)
    else:parity=None
    words,obs=env._reset_one(np.array([0,0],dtype=np.uint32));initial_error=float(np.degrees(obs[107]));trace=[];began=time.perf_counter()
    for step in range(env.horizon):
        action=policy.action(obs) if mode=='trained' else np.zeros(16)
        new,new_obs,reward,done,metrics=env._step_one(words,obs,action)
        trace.append({'step':step+1,'goal_error_deg':float(metrics[7]),'center_mm':float(metrics[2]),
                      'fault':bool(metrics[0]),'lower_grip':bool(metrics[5]),'upper_gap_mm':float(metrics[6]),
                      'success':bool(metrics[4]),'reward':float(reward)})
        words,obs=new,new_obs
        if done:break
    vector=decode(words)
    report={'status':'offline_next_turn_curriculum_evaluation','mode':mode,'checkpoint':selected.name,
            'increment_deg':settings['curriculum_increment_deg'],'target_tolerance_deg':env.tolerance,
            'steps_completed':step+1,'success':trace[-1]['success'],'fault':trace[-1]['fault'],
            'initial_goal_error_deg':initial_error,
            'final_goal_error_deg':trace[-1]['goal_error_deg'],'final_upper_gap_mm':trace[-1]['upper_gap_mm'],
            'trace':trace,'actor_jax_highest_numpy_parity':parity,'no_cube_reset_during_suffix':True,
            'this_is_not_a_second90_degree_turn':True,'second90_degree_rotation_passed':False,
            'devices_opened':0,'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
    save_json(folder/'review.json',report)
    np.savez_compressed(folder/'end_state.npz',qpos=vector[env.slices['qpos']],qvel=vector[env.slices['qvel']],
                        ctrl=vector[env.slices['ctrl']],qacc_warmstart=vector[env.slices['qacc_warmstart']],time=vector[83])
    print(json.dumps({key:value for key,value in report.items() if key!='trace'}),flush=True)


if __name__=='__main__':main()
