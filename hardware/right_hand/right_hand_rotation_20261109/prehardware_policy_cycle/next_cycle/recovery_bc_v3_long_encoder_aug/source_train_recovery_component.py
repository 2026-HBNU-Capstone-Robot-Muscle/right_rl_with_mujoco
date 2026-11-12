"""Behavior-clone a small correction component, not the whole SAC controller."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
import time
import json
import numpy as np
import jax
import jax.numpy as jnp
import optax
from native_cycle import OUT,save_json,sha


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--updates',type=int,default=5000)
    parser.add_argument('--tag',default='recovery_bc_v1');parser.add_argument('--use-all-examples',action='store_true')
    parser.add_argument('--dataset-tag',default='recovery_teacher_data')
    parser.add_argument('--encoder-quantization-augmentation',action='store_true')
    args=parser.parse_args()
    folder=OUT/'next_cycle'/args.tag;folder.mkdir(parents=True,exist_ok=True)
    if (folder/'training.json').exists():raise RuntimeError('Preserve completed training')
    from pathlib import Path
    (folder/'source_train_recovery_component.py').write_bytes(Path(__file__).read_bytes())
    path=OUT/'next_cycle'/args.dataset_tag/'examples.npz';data=dict(np.load(path,allow_pickle=False))
    X=data['observations'];Y=data['actions'];validation=data['steps']%5==0
    mean=X[~validation].mean(0);floor=np.r_[np.full(16,.015),np.full(48,.005),np.full(9,.0005),
        np.full(18,.005),np.full(16,.005),[.01,.05,.05]].astype(np.float32)
    std=np.maximum(X[~validation].std(0),floor);values=((X-mean)/std).astype(np.float32)
    keys=jax.random.split(jax.random.PRNGKey(7),3);sizes=(110,128,64,32);params=[]
    for i in range(3):
        w=jax.random.normal(keys[i],(sizes[i],sizes[i+1]))*np.sqrt(2/sizes[i])
        b=jnp.zeros(sizes[i+1]);
        if i==2:w=w*.05;b=b.at[16:].set(-4.)
        params.append((w,b))
    def logits(p,x):
        for i,(w,b) in enumerate(p):
            x=x@w+b
            if i<2:x=jax.nn.relu(x)
        return x[:,:16]
    @jax.jit
    def predict(p,x):return jnp.tanh(logits(p,x))
    def loss(p,x,y):
        raw=logits(p,x);actions=jnp.tanh(raw);target=jnp.arctanh(jnp.clip(y,-.99,.99))
        weights=1.+2.*(jnp.abs(y)>.1)
        return jnp.mean(weights*(actions-y)**2)+.05*jnp.mean((raw-target)**2)
    optimizer=optax.adamw(optax.cosine_decay_schedule(.002,args.updates,alpha=.01),weight_decay=.0001)
    state=optimizer.init(params)
    encoder_tick=2*np.pi/4096
    @jax.jit
    def step(p,s,x,y,key):
        gaussian_key,encoder_key=jax.random.split(key)
        noise=jax.random.normal(gaussian_key,x.shape)*.01
        if args.encoder_quantization_augmentation:
            # The most recent joint error and measured joint angle share the
            # same encoder rounding error; prior history errors are independent.
            err=jax.random.uniform(encoder_key,(x.shape[0],3,16),minval=-.5,maxval=.5)*encoder_tick
            physical=jnp.zeros_like(x).at[:,:16].set(err[:,0,:]).at[:,16:64].set(err.reshape(x.shape[0],48))
            noise=noise+physical/jnp.asarray(std)
        value,grad=jax.value_and_grad(loss)(p,x+noise,y);updates,s=optimizer.update(grad,s,p)
        return optax.apply_updates(p,updates),s,value
    train_mask=np.ones(len(X),bool) if args.use_all_examples else ~validation
    trainX=jnp.asarray(values[train_mask]);trainY=jnp.asarray(Y[train_mask]);valX=jnp.asarray(values[validation]);valY=Y[validation]
    key=jax.random.PRNGKey(8);began=time.perf_counter();best=None;best_error=float('inf');metrics=[]
    for iteration in range(args.updates):
        key,sample_key,noise_key=jax.random.split(key,3)
        indices=jax.random.randint(sample_key,(128,),0,len(trainX))
        params,state,value=step(params,state,trainX[indices],trainY[indices],noise_key)
        if (iteration+1)%500==0 or iteration==0:
            predicted=np.asarray(predict(params,valX));error=float(np.mean((predicted-valY)**2))
            rounded=np.rint(predicted*2);wanted=np.rint(valY*2)
            nonzero=np.abs(wanted)>0
            row=dict(updates=iteration+1,training_loss=float(value),validation_action_mse=error,
                validation_tick_decision_accuracy=float(np.mean(rounded==wanted)),
                validation_nonzero_tick_accuracy=float(np.mean(rounded[nonzero]==wanted[nonzero])))
            metrics.append(row);print(json.dumps(row),flush=True)
            if error<best_error:best_error=error;best=jax.tree_util.tree_map(lambda x:np.array(x),params)
    arrays={'mean':mean,'std':std}
    for i,(w,b) in enumerate(best):arrays[f'w{i}']=w;arrays[f'b{i}']=b
    np.savez_compressed(folder/'actor.npz',**arrays)
    save_json(folder/'training.json',dict(status='offline_behavior_cloning_component_complete',
        architecture=[110,128,64,32],deterministic_actions='tanh(first16 outputs)',
        action_contract='incremental sim-order maximum2ticks per75ms; no reference joint path',
        purpose='First-quarter final correction only; not a learned second90 rotation',
        dataset_sha256=sha(path),dataset_tag=args.dataset_tag,dataset_examples=len(X),trajectory_cases=3,
        train_examples=int(train_mask.sum()),validation_examples_are_also_training=args.use_all_examples,
        fitting_all_examples_is_demo_reproduction_not_generalization=args.use_all_examples,
        validation_is_time_subsample_not_an_independent_trajectory=True,updates=args.updates,
        best_validation_action_mse=best_error,metrics=metrics,actor_sha256=sha(folder/'actor.npz'),
        normalization_std_floors=floor.tolist(),source_teacher_is_privileged_model_control=True,
         encoder_quantization_augmentation=args.encoder_quantization_augmentation,
         encoder_augmentation_tick_halfwidth=.5 if args.encoder_quantization_augmentation else 0.,
        new_SAC_training_steps=0,is_whole_cycle_distillation=False,
        hardware_execution_allowed=False,devices_opened=0,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
