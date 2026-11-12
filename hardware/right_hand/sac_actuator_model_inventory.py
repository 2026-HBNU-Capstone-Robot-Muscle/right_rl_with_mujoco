"""Report simulator actuator/joint parameters; no hardware access."""
import json
import inspect
from pathlib import Path
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);model,_=build(env)
    out={name:getattr(model,name).tolist() for name in ('dof_damping','dof_armature','dof_frictionloss','actuator_gainprm','actuator_biasprm','actuator_forcerange','actuator_ctrlrange','jnt_range','jnt_limited','jnt_actfrcrange','jnt_actfrclimited','actuator_gear','actuator_trnid','actuator_forcelimited')}
    out['joint_names']=[model.joint(i).name for i in range(model.njnt)]
    out['actuator_names']=[model.actuator(i).name for i in range(model.nu)]
    out['config']=str(cfg);out['post_init_source']=inspect.getsource(type(env)._post_init)
    (ROOT/'sac_actuator_model_inventory_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:out[k] for k in ('joint_names','actuator_names','jnt_actfrcrange','jnt_actfrclimited','actuator_gear','actuator_trnid','actuator_forcelimited')},indent=2))

if __name__=='__main__':main()
