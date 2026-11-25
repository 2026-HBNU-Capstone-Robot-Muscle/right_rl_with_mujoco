"""Select only freshly matching complete proofs; preserve historical files."""
import hashlib
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent


def main():
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture','slow_grasp_demo_fixture'):
        folder=ROOT/'right_hand_rotation_20261023'/label
        kind=SlowGraspDemoPlan if label=='slow_grasp_demo_fixture' else GraspedYawPlan
        proof=validate_proof(kind(folder/'fresh_off_request.json'),folder/'screen.json')
        canonical=folder/'native_screen.json'
        if canonical.exists():
            old=canonical.read_bytes();archive=folder/('before_grasped_yaw_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
            if not archive.exists():archive.write_bytes(old)
        canonical.write_bytes((folder/'screen.json').read_bytes())
        print(label,'PASS',len(proof['static_screens']),len(proof['dynamic_sensitivity']),flush=True)


if __name__=='__main__':main()
