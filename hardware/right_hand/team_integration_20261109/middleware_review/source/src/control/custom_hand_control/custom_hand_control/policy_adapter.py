"""Hardware-free inference for the Custom Hand Stable-Baselines3 policy."""

import base64
import binascii
import hashlib
import io
import json
import zipfile
from pathlib import Path


OBSERVATION_SIZE = 8
ACTION_SIZE = 4
EXPECTED_POLICY_SHA256 = (
    '5B9A3F36B348FA7FCC16370432E9F5CE476091CBEE38431686166FFABABD426B')


class CustomPolicyAdapter:
    """Load a verified-shape PPO actor and infer four normalized finger actions."""

    def __init__(self, policy_path):
        try:
            import numpy as np
            import torch
            import torch.nn.functional as functional
        except ImportError as exc:
            raise RuntimeError('NumPy and PyTorch are required for Custom policy inference.') from exc

        self._np = np
        self._torch = torch
        self._functional = functional
        path = Path(policy_path).expanduser()
        if not path.is_file():
            raise ValueError(f'Custom policy file does not exist: {path}')
        digest = hashlib.sha256(path.read_bytes()).hexdigest().upper()
        if digest != EXPECTED_POLICY_SHA256:
            raise ValueError(
                'Custom policy SHA-256 differs from the middleware-bridge candidate: '
                f'{digest}')

        try:
            with zipfile.ZipFile(path) as archive:
                metadata = json.loads(archive.read('data'))
                state = torch.load(
                    io.BytesIO(archive.read('policy.pth')),
                    map_location='cpu', weights_only=True)
        except (OSError, KeyError, zipfile.BadZipFile, json.JSONDecodeError) as exc:
            raise ValueError(f'Invalid Custom PPO policy archive: {exc}') from exc

        observation_shape = metadata.get('observation_space', {}).get('_shape')
        action_shape = metadata.get('action_space', {}).get('_shape')
        if observation_shape != [OBSERVATION_SIZE] or action_shape != [ACTION_SIZE]:
            raise ValueError(
                'Custom policy must have observation(8) and action(4); '
                f'got observation{observation_shape}, action{action_shape}.')
        if metadata.get('policy_kwargs') not in ({}, None):
            raise ValueError('Custom policy uses unsupported non-default policy_kwargs.')
        policy_class = metadata.get('policy_class', {})
        try:
            serialized_class = base64.b64decode(
                policy_class.get(':serialized:', ''), validate=True)
        except (ValueError, binascii.Error) as exc:
            raise ValueError('Custom policy class metadata is malformed.') from exc
        if (policy_class.get('__module__') != 'stable_baselines3.common.policies'
                or b'ActorCriticPolicy' not in serialized_class):
            raise ValueError('Custom policy archive is not an ActorCriticPolicy model.')

        self._weights = {}
        expected_shapes = {
            'mlp_extractor.policy_net.0.weight': (64, 8),
            'mlp_extractor.policy_net.0.bias': (64,),
            'mlp_extractor.policy_net.2.weight': (64, 64),
            'mlp_extractor.policy_net.2.bias': (64,),
            'action_net.weight': (4, 64),
            'action_net.bias': (4,),
        }
        for name, shape in expected_shapes.items():
            if name not in state or tuple(state[name].shape) != shape:
                found = tuple(state[name].shape) if name in state else 'missing'
                raise ValueError(f'Custom policy tensor {name}: expected {shape}, got {found}.')
            tensor = state[name].detach().to(device='cpu', dtype=torch.float32)
            if not torch.isfinite(tensor).all().item():
                raise ValueError(f'Custom policy tensor {name} contains non-finite values.')
            self._weights[name] = tensor

        self._action_low, self._action_high = self._read_action_bounds(metadata)
        self.sha256 = digest

    @staticmethod
    def _read_action_bounds(metadata):
        import numpy as np

        bounds = []
        for key in ('low', 'high'):
            raw = metadata.get('action_space', {}).get(key)
            if not isinstance(raw, str):
                raise ValueError(f'Custom policy action_space.{key} metadata is missing.')
            values = np.fromstring(raw.strip().strip('[]'), sep=' ')
            if values.shape != (ACTION_SIZE,) or not np.all(np.isfinite(values)):
                raise ValueError(f'Custom policy action_space.{key} is invalid.')
            bounds.append(values)
        low, high = bounds
        if not (np.all(low == -1.0) and np.all(high == 1.0)):
            raise ValueError('Custom policy action range must be [-1, 1].')
        return low, high

    def infer(self, observation):
        """Return one deterministic SB3 PPO action, clipped to its Box bounds."""
        np = self._np
        torch = self._torch
        values = np.asarray(observation, dtype=np.float32)
        if values.shape != (OBSERVATION_SIZE,) or not np.all(np.isfinite(values)):
            raise ValueError('Custom observation must contain 8 finite float32 values.')

        state = torch.from_numpy(values.copy())
        weights = self._weights
        hidden = self._functional.linear(
            state,
            weights['mlp_extractor.policy_net.0.weight'],
            weights['mlp_extractor.policy_net.0.bias'])
        hidden = torch.tanh(hidden)
        hidden = self._functional.linear(
            hidden,
            weights['mlp_extractor.policy_net.2.weight'],
            weights['mlp_extractor.policy_net.2.bias'])
        hidden = torch.tanh(hidden)
        action = self._functional.linear(
            hidden, weights['action_net.weight'], weights['action_net.bias'])
        action = torch.clamp(action, -1.0, 1.0)
        result = action.detach().cpu().numpy().astype(np.float32)
        if result.shape != (ACTION_SIZE,) or not np.all(np.isfinite(result)):
            raise ValueError('Custom policy produced an invalid action vector.')
        return result
