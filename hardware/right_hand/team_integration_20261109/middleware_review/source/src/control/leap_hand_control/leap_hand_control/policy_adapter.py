"""Software-only LEAP policy inference adapter.

This module implements the NumPy MLP inference format used by the reference
deployment policy. It has no Dynamixel imports and cannot send motor commands.
"""

from pathlib import Path


OBSERVATION_SIZE = 108
ACTION_SIZE = 16


class LeapPolicyAdapter:
    """Load a compatible policy archive and map one observation to 16 actions."""

    def __init__(self, policy_path):
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError('NumPy is required for LEAP policy inference.') from exc

        self._np = np
        path = Path(policy_path).expanduser()
        if not path.is_file():
            raise ValueError(f'LEAP policy file does not exist: {path}')

        try:
            with np.load(path, allow_pickle=False) as archive:
                self._mean = np.asarray(archive['mean'], dtype=np.float64)
                self._std = np.asarray(archive['std'], dtype=np.float64)
                self._action_size = int(np.asarray(archive['action_size']).item())
                self._layers = self._load_layers(archive)
        except (KeyError, OSError, ValueError) as exc:
            raise ValueError(f'Invalid LEAP policy archive: {exc}') from exc

        if self._mean.shape != (OBSERVATION_SIZE,):
            raise ValueError(
                f'Policy mean must have shape ({OBSERVATION_SIZE},), '
                f'got {self._mean.shape}.')
        if self._std.shape != (OBSERVATION_SIZE,):
            raise ValueError(
                f'Policy std must have shape ({OBSERVATION_SIZE},), '
                f'got {self._std.shape}.')
        if (not np.all(np.isfinite(self._mean))
                or not np.all(np.isfinite(self._std))
                or np.any(self._std <= 0)):
            raise ValueError('Policy normalization values must be finite; std must be positive.')
        if self._action_size != ACTION_SIZE:
            raise ValueError(
                f'Policy action_size must be {ACTION_SIZE}, got {self._action_size}.')

        input_size = OBSERVATION_SIZE
        for index, layer in enumerate(self._layers):
            kernel, bias = layer
            if kernel.ndim != 2 or bias.ndim != 1:
                raise ValueError(f'Policy layer {index} has invalid kernel/bias ranks.')
            if kernel.shape[0] != input_size or kernel.shape[1] != bias.shape[0]:
                raise ValueError(f'Policy layer {index} has incompatible dimensions.')
            if not np.all(np.isfinite(kernel)) or not np.all(np.isfinite(bias)):
                raise ValueError(f'Policy layer {index} contains non-finite weights.')
            input_size = kernel.shape[1]
        if not self._layers or input_size < ACTION_SIZE:
            raise ValueError('Policy network must produce at least 16 action values.')

    @staticmethod
    def _load_layers(archive):
        """Read W:params/hidden_N/kernel and bias arrays in layer order."""
        np = __import__('numpy')
        layer_data = {}
        for key in archive.files:
            if not key.startswith('W:'):
                continue
            parts = key[2:].split('/')
            layer_name = next(
                (part for part in parts if part.startswith('hidden_')), None)
            if layer_name is None:
                continue
            try:
                index = int(layer_name.removeprefix('hidden_'))
            except ValueError as exc:
                raise ValueError(f'Invalid policy layer name: {layer_name}') from exc
            layer_data.setdefault(index, {})[parts[-1]] = np.asarray(
                archive[key], dtype=np.float64)

        if not layer_data or sorted(layer_data) != list(range(len(layer_data))):
            raise ValueError('Policy must contain contiguous hidden_0...hidden_N layers.')

        layers = []
        for index in sorted(layer_data):
            values = layer_data[index]
            if 'kernel' not in values or 'bias' not in values:
                raise ValueError(f'Policy layer {index} is missing kernel or bias.')
            layers.append((values['kernel'], values['bias']))
        return layers

    def infer(self, observation):
        """Return the deterministic 16-value action for a 108-value observation."""
        np = self._np
        values = np.asarray(observation, dtype=np.float64)
        if values.shape != (OBSERVATION_SIZE,):
            raise ValueError(
                f'Observation must have shape ({OBSERVATION_SIZE},), got {values.shape}.')
        if not np.all(np.isfinite(values)):
            raise ValueError('Observation contains non-finite values.')

        values = (values - self._mean) / self._std
        for index, (kernel, bias) in enumerate(self._layers):
            values = values @ kernel + bias
            if index < len(self._layers) - 1:
                values = np.maximum(values, 0.0)
        action = np.tanh(values[:ACTION_SIZE])
        if action.shape != (ACTION_SIZE,) or not np.all(np.isfinite(action)):
            raise ValueError('Policy produced an invalid action vector.')
        return action
