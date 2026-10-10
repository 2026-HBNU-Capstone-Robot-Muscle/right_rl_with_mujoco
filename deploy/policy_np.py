"""numpy 만으로 도는 정책 추론. export_policy.py 가 만든 .npz 를 읽는다."""
import numpy as np


class Policy:
    def __init__(self, npz_path):
        z = np.load(npz_path)
        self.mean = z["mean"].astype(np.float64)
        self.std = z["std"].astype(np.float64)
        self.action_size = int(z["action_size"])
        # "W:params/hidden_0/kernel" 형태에서 층 번호를 뽑아 순서대로 쌓는다
        layers = {}
        for k in z.files:
            if not k.startswith("W:"):
                continue
            parts = k[2:].split("/")
            name = [p for p in parts if p.startswith("hidden_")]
            if not name:
                continue
            i = int(name[0].split("_")[1])
            layers.setdefault(i, {})[parts[-1]] = z[k].astype(np.float64)
        self.layers = [layers[i] for i in sorted(layers)]

    def __call__(self, obs):
        x = (np.asarray(obs, np.float64) - self.mean) / self.std
        n = len(self.layers)
        for i, L in enumerate(self.layers):
            x = x @ L["kernel"] + L["bias"]
            if i < n - 1:
                x = np.maximum(x, 0.0)          # relu
        mu = x[:self.action_size]               # 뒤 절반은 log_std
        return np.tanh(mu)                      # 결정적 추론 = tanh(mean)
