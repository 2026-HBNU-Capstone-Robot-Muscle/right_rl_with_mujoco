# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Train a SAC agent using JAX on the specified environment."""

import datetime
import functools
import inspect
import json
import os
import signal
import subprocess
import threading
import time
import warnings

from absl import app
from absl import flags
from absl import logging
from brax.training.acme import running_statistics
from brax.training.agents.sac import networks as sac_networks
from brax.training.agents.sac import train as sac
from brax.training import replay_buffers
from etils import epath
import jax
import jax.numpy as jp
import mediapy as media
from ml_collections import config_dict
import mujoco
import mujoco_playground
import numpy as np
import psutil
from orbax import checkpoint as ocp
from mujoco_playground import registry
from mujoco_playground import wrapper
from mujoco_playground.config import manipulation_params

import sac_asym_networks
import sac_simba_networks
import sac_demo_mix
import sac_nstep
import residual_wrapper

try:
  import tensorboardX
except ImportError:
  tensorboardX = None

try:
  import wandb
except ImportError:
  wandb = None


xla_flags = os.environ.get("XLA_FLAGS", "")
xla_flags += " --xla_gpu_triton_gemm_any=True"
os.environ["XLA_FLAGS"] = xla_flags
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["MUJOCO_GL"] = "egl"

logging.set_verbosity(logging.WARNING)
warnings.filterwarnings("ignore", category=RuntimeWarning, module="jax")
warnings.filterwarnings("ignore", category=DeprecationWarning, module="jax")
warnings.filterwarnings("ignore", category=UserWarning, module="absl")


_ENV_NAME = flags.DEFINE_string(
    "env_name",
    "LeapCubeReorient",
    f"Name of the environment. One of {', '.join(registry.ALL_ENVS)}",
)
_IMPL = flags.DEFINE_enum("impl", "jax", ["jax", "warp"], "MJX implementation")
_PLAYGROUND_CONFIG_OVERRIDES = flags.DEFINE_string(
    "playground_config_overrides",
    None,
    "Overrides for the playground env config.",
)
_LOAD_CHECKPOINT_PATH = flags.DEFINE_string(
    "load_checkpoint_path", None, "Path to load checkpoint from"
)
_LOAD_REPLAY_BUFFER_PATH = flags.DEFINE_string(
    "load_replay_buffer_path",
    None,
    "Optional Orbax replay buffer directory to seed SAC training from.",
)
_SAVE_REPLAY_BUFFER_PATH = flags.DEFINE_string(
    "save_replay_buffer_path",
    None,
    "Optional Orbax directory where the final SAC replay buffer is saved.",
)
_DEMO_DATA_PATH = flags.DEFINE_string(
    "demo_data_path",
    None,
    "시연 전용 버퍼(Orbax) 경로. 지정하면 온라인 버퍼와 분리해 읽기 전용으로 두고 "
    "매 배치에서 demo_ratio 만큼을 여기서 뽑는다 (A1).",
)
_DEMO_RATIO = flags.DEFINE_float(
    "demo_ratio", 0.25, "배치에서 시연이 차지하는 비율 (demo_data_path 지정 시)"
)
_BC_COEF = flags.DEFINE_float(
    "bc_coef",
    0.0,
    "actor 손실에 더할 BC 정규화 계수 (B1). 0 이면 끔. mean|Q| 로 정규화된 값.",
)
# SimBa (ICLR 2025, arXiv:2410.09754): 잔차 블록 + LayerNorm 으로 단순성 편향을
# 주입해 파라미터/업데이트를 키울 수 있게 만드는 구조. SAC 에 구조만 갈아끼워도
# 여러 벤치마크에서 SOTA 급이라고 보고한다. 우리 문제(정책이 수렴 못 하고 배회,
# 체크포인트 간 SD 3.5%p)에 대한 구조적 처방 후보다.
_SIMBA = flags.DEFINE_boolean("simba", False, "SimBa 구조 네트워크 사용")
_SIMBA_HIDDEN = flags.DEFINE_integer(
    "simba_hidden_dim", 128, "SimBa 은닉 폭 (128 이면 기준선과 파라미터 수가 비슷)"
)
_SIMBA_POLICY_BLOCKS = flags.DEFINE_integer("simba_policy_blocks", 1, "policy 잔차 블록 수")
_SIMBA_CRITIC_BLOCKS = flags.DEFINE_integer("simba_critic_blocks", 2, "critic 잔차 블록 수")
_SIMBA_CRITIC_HIDDEN = flags.DEFINE_integer(
    "simba_critic_hidden_dim", 0, "critic 폭 (0 이면 policy 와 동일)"
)

_ASYMMETRIC_CRITIC = flags.DEFINE_boolean(
    "asymmetric_critic",
    False,
    "critic 은 privileged_state 전체를, policy 는 그 앞부분(state)만 보게 한다 (E1). "
    "--obs_key privileged_state 와 함께 써야 한다.",
)
_N_CRITICS = flags.DEFINE_integer(
    "n_critics", 2, "critic 앙상블 크기. brax 기본은 2. 늘리면 타깃이 더 보수적."
)
_BC_Q_FILTER = flags.DEFINE_boolean(
    "bc_q_filter",
    False,
    "BC 항을 시연 Q > 정책 Q 인 전이에만 적용한다 (Q-filter). 정책이 시연을 "
    "넘어선 뒤 모방이 발목 잡는 것을 막는다.",
)
_SUFFIX = flags.DEFINE_string("suffix", None, "Suffix for the experiment name")
_PLAY_ONLY = flags.DEFINE_boolean(
    "play_only", False, "If true, only play with the model and do not train"
)
_USE_WANDB = flags.DEFINE_boolean("use_wandb", False, "Use W&B logging")
_USE_TB = flags.DEFINE_boolean("use_tb", False, "Use TensorBoard logging")
_DOMAIN_RANDOMIZATION = flags.DEFINE_boolean(
    "domain_randomization", False, "Use domain randomization"
)
_SEED = flags.DEFINE_integer("seed", 1, "Random seed")
_NUM_TIMESTEPS = flags.DEFINE_integer(
    "num_timesteps", 6_000_000, "Number of timesteps"
)
_NUM_VIDEOS = flags.DEFINE_integer(
    "num_videos", 1, "Number of videos to record after training."
)
_VIDEO_FPS = flags.DEFINE_float("video_fps", 30.0, "FPS for saved videos.")
_NUM_EVALS = flags.DEFINE_integer("num_evals", 7, "Number of evaluations")
_REWARD_SCALING = flags.DEFINE_float("reward_scaling", 1.0, "Reward scaling")
_EPISODE_LENGTH = flags.DEFINE_integer("episode_length", 800, "Episode length")
_NORMALIZE_OBSERVATIONS = flags.DEFINE_boolean(
    "normalize_observations", True, "Normalize observations"
)
_ACTION_REPEAT = flags.DEFINE_integer("action_repeat", 1, "Action repeat")
_DISCOUNTING = flags.DEFINE_float("discounting", 0.99, "Discounting")
_LEARNING_RATE = flags.DEFINE_float("learning_rate", 5e-5, "Learning rate")
_TAU = flags.DEFINE_float("tau", 0.005, "Target critic Polyak coefficient")
_NUM_ENVS = flags.DEFINE_integer("num_envs", 1024, "Number of environments")
_NUM_EVAL_ENVS = flags.DEFINE_integer(
    "num_eval_envs", 128, "Number of evaluation environments"
)
_BATCH_SIZE = flags.DEFINE_integer("batch_size", 1024, "Batch size")
_MIN_REPLAY_SIZE = flags.DEFINE_integer(
    "min_replay_size", 32_768, "Minimum replay size before training"
)
_MAX_REPLAY_SIZE = flags.DEFINE_integer(
    "max_replay_size", 500_000, "Maximum replay buffer size"
)
_GRAD_UPDATES_PER_STEP = flags.DEFINE_integer(
    "grad_updates_per_step", 1, "Gradient updates per environment step"
)
_HIDDEN_LAYER_SIZES = flags.DEFINE_list(
    "hidden_layer_sizes", [256, 256], "SAC actor/critic hidden sizes"
)
_Q_LAYER_NORM = flags.DEFINE_boolean(
    "q_network_layer_norm",
    None,
    "critic MLP 의 LayerNorm. 지정하지 않으면 playground 설정값을 쓴다. "
    "LeapCube* 의 기본값은 이미 True 이므로 이 플래그는 사실상 "
    "'끄고 재보는' 용도다(RLPD 계열 절제 실험). critic 파라미터는 "
    "체크포인트에 저장되지 않으므로(policy 만 저장) --load_checkpoint_path "
    "와 함께 써도 무방하다.",
)
_RESIDUAL_TEACHER = flags.DEFINE_string(
    "residual_teacher", None,
    "지정하면 학생 행동을 이 PPO 교사에 대한 **교정량**으로 해석한다: "
    "a = clip(a_교사(s) + residual_scale * a_학생(s), -1, 1). "
    "핸드오프 진단(2026-08-27)에서 학생이 자기 궤적 위에서만 못 하는 것이 "
    "확인됐다(교사가 25스텝 굴려주면 80.8->93.6%). residual 은 자기 궤적 위에서도 "
    "base 가 교사 행동을 내므로 그 분포 이탈을 구조적으로 없앤다. "
    "residual=0 이 이미 교사 성능이므로 BC(--demo_data_path)는 쓰지 않는다.")
_RESIDUAL_SCALE = flags.DEFINE_float(
    "residual_scale", 0.3, "교정량의 크기. 행동 공간(-1~1) 기준.")
_RESIDUAL_L2 = flags.DEFINE_float(
    "residual_l2", 0.0,
    "residual 크기에 대한 스텝당 보상 페널티 계수 (sum(a_res^2) 에 곱한다). "
    "0 이면 끔. residual 을 0 근처로 당기는 유일한 힘이다.")
_FULL_RESET = flags.DEFINE_boolean(
    "full_reset",
    False,
    "에피소드 종료 시 env.reset 을 다시 호출한다. 기본값 False 면 brax "
    "AutoResetWrapper 가 **캐시된 첫 상태**로 되돌리는데, 그 data 에는 목표 면을 "
    "인코딩한 mocap_quat 과 큐브·손 초기 자세가 들어 있다. 즉 각 환경이 학습 "
    "내내 자기 첫 에피소드의 목표 면·초기조건만 반복한다(실측 확인: 96회 종료 "
    "후에도 환경당 목표 면 1개). 초기 상태 다양성이 num_envs 개로 제한되고, "
    "면별 초기조건 배정이 시드에 고정된다. True 면 매 종료마다 재샘플링한다 "
    "(reset 호출 비용만큼 느려진다).",
)
_N_STEP = flags.DEFINE_integer(
    "n_step",
    1,
    "n-step 반환의 n. 1 이면 brax 기본(1-step TD). RSL-RL-SAC(2605.24975)는 "
    "병렬 시뮬에서 SAC 이 PPO 에 못 미치는 원인 중 하나로 1-step TD 의 느린 "
    "보상 전파를 지목하고 5-step 을 쓴다. 우리 과제는 성공 보상이 유지 100스텝을 "
    "채운 뒤 단발로 오고 성공 에피소드 평균 길이가 221스텝이라 정확히 이 문제다. "
    "주의: 겹치지 않는 창이므로 삽입되는 전이 수가 1/n 로 준다(환경 수가 크면 "
    "무해). env_steps_per_actor_step 도 n 배로 보정된다.",
)
_INIT_NOISE_STD = flags.DEFINE_float(
    "init_noise_std",
    None,
    "정책의 초기 표준편차. 지정하지 않으면 brax 기본값 1.0 을 쓴다. "
    "RSL-RL-SAC(2605.24975)는 이 값이 크면 squashed Gaussian 이 행동 범위 "
    "전체로 확률을 퍼뜨려 학습 초기부터 난폭하게 움직인다고 지적하고 0.15 를 "
    "쓴다. 주의: std 는 정책 파라미터이므로 --load_checkpoint_path 로 "
    "이어받으면 체크포인트 값에 덮어써진다. 처음부터 학습할 때만 의미가 있다.",
)
_OBS_KEY = flags.DEFINE_string(
    "obs_key", "state", "Observation key to expose to SAC"
)
_DETERMINISTIC_EVAL = flags.DEFINE_boolean(
    "deterministic_eval", True, "Use deterministic policy for evaluation"
)
_LOGDIR = flags.DEFINE_string("logdir", None, "Directory for logging.")
_RESOURCE_MONITOR_INTERVAL = flags.DEFINE_float(
    "resource_monitor_interval_sec",
    10.0,
    "Seconds between resource monitor samples.",
)
_RESOURCE_GPU_LIMIT_PERCENT = flags.DEFINE_float(
    "resource_gpu_limit_percent",
    0.0,
    "If > 0, stop the recorded PGID when its summed PID GPU utilization reaches this percent.",
)
_RESOURCE_GPU_MEM_LIMIT_PERCENT = flags.DEFINE_float(
    "resource_gpu_mem_limit_percent",
    40.0,
    "If > 0, stop the recorded PGID when its summed PID GPU memory reaches this percent of total GPU memory.",
)
_RESOURCE_RAM_LIMIT_PERCENT = flags.DEFINE_float(
    "resource_ram_limit_percent",
    40.0,
    "If > 0, stop the recorded PGID when its summed RSS reaches this percent of total RAM.",
)
_RESOURCE_CPU_LIMIT_PERCENT = flags.DEFINE_float(
    "resource_cpu_limit_percent",
    60.0,
    "If > 0, stop the recorded PGID when its normalized CPU reaches this percent.",
)
_KILL_ON_RESOURCE_EXCEED = flags.DEFINE_boolean(
    "kill_on_resource_exceed",
    False,
    "Kill the recorded PGID immediately when a resource threshold is exceeded.",
)


class ResourceLimitExceeded(RuntimeError):
  """Raised when resource usage exceeds a configured limit."""


class ResourceMonitor:
  """Samples process/system resources while training is running."""

  def __init__(
      self,
      logdir,
      interval_sec: float = 1.0,
      gpu_limit_percent: float = 0.0,
      ram_limit_percent: float = 0.0,
      cpu_limit_percent: float = 0.0,
      kill_on_exceed: bool = False,
  ):
    self._logdir = logdir
    self._interval_sec = interval_sec
    self._limits = {
        "pgid_gpu_util_percent": float(gpu_limit_percent or 0.0),
        "pgid_gpu_mem_percent": float(_RESOURCE_GPU_MEM_LIMIT_PERCENT.value or 0.0),
        "pgid_rss_percent": float(ram_limit_percent or 0.0),
        "normalized_pgid_cpu_percent": float(cpu_limit_percent or 0.0),
    }
    self._kill_on_exceed = kill_on_exceed
    self._stop_event = threading.Event()
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._process = psutil.Process(os.getpid())
    self._pid = os.getpid()
    self._recorded_pgid = os.getpgid(self._pid)
    self._nproc = max(psutil.cpu_count(logical=True) or 1, 1)
    self._pgid_cpu_time_cache = {}
    self._samples = []
    self._limit_event = None
    self._summary = {
        "recorded_pid": self._pid,
        "recorded_pgid": self._recorded_pgid,
        "sample_count": 0,
        "max_system_cpu_percent": 0.0,
        "max_system_ram_percent": 0.0,
        "max_system_ram_used_mb": 0.0,
        "max_process_cpu_percent": 0.0,
        "max_normalized_process_cpu_percent": 0.0,
        "max_process_rss_mb": 0.0,
        "max_pgid_cpu_percent": 0.0,
        "max_normalized_pgid_cpu_percent": 0.0,
        "max_pgid_rss_mb": 0.0,
        "max_pgid_rss_percent": 0.0,
        "max_gpu_util_percent": None,
        "max_gpu_mem_used_mb": None,
        "max_pgid_gpu_util_percent": None,
        "max_pgid_gpu_mem_used_mb": None,
        "max_pgid_gpu_mem_percent": None,
        "resource_limits": dict(self._limits),
        "kill_on_resource_exceed": self._kill_on_exceed,
        "resource_limit_exceeded": False,
        "resource_limit_event": None,
    }

  def start(self):
    psutil.cpu_percent(interval=None)
    self._process.cpu_percent(interval=None)
    self._thread.start()

  def stop(self):
    self._stop_event.set()
    self._thread.join(timeout=self._interval_sec + 1.0)
    return self._summary

  @property
  def limit_event(self):
    return self._limit_event

  def _pids_in_recorded_pgid(self):
    pids = []
    for process in psutil.process_iter(["pid"]):
      try:
        if os.getpgid(process.info["pid"]) == self._recorded_pgid:
          pids.append(process.info["pid"])
      except (ProcessLookupError, PermissionError, psutil.Error):
        continue
    return pids

  def _pgid_process_stats(self, pids):
    rss_mb = 0.0
    cpu_percent = 0.0
    now = time.monotonic()
    next_cache = {}
    for pid in pids:
      try:
        process = psutil.Process(pid)
        rss_mb += float(process.memory_info().rss / 1024**2)
        cpu_times = process.cpu_times()
        total_cpu_time = float(cpu_times.user + cpu_times.system)
        previous = self._pgid_cpu_time_cache.get(pid)
        if previous is not None:
          previous_time, previous_cpu_time = previous
          wall_delta = max(now - previous_time, 1e-6)
          cpu_percent += max(total_cpu_time - previous_cpu_time, 0.0) / wall_delta * 100.0
        next_cache[pid] = (now, total_cpu_time)
      except psutil.Error:
        continue
    self._pgid_cpu_time_cache = next_cache
    total_ram_mb = psutil.virtual_memory().total / 1024**2
    return {
        "pgid_process_count": len(pids),
        "pgid_cpu_percent": cpu_percent,
        "normalized_pgid_cpu_percent": cpu_percent / self._nproc,
        "pgid_rss_mb": rss_mb,
        "pgid_rss_percent": (rss_mb / total_ram_mb * 100.0)
        if total_ram_mb
        else 0.0,
    }

  def _query_gpu(self):
    try:
      result = subprocess.run(
          [
              "nvidia-smi",
              "--query-gpu=utilization.gpu,memory.used",
              "--format=csv,noheader,nounits",
          ],
          check=True,
          capture_output=True,
          text=True,
          timeout=2.0,
      )
    except (FileNotFoundError, subprocess.SubprocessError):
      return None, None

    gpu_utils = []
    gpu_mem = []
    for line in result.stdout.splitlines():
      parts = [p.strip() for p in line.split(",")]
      if len(parts) >= 2:
        try:
          gpu_utils.append(float(parts[0]))
          gpu_mem.append(float(parts[1]))
        except ValueError:
          continue
    if not gpu_utils:
      return None, None
    return max(gpu_utils), max(gpu_mem)

  def _query_pgid_gpu(self, pids):
    pid_set = {int(pid) for pid in pids}
    if not pid_set:
      return None, None, None

    gpu_mem_by_pid = {}
    try:
      result = subprocess.run(
          [
              "nvidia-smi",
              "--query-compute-apps=pid,used_memory",
              "--format=csv,noheader,nounits",
          ],
          check=True,
          capture_output=True,
          text=True,
          timeout=2.0,
      )
      for line in result.stdout.splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
          continue
        try:
          pid = int(parts[0])
          mem = float(parts[1])
        except ValueError:
          continue
        if pid in pid_set:
          gpu_mem_by_pid[pid] = gpu_mem_by_pid.get(pid, 0.0) + mem
    except (FileNotFoundError, subprocess.SubprocessError):
      pass

    gpu_util_by_pid = {}
    try:
      result = subprocess.run(
          ["nvidia-smi", "pmon", "-c", "1", "-s", "um"],
          check=True,
          capture_output=True,
          text=True,
          timeout=3.0,
      )
      for line in result.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
          continue
        parts = line.split()
        if len(parts) < 4:
          continue
        try:
          pid = int(parts[1])
          sm_util = 0.0 if parts[3] == "-" else float(parts[3])
        except ValueError:
          continue
        if pid in pid_set:
          gpu_util_by_pid[pid] = gpu_util_by_pid.get(pid, 0.0) + sm_util
    except (FileNotFoundError, subprocess.SubprocessError):
      pass

    gpu_mem = sum(gpu_mem_by_pid.values())
    gpu_total_mem = None
    try:
      result = subprocess.run(
          [
              "nvidia-smi",
              "--query-gpu=memory.total",
              "--format=csv,noheader,nounits",
          ],
          check=True,
          capture_output=True,
          text=True,
          timeout=2.0,
      )
      totals = []
      for line in result.stdout.splitlines():
        try:
          totals.append(float(line.strip()))
        except ValueError:
          continue
      gpu_total_mem = sum(totals) if totals else None
    except (FileNotFoundError, subprocess.SubprocessError):
      pass
    gpu_mem_percent = (
        gpu_mem / gpu_total_mem * 100.0 if gpu_total_mem else None
    )
    gpu_util = sum(gpu_util_by_pid.values()) if gpu_util_by_pid else None
    return gpu_util, gpu_mem, gpu_mem_percent

  def _run(self):
    while not self._stop_event.is_set():
      vm = psutil.virtual_memory()
      pgid_pids = self._pids_in_recorded_pgid()
      pgid_stats = self._pgid_process_stats(pgid_pids)
      gpu_util, gpu_mem = self._query_gpu()
      pgid_gpu_util, pgid_gpu_mem, pgid_gpu_mem_percent = (
          self._query_pgid_gpu(pgid_pids)
      )
      sample = {
          "time": time.time(),
          "system_cpu_percent": psutil.cpu_percent(interval=None),
          "system_ram_percent": float(vm.percent),
          "system_ram_used_mb": float(vm.used / 1024**2),
          "process_cpu_percent": self._process.cpu_percent(interval=None),
          "process_rss_mb": float(self._process.memory_info().rss / 1024**2),
          "gpu_util_percent": gpu_util,
          "gpu_mem_used_mb": gpu_mem,
          "pgid_pids": pgid_pids,
          "pgid_gpu_util_percent": pgid_gpu_util,
          "pgid_gpu_mem_used_mb": pgid_gpu_mem,
          "pgid_gpu_mem_percent": pgid_gpu_mem_percent,
      }
      sample.update(pgid_stats)
      sample["normalized_process_cpu_percent"] = (
          sample["process_cpu_percent"] / self._nproc
      )
      self._samples.append(sample)
      self._summary["sample_count"] = len(self._samples)
      for key in (
          "system_cpu_percent",
          "system_ram_percent",
          "system_ram_used_mb",
          "process_cpu_percent",
          "normalized_process_cpu_percent",
          "process_rss_mb",
          "pgid_cpu_percent",
          "normalized_pgid_cpu_percent",
          "pgid_rss_mb",
          "pgid_rss_percent",
      ):
        self._summary[f"max_{key}"] = max(
            self._summary[f"max_{key}"], sample[key]
        )
      if gpu_util is not None:
        self._summary["max_gpu_util_percent"] = max(
            self._summary["max_gpu_util_percent"] or 0.0, gpu_util
        )
      if gpu_mem is not None:
        self._summary["max_gpu_mem_used_mb"] = max(
            self._summary["max_gpu_mem_used_mb"] or 0.0, gpu_mem
        )
      if pgid_gpu_util is not None:
        self._summary["max_pgid_gpu_util_percent"] = max(
            self._summary["max_pgid_gpu_util_percent"] or 0.0, pgid_gpu_util
        )
      if pgid_gpu_mem is not None:
        self._summary["max_pgid_gpu_mem_used_mb"] = max(
            self._summary["max_pgid_gpu_mem_used_mb"] or 0.0, pgid_gpu_mem
        )
      if pgid_gpu_mem_percent is not None:
        self._summary["max_pgid_gpu_mem_percent"] = max(
            self._summary["max_pgid_gpu_mem_percent"] or 0.0,
            pgid_gpu_mem_percent,
        )
      exceeded = self._check_limits(sample)
      if exceeded:
        self._record_limit_event(sample, exceeded)
        if self._kill_on_exceed:
          self._terminate_recorded_pgid()
        self._stop_event.set()
        break
      self._stop_event.wait(self._interval_sec)

  def _check_limits(self, sample):
    exceeded = {}
    for key, limit in self._limits.items():
      value = sample.get(key)
      if limit > 0 and value is not None and value >= limit:
        exceeded[key] = {"value": value, "limit": limit}
    return exceeded

  def _record_limit_event(self, sample, exceeded):
    self._limit_event = {
        "time": sample["time"],
        "timestamp": datetime.datetime.now().isoformat(),
        "recorded_pid": self._pid,
        "recorded_pgid": self._recorded_pgid,
        "exceeded": exceeded,
        "kill_on_resource_exceed": self._kill_on_exceed,
    }
    self._summary["resource_limit_exceeded"] = True
    self._summary["resource_limit_event"] = self._limit_event
    event_path = self._logdir / "resource_limit_exceeded.json"
    with open(event_path, "w", encoding="utf-8") as fp:
      json.dump(self._limit_event, fp, indent=2, ensure_ascii=False)

  def _terminate_recorded_pgid(self):
    print(
        "Resource threshold exceeded. "
        f"Terminating recorded PGID {self._recorded_pgid}.",
        flush=True,
    )
    try:
      os.killpg(self._recorded_pgid, signal.SIGTERM)
    except ProcessLookupError:
      return
    except PermissionError as exc:
      print(f"Could not terminate PGID {self._recorded_pgid}: {exc}", flush=True)

  def write(self, logdir, training_seconds: float):
    summary = dict(self._summary)
    summary["training_seconds"] = training_seconds
    with open(logdir / "resource_samples.jsonl", "w", encoding="utf-8") as fp:
      for sample in self._samples:
        fp.write(json.dumps(sample, sort_keys=True) + "\n")
    with open(logdir / "resource_summary.json", "w", encoding="utf-8") as fp:
      json.dump(summary, fp, indent=2)

    lines = [
        "Resource usage summary",
        f"training_seconds: {summary['training_seconds']}",
        f"sample_count: {summary['sample_count']}",
        f"max_system_cpu_percent: {summary['max_system_cpu_percent']}",
        f"max_process_cpu_percent: {summary['max_process_cpu_percent']}",
        "max_normalized_process_cpu_percent: "
        f"{summary['max_normalized_process_cpu_percent']}",
        f"max_pgid_cpu_percent: {summary['max_pgid_cpu_percent']}",
        "max_normalized_pgid_cpu_percent: "
        f"{summary['max_normalized_pgid_cpu_percent']}",
        f"max_system_ram_percent: {summary['max_system_ram_percent']}",
        f"max_system_ram_used_mb: {summary['max_system_ram_used_mb']}",
        f"max_process_rss_mb: {summary['max_process_rss_mb']}",
        f"max_pgid_rss_mb: {summary['max_pgid_rss_mb']}",
        f"max_pgid_rss_percent: {summary['max_pgid_rss_percent']}",
        f"max_gpu_util_percent: {summary['max_gpu_util_percent']}",
        f"max_gpu_mem_used_mb: {summary['max_gpu_mem_used_mb']}",
        f"max_pgid_gpu_util_percent: {summary['max_pgid_gpu_util_percent']}",
        f"max_pgid_gpu_mem_used_mb: {summary['max_pgid_gpu_mem_used_mb']}",
        f"max_pgid_gpu_mem_percent: {summary['max_pgid_gpu_mem_percent']}",
        f"recorded_pid: {summary['recorded_pid']}",
        f"recorded_pgid: {summary['recorded_pgid']}",
        f"resource_limits: {summary['resource_limits']}",
        f"kill_on_resource_exceed: {summary['kill_on_resource_exceed']}",
        f"resource_limit_exceeded: {summary['resource_limit_exceeded']}",
    ]
    with open(logdir / "resource_summary.txt", "w", encoding="utf-8") as fp:
      fp.write("\n".join(lines) + "\n")
    return summary


def _metric_to_float(value):
  value = jax.device_get(value)
  try:
    return float(value)
  except (TypeError, ValueError):
    return value


def _patch_sac_checkpoint_restore():
  """Replicates restored SAC params to match Brax's pmapped train state."""
  original_load = sac.checkpoint.load

  def load_replicated(path):
    params = original_load(path)
    return jax.device_put_replicated(params, jax.local_devices())

  sac.checkpoint.load = load_replicated


def _make_sac_train_with_replay_buffer_io(demo_namespace=None,
                                          nstep_namespace=None):
  """Returns a SAC train function with replay buffer save/load hooks.

  Brax's public SAC API restores network params only.  This wrapper keeps the
  upstream implementation intact and adds two optional hooks around the internal
  replay buffer state: load right after `replay_buffer.init`, and save just
  before the function returns.

  demo_namespace 가 주어지면 sac_demo_mix 의 A1(시연 전용 버퍼 혼합) 패치도
  함께 적용한다.
  """
  source = inspect.getsource(sac.train)
  source = source.replace("def train(\n", "def train_with_replay_buffer_io(\n", 1)
  source = source.replace(
      "    restore_checkpoint_path: Optional[str] = None,\n):",
      "    restore_checkpoint_path: Optional[str] = None,\n"
      "    replay_buffer_load_path: Optional[str] = None,\n"
      "    replay_buffer_save_path: Optional[str] = None,\n"
      "):",
      1,
  )
  source = source.replace(
      "  # Replay buffer init\n"
      "  buffer_state = jax.pmap(replay_buffer.init)(\n"
      "      jax.random.split(rb_key, local_devices_to_use)\n"
      "  )\n",
      "  # Replay buffer init\n"
      "  buffer_state = jax.pmap(replay_buffer.init)(\n"
      "      jax.random.split(rb_key, local_devices_to_use)\n"
      "  )\n"
      "\n"
      "  if replay_buffer_load_path is not None:\n"
      "    replay_buffer_load_path = epath.Path(replay_buffer_load_path)\n"
      "    if replay_buffer_load_path.exists():\n"
      "      logging.info('loading replay buffer from %s', replay_buffer_load_path)\n"
      "      buffer_state = ocp.PyTreeCheckpointer().restore(\n"
      "          replay_buffer_load_path, item=None\n"
      "      )\n"
      "      if isinstance(buffer_state, dict):\n"
      "        buffer_state = ReplayBufferState(**buffer_state)\n"
      "    else:\n"
      "      logging.warning(\n"
      "          'replay buffer load path does not exist: %s',\n"
      "          replay_buffer_load_path,\n"
      "      )\n",
      1,
  )
  source = source.replace(
      "  pmap.synchronize_hosts()\n"
      "  return (make_policy, params, metrics)\n",
      "  if replay_buffer_save_path is not None and process_id == 0:\n"
      "    replay_buffer_save_path = epath.Path(replay_buffer_save_path)\n"
      "    replay_buffer_save_path.parent.mkdir(parents=True, exist_ok=True)\n"
      "    logging.info('saving replay buffer to %s', replay_buffer_save_path)\n"
      "    ocp.PyTreeCheckpointer().save(\n"
      "        replay_buffer_save_path,\n"
      "        jax.device_get(buffer_state),\n"
      "        force=True,\n"
      "    )\n"
      "\n"
      "  pmap.synchronize_hosts()\n"
      "  return (make_policy, params, metrics)\n",
      1,
  )

  if demo_namespace is not None:
    source = sac_demo_mix.patch_source(source)

  if nstep_namespace is not None:
    source = sac_nstep.patch_source(source)

  namespace = dict(sac.__dict__)
  namespace.update({
      "epath": epath,
      "jax": jax,
      "ocp": ocp,
      "ReplayBufferState": replay_buffers.ReplayBufferState,
  })
  if demo_namespace is not None:
    namespace.update(demo_namespace)
  if nstep_namespace is not None:
    namespace.update(nstep_namespace)
  exec(source, namespace)  # pylint: disable=exec-used
  return namespace["train_with_replay_buffer_io"]


def _write_metric_analysis(logdir, metric_rows):
  eval_rows = [r for r in metric_rows if "eval/episode_reward" in r]
  critic_rows = [r for r in metric_rows if "training/critic_loss" in r]
  actor_rows = [r for r in metric_rows if "training/actor_loss" in r]
  alpha_rows = [r for r in metric_rows if "training/alpha" in r]

  def values(rows, key):
    return [float(r[key]) for r in rows if key in r]

  eval_rewards = values(eval_rows, "eval/episode_reward")
  critic_losses = values(critic_rows, "training/critic_loss")
  actor_losses = values(actor_rows, "training/actor_loss")
  alphas = values(alpha_rows, "training/alpha")
  success_counts = values(eval_rows, "eval/episode_success_count")
  success_holds = values(eval_rows, "eval/episode_success_hold_steps")
  avg_episode_lengths = values(eval_rows, "eval/avg_episode_length")
  orientation_rewards = values(eval_rows, "eval/episode_reward/orientation")
  hand_pose_rewards = values(eval_rows, "eval/episode_reward/hand_pose")
  action_rate_rewards = values(eval_rows, "eval/episode_reward/action_rate")
  termination_rewards = values(eval_rows, "eval/episode_reward/termination")
  orientation_errors = values(eval_rows, "eval/episode_orientation_error")
  ori_lt_060 = values(eval_rows, "eval/episode_orientation_error_lt_0_60")
  ori_lt_040 = values(eval_rows, "eval/episode_orientation_error_lt_0_40")
  ori_lt_025 = values(eval_rows, "eval/episode_orientation_error_lt_0_25")
  ori_lt_020 = values(eval_rows, "eval/episode_orientation_error_lt_0_20")
  ori_lt_015 = values(eval_rows, "eval/episode_orientation_error_lt_0_15")
  ori_lt_012 = values(eval_rows, "eval/episode_orientation_error_lt_0_12")
  ori_lt_010 = values(eval_rows, "eval/episode_orientation_error_lt_0_10")

  def trend(xs):
    if len(xs) < 2:
      return 0.0
    return xs[-1] - xs[0]

  recommendations = []
  if success_counts and max(success_counts) <= 0:
    recommendations.append(
        "No successful eval episodes yet: do not change success_reward first; "
        "increase exploration or ease motion constraints before tuning the bonus."
    )
  if alphas and alphas[-1] < 0.05:
    recommendations.append(
        "Entropy alpha collapsed below 0.05: consider increasing exploration "
        "by using a larger initial entropy/noise setting or a custom target entropy."
    )
  if action_rate_rewards and abs(action_rate_rewards[-1]) > abs(action_rate_rewards[0]) * 2:
    recommendations.append(
        "Action-rate penalty grew during training: avoid increasing "
        "action_rate penalty until success appears."
    )
  if hand_pose_rewards and abs(hand_pose_rewards[-1]) > 4000:
    recommendations.append(
        "Hand-pose penalty dominates the reward: consider reducing hand_pose "
        "scale from -0.5 to about -0.2 after checking rollout behavior."
    )
  if critic_losses and max(critic_losses) > 5 * max(min(critic_losses), 1e-6):
    recommendations.append(
        "Critic Q-loss is spiky: consider lowering learning_rate to 5e-5, "
        "shortening each stage, or keeping grad_updates_per_step at 1."
    )
  if eval_rewards and len(eval_rewards) >= 3 and eval_rewards[-1] < max(eval_rewards):
    recommendations.append(
        "Final reward is below the best checkpoint: evaluate the best checkpoint "
        "instead of only the final one before changing rewards."
    )
  if success_counts and max(success_counts) <= 0 and ori_lt_010 and max(ori_lt_010) > 0:
    recommendations.append(
        "Orientation sometimes reaches the final threshold, but success is still "
        "zero: reduce success_hold_steps or add a hold-progress reward."
    )
  elif success_counts and max(success_counts) <= 0 and ori_lt_012 and max(ori_lt_012) > 0:
    recommendations.append(
        "Orientation reaches 0.12 rad but not stable final success: add an "
        "intermediate 0.12 stage before the 0.10 final stage."
    )
  elif success_counts and max(success_counts) <= 0 and ori_lt_015 and max(ori_lt_015) > 0:
    recommendations.append(
        "Orientation reaches 0.15 rad but not the final threshold: use a longer "
        "0.15 stage or lower late-stage learning_rate."
    )
  elif success_counts and max(success_counts) <= 0 and ori_lt_020 and max(ori_lt_020) > 0:
    recommendations.append(
        "Orientation reaches 0.20 rad but not tighter thresholds: add 0.20 and "
        "0.15 curriculum stages before final success tuning."
    )

  summary = {
      "num_metric_rows": len(metric_rows),
      "eval_reward": {
          "count": len(eval_rewards),
          "first": eval_rewards[0] if eval_rewards else None,
          "last": eval_rewards[-1] if eval_rewards else None,
          "best": max(eval_rewards) if eval_rewards else None,
          "trend_last_minus_first": trend(eval_rewards),
      },
      "critic_loss": {
          "count": len(critic_losses),
          "first": critic_losses[0] if critic_losses else None,
          "last": critic_losses[-1] if critic_losses else None,
          "min": min(critic_losses) if critic_losses else None,
          "max": max(critic_losses) if critic_losses else None,
          "trend_last_minus_first": trend(critic_losses),
      },
      "actor_loss": {
          "count": len(actor_losses),
          "first": actor_losses[0] if actor_losses else None,
          "last": actor_losses[-1] if actor_losses else None,
      },
      "alpha": {
          "count": len(alphas),
          "first": alphas[0] if alphas else None,
          "last": alphas[-1] if alphas else None,
      },
      "success": {
          "max_success_count": max(success_counts) if success_counts else None,
          "max_success_hold_steps": max(success_holds) if success_holds else None,
      },
      "orientation_error": {
          "count": len(orientation_errors),
          "first": orientation_errors[0] if orientation_errors else None,
          "last": orientation_errors[-1] if orientation_errors else None,
          "best_min": min(orientation_errors) if orientation_errors else None,
          "trend_last_minus_first": trend(orientation_errors),
          "threshold_hit_rates": {
              "lt_0_40_best": max(ori_lt_040) if ori_lt_040 else None,
              "lt_0_60_best": max(ori_lt_060) if ori_lt_060 else None,
              "lt_0_25_best": max(ori_lt_025) if ori_lt_025 else None,
              "lt_0_20_best": max(ori_lt_020) if ori_lt_020 else None,
              "lt_0_15_best": max(ori_lt_015) if ori_lt_015 else None,
              "lt_0_12_best": max(ori_lt_012) if ori_lt_012 else None,
              "lt_0_10_best": max(ori_lt_010) if ori_lt_010 else None,
          },
      },
      "eval_components": {
          "avg_episode_length_last": (
              avg_episode_lengths[-1] if avg_episode_lengths else None
          ),
          "orientation_last": (
              orientation_rewards[-1] if orientation_rewards else None
          ),
          "hand_pose_last": hand_pose_rewards[-1] if hand_pose_rewards else None,
          "action_rate_last": (
              action_rate_rewards[-1] if action_rate_rewards else None
          ),
          "termination_last": (
              termination_rewards[-1] if termination_rewards else None
          ),
      },
      "recommendations": recommendations,
  }

  with open(logdir / "analysis_summary.json", "w", encoding="utf-8") as fp:
    json.dump(summary, fp, indent=2)

  lines = [
      "SAC reward / Q-loss analysis",
      f"metric rows: {summary['num_metric_rows']}",
      (
          "eval reward: "
          f"first={summary['eval_reward']['first']}, "
          f"last={summary['eval_reward']['last']}, "
          f"best={summary['eval_reward']['best']}, "
          f"trend={summary['eval_reward']['trend_last_minus_first']}"
      ),
      (
          "critic Q-loss: "
          f"first={summary['critic_loss']['first']}, "
          f"last={summary['critic_loss']['last']}, "
          f"min={summary['critic_loss']['min']}, "
          f"max={summary['critic_loss']['max']}, "
          f"trend={summary['critic_loss']['trend_last_minus_first']}"
      ),
      (
          "actor loss: "
          f"first={summary['actor_loss']['first']}, "
          f"last={summary['actor_loss']['last']}"
      ),
      (
          "alpha: "
          f"first={summary['alpha']['first']}, "
          f"last={summary['alpha']['last']}"
      ),
      (
          "success: "
          f"max_count={summary['success']['max_success_count']}, "
          f"max_hold_steps={summary['success']['max_success_hold_steps']}"
      ),
      (
          "orientation error: "
          f"first={summary['orientation_error']['first']}, "
          f"last={summary['orientation_error']['last']}, "
          f"best_min={summary['orientation_error']['best_min']}, "
          f"trend={summary['orientation_error']['trend_last_minus_first']}"
      ),
      (
          "orientation threshold hit rates: "
          f"{summary['orientation_error']['threshold_hit_rates']}"
      ),
      "recommendations:",
      *[f"- {r}" for r in recommendations],
  ]
  with open(logdir / "analysis_summary.txt", "w", encoding="utf-8") as fp:
    fp.write("\n".join(lines) + "\n")

  return summary


def _write_result_markdown(
    logdir,
    exp_name: str,
    sac_params,
    env_cfg_overrides,
    analysis,
    resource_summary,
    videos,
):
  resource_summary = resource_summary or {}
  eval_reward = analysis.get("eval_reward", {}) if analysis else {}
  critic_loss = analysis.get("critic_loss", {}) if analysis else {}
  success = analysis.get("success", {}) if analysis else {}
  orientation_error = analysis.get("orientation_error", {}) if analysis else {}
  threshold_rates = orientation_error.get("threshold_hit_rates", {})
  recommendations = analysis.get("recommendations", []) if analysis else []

  lines = [
      "# SAC Training Result",
      "",
      f"- experiment: `{exp_name}`",
      f"- logdir: `{logdir}`",
      f"- env_overrides: `{env_cfg_overrides}`",
      "",
      "## Training Settings",
      "",
      f"- num_timesteps: `{sac_params.num_timesteps}`",
      f"- episode_length: `{sac_params.episode_length}`",
      f"- num_envs: `{sac_params.num_envs}`",
      f"- num_eval_envs: `{sac_params.num_eval_envs}`",
      f"- batch_size: `{sac_params.batch_size}`",
      f"- min_replay_size: `{sac_params.min_replay_size}`",
      f"- learning_rate: `{sac_params.learning_rate}`",
      f"- grad_updates_per_step: `{sac_params.grad_updates_per_step}`",
      f"- hidden_layer_sizes: `{tuple(sac_params.network_factory.hidden_layer_sizes)}`",
      "",
      "## Metrics",
      "",
      f"- eval_reward_first: `{eval_reward.get('first')}`",
      f"- eval_reward_last: `{eval_reward.get('last')}`",
      f"- eval_reward_best: `{eval_reward.get('best')}`",
      f"- critic_loss_first: `{critic_loss.get('first')}`",
      f"- critic_loss_last: `{critic_loss.get('last')}`",
      f"- success_count_max: `{success.get('max_success_count')}`",
      f"- success_hold_steps_max: `{success.get('max_success_hold_steps')}`",
      f"- orientation_error_best_min_accumulated: `{orientation_error.get('best_min')}`",
      f"- orientation_threshold_hit_rates_accumulated: `{threshold_rates}`",
      "",
      "## Resources",
      "",
      f"- training_seconds: `{resource_summary.get('training_seconds')}`",
      f"- max_system_cpu_percent: `{resource_summary.get('max_system_cpu_percent')}`",
      f"- max_process_cpu_percent: `{resource_summary.get('max_process_cpu_percent')}`",
      "- max_normalized_process_cpu_percent: "
      f"`{resource_summary.get('max_normalized_process_cpu_percent')}`",
      f"- max_pgid_cpu_percent: `{resource_summary.get('max_pgid_cpu_percent')}`",
      "- max_normalized_pgid_cpu_percent: "
      f"`{resource_summary.get('max_normalized_pgid_cpu_percent')}`",
      f"- max_system_ram_percent: `{resource_summary.get('max_system_ram_percent')}`",
      f"- max_pgid_rss_mb: `{resource_summary.get('max_pgid_rss_mb')}`",
      f"- max_pgid_rss_percent: `{resource_summary.get('max_pgid_rss_percent')}`",
      f"- max_gpu_util_percent: `{resource_summary.get('max_gpu_util_percent')}`",
      f"- max_gpu_mem_used_mb: `{resource_summary.get('max_gpu_mem_used_mb')}`",
      "- max_pgid_gpu_util_percent: "
      f"`{resource_summary.get('max_pgid_gpu_util_percent')}`",
      "- max_pgid_gpu_mem_used_mb: "
      f"`{resource_summary.get('max_pgid_gpu_mem_used_mb')}`",
      "- max_pgid_gpu_mem_percent: "
      f"`{resource_summary.get('max_pgid_gpu_mem_percent')}`",
      f"- recorded_pid: `{resource_summary.get('recorded_pid')}`",
      f"- recorded_pgid: `{resource_summary.get('recorded_pgid')}`",
      f"- resource_limits: `{resource_summary.get('resource_limits')}`",
      "- kill_on_resource_exceed: "
      f"`{resource_summary.get('kill_on_resource_exceed')}`",
      "- resource_limit_exceeded: "
      f"`{resource_summary.get('resource_limit_exceeded')}`",
      "",
      "## Videos",
      "",
      *[f"- `{video}`" for video in videos],
      "",
      "## Recommendations",
      "",
      *([f"- {item}" for item in recommendations] or ["- No recommendation."]),
      "",
  ]
  with open(logdir / "result.md", "w", encoding="utf-8") as fp:
    fp.write("\n".join(lines))


class ObsKeyWrapper(wrapper.Wrapper):
  """dict 관측에서 한 키만 골라 SAC 에 넘긴다.

  주의: 골라낸 뒤 `state.obs` 를 덮어쓰므로, **안쪽** 래퍼는 다음 step 에서
  dict 관측을 볼 수 없게 된다. 대부분의 env 는 step 에서 state.obs 를 쓰지 않아
  드러나지 않지만, 교사 정책을 호출하는 residual 래퍼처럼 obs 가 필요한 래퍼를
  안쪽에 두면 깨진다(교사 정규화가 dict 구조 전체를 요구한다).
  그래서 원본 dict 를 info 에 보관하고 안쪽으로 내려갈 때 복원한다.
  """

  _FULL = "ObsKeyWrapper_full_obs"

  def __init__(self, env, obs_key="state"):
    super().__init__(env)
    self._obs_key = obs_key

  def _pack(self, state):
    if isinstance(state.obs, dict):
      state.info[self._FULL] = state.obs
      return state.replace(obs=state.obs[self._obs_key])
    return state

  def reset(self, rng):
    return self._pack(self.env.reset(rng))

  def step(self, state, action):
    inner = state
    if self._FULL in state.info:
      inner = state.replace(obs=state.info[self._FULL])
    return self._pack(self.env.step(inner, action))

  @property
  def observation_size(self):
    obs_size = self.env.observation_size
    if isinstance(obs_size, dict):
      return obs_size[self._obs_key][0]
    return obs_size


def wrap_for_sac_training(*args, **kwargs):
  base = args[0]
  if _RESIDUAL_TEACHER.value:
    # 교사(PPO)는 dict 관측을 받으므로 평탄화(ObsKeyWrapper) **아래**에 놓는다.
    from brax.training import networks as _bn
    from brax.training.agents.ppo import checkpoint as _pc
    _bn.KERNEL_INITIALIZER.setdefault(None, None)
    tfn = _pc.load_policy(_RESIDUAL_TEACHER.value, deterministic=True)
    base = residual_wrapper.ResidualTeacherWrapper(
        base, tfn, _RESIDUAL_SCALE.value, _RESIDUAL_L2.value)
  env = ObsKeyWrapper(base, _OBS_KEY.value)
  kwargs.setdefault("full_reset", _FULL_RESET.value)
  return wrapper.wrap_for_brax_training(env, *args[1:], **kwargs)


def get_rl_config(env_name: str) -> config_dict.ConfigDict:
  if env_name in mujoco_playground.manipulation._envs:
    return manipulation_params.brax_sac_config(env_name, _IMPL.value)
  raise ValueError(f"SAC is configured only for manipulation envs: {env_name}")


def main(argv):
  del argv

  env_cfg = registry.get_default_config(_ENV_NAME.value)
  sac_params = get_rl_config(_ENV_NAME.value)

  if _NUM_TIMESTEPS.present:
    sac_params.num_timesteps = _NUM_TIMESTEPS.value
  if _PLAY_ONLY.present:
    sac_params.num_timesteps = 0
  if _NUM_EVALS.present:
    sac_params.num_evals = _NUM_EVALS.value
  if _REWARD_SCALING.present:
    sac_params.reward_scaling = _REWARD_SCALING.value
  if _EPISODE_LENGTH.present:
    sac_params.episode_length = _EPISODE_LENGTH.value
  if _NORMALIZE_OBSERVATIONS.present:
    sac_params.normalize_observations = _NORMALIZE_OBSERVATIONS.value
  if _ACTION_REPEAT.present:
    sac_params.action_repeat = _ACTION_REPEAT.value
  if _DISCOUNTING.present:
    sac_params.discounting = _DISCOUNTING.value
  if _LEARNING_RATE.present:
    sac_params.learning_rate = _LEARNING_RATE.value
  if _TAU.present:
    sac_params.tau = _TAU.value
  if _NUM_ENVS.present:
    sac_params.num_envs = _NUM_ENVS.value
  if _NUM_EVAL_ENVS.present:
    sac_params.num_eval_envs = _NUM_EVAL_ENVS.value
  if _BATCH_SIZE.present:
    sac_params.batch_size = _BATCH_SIZE.value
  if _MIN_REPLAY_SIZE.present:
    sac_params.min_replay_size = _MIN_REPLAY_SIZE.value
  if _MAX_REPLAY_SIZE.present:
    sac_params.max_replay_size = _MAX_REPLAY_SIZE.value
  if _GRAD_UPDATES_PER_STEP.present:
    sac_params.grad_updates_per_step = _GRAD_UPDATES_PER_STEP.value
  if _DETERMINISTIC_EVAL.present:
    sac_params.deterministic_eval = _DETERMINISTIC_EVAL.value
  if _HIDDEN_LAYER_SIZES.present:
    sac_params.network_factory.hidden_layer_sizes = list(
        map(int, _HIDDEN_LAYER_SIZES.value)
    )
  if _Q_LAYER_NORM.value is not None:
    sac_params.network_factory.q_network_layer_norm = _Q_LAYER_NORM.value
  if _INIT_NOISE_STD.value is not None:
    if _LOAD_CHECKPOINT_PATH.value is not None:
      print(
          "경고: --init_noise_std 는 --load_checkpoint_path 와 함께 쓰면 "
          "무의미하다. 체크포인트의 정책 파라미터가 std 를 덮어쓴다."
      )
    sac_params.network_factory.init_noise_std = _INIT_NOISE_STD.value

  env_cfg_overrides = {"impl": _IMPL.value}
  if _PLAYGROUND_CONFIG_OVERRIDES.value is not None:
    env_cfg_overrides.update(json.loads(_PLAYGROUND_CONFIG_OVERRIDES.value))

  env = registry.load(
      _ENV_NAME.value, config=env_cfg, config_overrides=env_cfg_overrides
  )

  print(f"Environment Config:\n{env_cfg}")
  print(f"Environment Config Overrides:\n{env_cfg_overrides}\n")
  print(f"SAC Training Parameters:\n{sac_params}")
  print(f"SAC observation key: {_OBS_KEY.value}")

  timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
  exp_name = f"{_ENV_NAME.value}-SAC-{timestamp}"
  if _SUFFIX.value is not None:
    exp_name += f"-{_SUFFIX.value}"

  logdir = epath.Path(_LOGDIR.value or "logs").resolve() / exp_name
  logdir.mkdir(parents=True, exist_ok=True)
  ckpt_path = logdir / "checkpoints"
  ckpt_path.mkdir(parents=True, exist_ok=True)
  print(f"Experiment name: {exp_name}")
  print(f"Logs are being stored in: {logdir}")
  print(f"Checkpoint path: {ckpt_path}")

  if _USE_WANDB.value and not _PLAY_ONLY.value:
    if wandb is None:
      raise ImportError("wandb is required for --use_wandb.")
    wandb.init(project="mjxrl", name=exp_name)
    wandb.config.update(env_cfg.to_dict())
    wandb.config.update({"env_name": _ENV_NAME.value, "algo": "sac"})

  writer = None
  if _USE_TB.value and not _PLAY_ONLY.value and tensorboardX is not None:
    writer = tensorboardX.SummaryWriter(logdir)

  with open(ckpt_path / "config.json", "w", encoding="utf-8") as fp:
    json.dump(env_cfg.to_dict(), fp, indent=4)

  restore_checkpoint_path = None
  if _LOAD_CHECKPOINT_PATH.value is not None:
    restore_checkpoint_path = epath.Path(_LOAD_CHECKPOINT_PATH.value).resolve()
    print(f"Restoring from checkpoint: {restore_checkpoint_path}")
    if not _PLAY_ONLY.value:
      _patch_sac_checkpoint_restore()
  else:
    print("No checkpoint path provided, not restoring from checkpoint")

  training_params = dict(sac_params)
  network_factory_params = training_params.pop("network_factory", {})
  num_eval_envs = training_params.pop("num_eval_envs", 128)
  training_params.pop("num_resets_per_eval", None)

  network_factory = functools.partial(
      sac_networks.make_sac_networks, **network_factory_params
  )
  if _ASYMMETRIC_CRITIC.value or _N_CRITICS.value != 2:
    asym_params = dict(network_factory_params)
    asym_params.pop("policy_network_layer_norm", None)
    asym_params["n_critics"] = _N_CRITICS.value
    if _ASYMMETRIC_CRITIC.value:
      # policy 는 'state' 만, critic 은 관측 전체를 본다. reorient 의
      # privileged_state 는 state 를 앞에 그대로 붙여 만들므로 앞부분만 자르면 된다.
      if _OBS_KEY.value != "privileged_state":
        raise ValueError(
            "--asymmetric_critic 은 --obs_key privileged_state 와 함께 써야 한다 "
            f"(현재 {_OBS_KEY.value})."
        )
      asym_params["policy_obs_size"] = int(env.observation_size["state"][0])
      print(
          f"Asymmetric critic (E1): policy obs {asym_params['policy_obs_size']} "
          f"/ critic obs {int(env.observation_size['privileged_state'][0])}"
      )
    print(f"n_critics: {_N_CRITICS.value}")
    network_factory = functools.partial(
        sac_asym_networks.make_asym_sac_networks, **asym_params
    )

  if _SIMBA.value:
    if _ASYMMETRIC_CRITIC.value:
      raise ValueError("--simba 와 --asymmetric_critic 은 함께 쓸 수 없다.")
    simba_params = dict(
        hidden_dim=_SIMBA_HIDDEN.value,
        policy_blocks=_SIMBA_POLICY_BLOCKS.value,
        critic_blocks=_SIMBA_CRITIC_BLOCKS.value,
        n_critics=_N_CRITICS.value,
    )
    if _SIMBA_CRITIC_HIDDEN.value > 0:
      simba_params["critic_hidden_dim"] = _SIMBA_CRITIC_HIDDEN.value
    print(f"SimBa 네트워크: {simba_params}")
    network_factory = functools.partial(
        sac_simba_networks.make_simba_sac_networks, **simba_params
    )

  if _DOMAIN_RANDOMIZATION.value:
    training_params["randomization_fn"] = registry.get_domain_randomizer(
        _ENV_NAME.value
    )

  sac_train_fn = sac.train
  replay_buffer_load_path = None
  replay_buffer_save_path = None
  if _LOAD_REPLAY_BUFFER_PATH.value is not None:
    replay_buffer_load_path = epath.Path(_LOAD_REPLAY_BUFFER_PATH.value).resolve()
  if _SAVE_REPLAY_BUFFER_PATH.value is not None:
    replay_buffer_save_path = epath.Path(_SAVE_REPLAY_BUFFER_PATH.value).resolve()
  demo_namespace = None
  if _DEMO_DATA_PATH.value is not None:
    demo_n = int(round(_DEMO_RATIO.value * sac_params.batch_size))
    if not 0 < demo_n < sac_params.batch_size:
      raise ValueError(
          f"demo_ratio={_DEMO_RATIO.value} 로 계산한 배치 내 시연 개수 {demo_n} 가 "
          f"batch_size={sac_params.batch_size} 범위를 벗어난다."
      )
    demo_namespace, demo_shape = sac_demo_mix.build_namespace_entries(
        epath.Path(_DEMO_DATA_PATH.value).resolve(), demo_n, _BC_COEF.value,
        _BC_Q_FILTER.value,
    )
    print(
        f"Demo mixing (A1): path={_DEMO_DATA_PATH.value} rows={demo_shape[0]:,} "
        f"batch {demo_n}/{sac_params.batch_size} (ratio {_DEMO_RATIO.value}) "
        f"bc_coef={_BC_COEF.value} q_filter={_BC_Q_FILTER.value}"
    )
  nstep_namespace = None
  if _N_STEP.value > 1:
    nstep_namespace = sac_nstep.build_namespace_entries(
        _N_STEP.value, sac_params.discounting
    )
    print(
        f"n-step 반환: n={_N_STEP.value} (γ={sac_params.discounting}). "
        f"부트스트랩 계수 γ^n={sac_params.discounting ** _N_STEP.value:.4f}. "
        f"겹치지 않는 창이므로 삽입 전이는 env 스텝당 "
        f"{sac_params.num_envs / _N_STEP.value:.0f}개."
    )
  if (
      replay_buffer_load_path is not None
      or replay_buffer_save_path is not None
      or demo_namespace is not None
      or nstep_namespace is not None
  ):
    sac_train_fn = _make_sac_train_with_replay_buffer_io(
        demo_namespace, nstep_namespace
    )
    print(f"Replay buffer load path: {replay_buffer_load_path}")
    print(f"Replay buffer save path: {replay_buffer_save_path}")
  replay_buffer_kwargs = {}
  if sac_train_fn is not sac.train:
    replay_buffer_kwargs = {
        "replay_buffer_load_path": replay_buffer_load_path,
        "replay_buffer_save_path": replay_buffer_save_path,
    }

  train_fn = functools.partial(
      sac_train_fn,
      **training_params,
      network_factory=network_factory,
      seed=_SEED.value,
      restore_checkpoint_path=restore_checkpoint_path,
      **replay_buffer_kwargs,
      checkpoint_logdir=ckpt_path,
      wrap_env_fn=wrap_for_sac_training,
      num_eval_envs=num_eval_envs,
  )

  times = [time.monotonic()]
  metric_rows = []
  metrics_path = logdir / "metrics.jsonl"

  def progress(num_steps, metrics):
    times.append(time.monotonic())
    row = {"num_steps": int(num_steps)}
    row.update({k: _metric_to_float(v) for k, v in metrics.items()})
    metric_rows.append(row)
    with open(metrics_path, "a", encoding="utf-8") as fp:
      fp.write(json.dumps(row, sort_keys=True) + "\n")

    if _USE_WANDB.value and not _PLAY_ONLY.value:
      wandb.log(metrics, step=num_steps)
    if _USE_TB.value and not _PLAY_ONLY.value and writer is not None:
      for key, value in metrics.items():
        writer.add_scalar(key, value, num_steps)
      writer.flush()
    if "eval/episode_reward" in metrics:
      print(f"{num_steps}: reward={metrics['eval/episode_reward']:.3f}")
    elif "training/actor_loss" in metrics:
      print(f"{num_steps}: actor_loss={metrics['training/actor_loss']:.3f}")

  eval_env = registry.load(
      _ENV_NAME.value,
      config=registry.get_default_config(_ENV_NAME.value),
      config_overrides=env_cfg_overrides,
  )

  resource_monitor = ResourceMonitor(
      logdir=logdir,
      interval_sec=_RESOURCE_MONITOR_INTERVAL.value,
      gpu_limit_percent=_RESOURCE_GPU_LIMIT_PERCENT.value,
      ram_limit_percent=_RESOURCE_RAM_LIMIT_PERCENT.value,
      cpu_limit_percent=_RESOURCE_CPU_LIMIT_PERCENT.value,
      kill_on_exceed=_KILL_ON_RESOURCE_EXCEED.value,
  )
  if _PLAY_ONLY.value:
    if restore_checkpoint_path is None:
      raise ValueError("--play_only requires --load_checkpoint_path.")
    wrapped_play_env = wrap_for_sac_training(
        env,
        episode_length=sac_params.episode_length,
        action_repeat=sac_params.get("action_repeat", 1),
    )
    normalize_fn = (
        running_statistics.normalize
        if sac_params.normalize_observations
        else lambda x, y: x
    )
    sac_network = network_factory(
        observation_size=wrapped_play_env.observation_size,
        action_size=wrapped_play_env.action_size,
        preprocess_observations_fn=normalize_fn,
    )
    make_inference_fn = sac_networks.make_inference_fn(sac_network)
    params = sac.checkpoint.load(restore_checkpoint_path)
    analysis = {}
    resource_summary = {}
    print("Loaded checkpoint for play-only inference.")
  else:
    resource_monitor.start()
    train_start = time.monotonic()
    try:
      make_inference_fn, params, _ = train_fn(
          environment=env, progress_fn=progress, eval_env=eval_env
      )
    finally:
      training_seconds = time.monotonic() - train_start
      resource_monitor.stop()
    resource_summary = resource_monitor.write(logdir, training_seconds)

    print("Done training.")
    if len(times) > 1:
      print(f"Time to JIT compile: {times[1] - times[0]}")
      print(f"Time to train: {times[-1] - times[1]}")
    analysis = _write_metric_analysis(logdir, metric_rows)
    print(f"Metrics saved to: {metrics_path}")
    print(f"Analysis saved to: {logdir / 'analysis_summary.txt'}")
    print(f"Analysis summary: {analysis}")
    print(f"Resource summary: {resource_summary}")

  if _NUM_VIDEOS.value <= 0:
    _write_result_markdown(
        logdir,
        exp_name,
        sac_params,
        env_cfg_overrides,
        analysis,
        resource_summary,
        [],
    )
    print(f"Result markdown saved to: {logdir / 'result.md'}")
    print("Skipping rollout video generation because --num_videos <= 0.")
    return

  print("Starting inference...")
  inference_fn = make_inference_fn(params, deterministic=True)
  jit_inference_fn = jax.jit(inference_fn)

  infer_env = registry.load(
      _ENV_NAME.value,
      config=registry.get_default_config(_ENV_NAME.value),
      config_overrides=env_cfg_overrides,
  )
  wrapped_infer_env = wrap_for_sac_training(
      infer_env,
      episode_length=sac_params.episode_length,
      action_repeat=sac_params.get("action_repeat", 1),
  )

  rng = jax.random.split(jax.random.PRNGKey(_SEED.value), _NUM_VIDEOS.value)
  reset_states = jax.jit(wrapped_infer_env.reset)(rng)

  empty_data = reset_states.data.__class__(
      **{k: None for k in reset_states.data.__annotations__}
  )
  empty_traj = reset_states.__class__(
      **{k: None for k in reset_states.__annotations__}
  )
  empty_traj = empty_traj.replace(data=empty_data)

  def step(carry, _):
    state, rng = carry
    rng, act_key = jax.random.split(rng)
    act_keys = jax.random.split(act_key, _NUM_VIDEOS.value)
    act = jax.vmap(jit_inference_fn)(state.obs, act_keys)[0]
    state = wrapped_infer_env.step(state, act)
    traj_data = empty_traj.tree_replace({
        "data.qpos": state.data.qpos,
        "data.qvel": state.data.qvel,
        "data.time": state.data.time,
        "data.ctrl": state.data.ctrl,
        "data.mocap_pos": state.data.mocap_pos,
        "data.mocap_quat": state.data.mocap_quat,
        "data.xfrc_applied": state.data.xfrc_applied,
    })
    return (state, rng), traj_data

  @jax.jit
  def do_rollout(state, rng):
    _, traj = jax.lax.scan(
        step, (state, rng), None, length=sac_params.episode_length
    )
    return traj

  traj_stacked = do_rollout(reset_states, jax.random.PRNGKey(_SEED.value + 1))
  traj_stacked = jax.tree.map(lambda x: jp.moveaxis(x, 0, 1), traj_stacked)
  trajectories = [None] * _NUM_VIDEOS.value
  for i in range(_NUM_VIDEOS.value):
    t = jax.tree.map(lambda x, i=i: x[i], traj_stacked)
    trajectories[i] = [
        jax.tree.map(lambda x, j=j: x[j], t)
        for j in range(sac_params.episode_length)
    ]

  render_every = 1
  fps = _VIDEO_FPS.value
  print(f"FPS for rendering: {fps}")
  scene_option = mujoco.MjvOption()
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
  video_paths = []
  for i, rollout in enumerate(trajectories):
    traj = rollout[::render_every]
    frames = infer_env.render(
        traj, height=480, width=640, scene_option=scene_option
    )
    video_duration = len(traj) * infer_env.dt * render_every
    target_frame_count = max(1, int(round(video_duration * fps)))
    frame_ids = np.linspace(0, len(frames) - 1, target_frame_count)
    frames = [frames[int(round(j))] for j in frame_ids]
    video_path = logdir / f"rollout{i}.mp4"
    media.write_video(video_path, frames, fps=fps)
    video_paths.append(str(video_path))
    print(f"Rollout video saved as '{video_path}'.")

  _write_result_markdown(
      logdir,
      exp_name,
      sac_params,
      env_cfg_overrides,
      analysis,
      resource_summary,
      video_paths,
  )
  print(f"Result markdown saved to: {logdir / 'result.md'}")


def run():
  app.run(main)


if __name__ == "__main__":
  run()
