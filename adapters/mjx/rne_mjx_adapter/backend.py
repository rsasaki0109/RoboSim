"""Task-bound fake and MJX-Warp session implementations."""

from __future__ import annotations

import importlib.util
import math
import platform
import subprocess
import sys
import tomllib
from abc import ABC, abstractmethod
from copy import deepcopy
from pathlib import Path
from typing import Any

from .protocol import (
    ADAPTER_ID,
    BATCH_CHECKPOINT_SCHEMA_VERSION,
    CAPABILITY_REPORT_KIND,
    CAPABILITY_REPORT_SCHEMA_VERSION,
    CONFORMANCE_REPORT_SCHEMA_VERSION,
    SCALE_REPORT_SCHEMA_VERSION,
    MASK64,
    PROTOCOL_SCHEMA_VERSION,
    RUNTIME_ID,
    SEED_STRATEGY,
    SUPPORTED_BATCH_WIDTHS,
    TASK_SPEC_SCHEMA_VERSION,
    ProtocolError,
    canonical_json,
    derive_episode_seed,
    fnv1a64,
    load_json_fixture,
    require_exact_keys,
    require_unsigned,
    validate_bound_task_spec,
)

TASK_ID = "rne.physics.free_fall.mjx.v1"
GRAVITY_M_S2 = -9.81
INITIAL_POSITION_Y_M = 5.0
UNSUPPORTED_FEATURES = [
    "automatic_differentiation",
    "midpoint_implicitfast_integrator",
    "noslip_solver",
    "pgs_solver",
    "plugin_sensors",
]
MAX_REPLAY_OPERATIONS = 100_000


class AcceleratorBackend(ABC):
    """Factory boundary used by the protocol server."""

    @abstractmethod
    def capability_report(self) -> dict[str, Any]:
        """Returns a stable, versioned capability report."""

    @abstractmethod
    def create_session(
        self,
        task_spec: dict[str, Any],
        root_seed: int,
        batch_width: int,
        auto_reset: bool,
    ) -> "FreeFallSession":
        """Creates one validated task session."""


class FakeBackend(AcceleratorBackend):
    """Dependency-free test backend; never selected by the production CLI."""

    def __init__(self, fixtures: Path) -> None:
        self._fixtures = fixtures
        self._runtime_contract = _load_runtime_contract(fixtures.parent / "runtime.toml")

    def capability_report(self) -> dict[str, Any]:
        return _capability_report(
            status="test_only",
            reason_code=None,
            runtime_contract=self._runtime_contract,
            runtime={
                "python_version": platform.python_version(),
                "platform": platform.system().lower(),
                "machine": platform.machine().lower(),
                "jax_version": None,
                "jaxlib_version": None,
                "jax_cuda_plugin_version": None,
                "mujoco_version": None,
                "mujoco_mjx_version": None,
                "warp_version": None,
                "jax_backend": None,
                "jax_devices": [],
                "nvidia_driver_version": None,
            },
        )

    def create_session(
        self,
        task_spec: dict[str, Any],
        root_seed: int,
        batch_width: int,
        auto_reset: bool,
    ) -> "FreeFallSession":
        expected = load_json_fixture(self._fixtures / "free-fall-task-spec-v1.json")
        return FakeFreeFallSession(
            validate_bound_task_spec(task_spec, expected),
            root_seed,
            batch_width,
            auto_reset,
        )


class MjxWarpBackend(AcceleratorBackend):
    """Production backend that imports MJX-Warp only after a protocol request."""

    def __init__(self, fixtures: Path) -> None:
        self._fixtures = fixtures
        self._runtime_contract = _load_runtime_contract(fixtures.parent / "runtime.toml")

    def capability_report(self) -> dict[str, Any]:
        status, reason_code, runtime = _probe_runtime(self._runtime_contract)
        return _capability_report(status, reason_code, self._runtime_contract, runtime)

    def create_session(
        self,
        task_spec: dict[str, Any],
        root_seed: int,
        batch_width: int,
        auto_reset: bool,
    ) -> "FreeFallSession":
        report = self.capability_report()
        if report["status"] != "available":
            raise ProtocolError(
                "runtime_unavailable",
                "MJX-Warp runtime is unavailable on this host",
                details={"reason_code": report["unavailable_reason_code"]},
            )
        expected = load_json_fixture(self._fixtures / "free-fall-task-spec-v1.json")
        return MjxWarpFreeFallSession(
            validate_bound_task_spec(task_spec, expected),
            root_seed,
            batch_width,
            auto_reset,
            self._fixtures / "free-fall-v1.xml",
        )


class FreeFallSession(ABC):
    """Portable lifecycle and replay logic around one physics implementation."""

    def __init__(
        self,
        task_spec: dict[str, Any],
        root_seed: int,
        batch_width: int,
        auto_reset: bool,
    ) -> None:
        require_unsigned(root_seed, "root_seed")
        if batch_width not in SUPPORTED_BATCH_WIDTHS:
            raise ProtocolError(
                "unsupported_batch_width",
                f"batch_width must be one of {list(SUPPORTED_BATCH_WIDTHS)}",
            )
        if not isinstance(auto_reset, bool):
            raise ProtocolError("invalid_request", "auto_reset must be boolean")
        self.task_spec = deepcopy(task_spec)
        self.root_seed = root_seed
        self.batch_width = batch_width
        self.auto_reset = auto_reset
        self.max_episode_steps = task_spec["termination"]["max_episode_steps"]
        self.episode_indices = [0] * batch_width
        self.episode_seeds = [
            derive_episode_seed(root_seed, lane_id, 0)
            for lane_id in range(batch_width)
        ]
        self.episode_steps = [0] * batch_width
        self.pending_auto_reset = [False] * batch_width
        self.lane_digests = [fnv1a64({"lane_id": lane_id, "seed": seed}) for lane_id, seed in enumerate(self.episode_seeds)]
        self.operations: list[dict[str, Any]] = []
        self._initialize_physics()

    def initial_state(self) -> dict[str, Any]:
        """Returns the reset observation and lane metadata."""

        return self._state_result([True] * self.batch_width)

    def reset_lanes(self, lane_ids: Any) -> dict[str, Any]:
        """Resets a canonical subset without perturbing other lane sequences."""

        lanes = self._validate_lane_ids(lane_ids)
        self._require_replay_capacity()
        self._reset_lanes_internal(lanes, record=True)
        reset_mask = [lane_id in lanes for lane_id in range(self.batch_width)]
        return self._state_result(reset_mask)

    def step(self, actions: Any) -> dict[str, Any]:
        """Steps every lane once and preserves terminal observations."""

        canonical_actions = self._validate_actions(actions)
        self._require_replay_capacity()
        reset_lanes = [
            lane_id
            for lane_id, pending in enumerate(self.pending_auto_reset)
            if pending
        ]
        if reset_lanes:
            self._reset_lanes_internal(reset_lanes, record=False)
        self._step_physics(canonical_actions)
        observations = self._observations()
        rewards: list[float] = []
        terminated: list[bool] = []
        truncated: list[bool] = []
        for lane_id, observation in enumerate(observations):
            self.episode_steps[lane_id] += 1
            finite = all(math.isfinite(value) for value in observation)
            lane_terminated = not finite
            lane_truncated = self.episode_steps[lane_id] >= self.max_episode_steps
            if lane_terminated:
                raise ProtocolError(
                    "non_finite_state",
                    f"lane {lane_id} produced a non-finite physics state",
                    details={"lane_id": lane_id},
                )
            reward = observation[0]
            rewards.append(reward)
            terminated.append(lane_terminated)
            truncated.append(lane_truncated)
            self.pending_auto_reset[lane_id] = self.auto_reset and (
                lane_terminated or lane_truncated
            )
            self._update_lane_digest(
                lane_id,
                {
                    "type": "step",
                    "action": canonical_actions[lane_id],
                    "observation": observation,
                    "reward": reward,
                    "terminated": lane_terminated,
                    "truncated": lane_truncated,
                },
            )
        operation = {"type": "step", "actions": canonical_actions}
        self.operations.append(operation)
        return {
            **self._state_result(
                [lane_id in reset_lanes for lane_id in range(self.batch_width)],
                observations=observations,
            ),
            "rewards": rewards,
            "terminated": terminated,
            "truncated": truncated,
        }

    def checkpoint(self) -> dict[str, Any]:
        """Returns the portable replay checkpoint schema used by RNE CPU batches."""

        return {
            "schema_version": BATCH_CHECKPOINT_SCHEMA_VERSION,
            "seed": self.root_seed,
            "num_envs": self.batch_width,
            "auto_reset": self.auto_reset,
            "seed_strategy": SEED_STRATEGY,
            "task_spec": deepcopy(self.task_spec),
            "lanes": [
                {
                    "lane_id": lane_id,
                    "episode_index": self.episode_indices[lane_id],
                    "episode_seed": self.episode_seeds[lane_id],
                    "pending_auto_reset": self.pending_auto_reset[lane_id],
                    "replay_digest": self.lane_digests[lane_id],
                }
                for lane_id in range(self.batch_width)
            ],
            "operations": deepcopy(self.operations),
            "replay_digest": self._batch_digest(),
        }

    def restore_checkpoint(self, checkpoint: Any) -> dict[str, Any]:
        """Restores solely by replaying the versioned operation log."""

        self._validate_checkpoint_envelope(checkpoint)
        expected_checkpoint = deepcopy(checkpoint)
        operations = deepcopy(checkpoint["operations"])
        self._reset_session_state()
        for operation in operations:
            if not isinstance(operation, dict) or "type" not in operation:
                raise ProtocolError("checkpoint_invalid", "checkpoint operation is invalid")
            if operation["type"] == "step":
                require_exact_keys(
                    operation,
                    required={"type", "actions"},
                    context="checkpoint step operation",
                )
                self.step(operation["actions"])
            elif operation["type"] == "reset_lanes":
                require_exact_keys(
                    operation,
                    required={"type", "lane_ids"},
                    context="checkpoint reset operation",
                )
                self.reset_lanes(operation["lane_ids"])
            else:
                raise ProtocolError(
                    "checkpoint_invalid",
                    f"unsupported checkpoint operation {operation['type']!r}",
                )
        if canonical_json(self.checkpoint()) != canonical_json(expected_checkpoint):
            raise ProtocolError(
                "checkpoint_replay_mismatch",
                "checkpoint metadata or digest does not match deterministic replay",
            )
        return self._state_result([False] * self.batch_width)

    def _reset_session_state(self) -> None:
        self.episode_indices = [0] * self.batch_width
        self.episode_seeds = [
            derive_episode_seed(self.root_seed, lane_id, 0)
            for lane_id in range(self.batch_width)
        ]
        self.episode_steps = [0] * self.batch_width
        self.pending_auto_reset = [False] * self.batch_width
        self.lane_digests = [fnv1a64({"lane_id": lane_id, "seed": seed}) for lane_id, seed in enumerate(self.episode_seeds)]
        self.operations = []
        self._initialize_physics()

    def _reset_lanes_internal(self, lane_ids: list[int], *, record: bool) -> None:
        for lane_id in lane_ids:
            if self.episode_indices[lane_id] == MASK64:
                raise ProtocolError(
                    "episode_index_overflow",
                    f"lane {lane_id} episode index cannot advance",
                )
        self._reset_physics(lane_ids)
        for lane_id in lane_ids:
            self.episode_indices[lane_id] += 1
            self.episode_seeds[lane_id] = derive_episode_seed(
                self.root_seed,
                lane_id,
                self.episode_indices[lane_id],
            )
            self.episode_steps[lane_id] = 0
            self.pending_auto_reset[lane_id] = False
            self._update_lane_digest(
                lane_id,
                {
                    "type": "reset_lane",
                    "episode_index": self.episode_indices[lane_id],
                    "episode_seed": self.episode_seeds[lane_id],
                },
            )
        if record:
            self.operations.append({"type": "reset_lanes", "lane_ids": list(lane_ids)})

    def _state_result(
        self,
        reset_mask: list[bool],
        *,
        observations: list[list[float]] | None = None,
    ) -> dict[str, Any]:
        return {
            "lane_ids": list(range(self.batch_width)),
            "episode_indices": list(self.episode_indices),
            "episode_seeds": list(self.episode_seeds),
            "reset": reset_mask,
            "observations": observations if observations is not None else self._observations(),
            "lane_replay_digests": list(self.lane_digests),
            "replay_digest": self._batch_digest(),
        }

    def _batch_digest(self) -> int:
        return fnv1a64(
            [
                {"lane_id": lane_id, "replay_digest": digest}
                for lane_id, digest in enumerate(self.lane_digests)
            ]
        )

    def _update_lane_digest(self, lane_id: int, event: dict[str, Any]) -> None:
        self.lane_digests[lane_id] = fnv1a64(
            {"previous": self.lane_digests[lane_id], "event": event}
        )

    def _validate_lane_ids(self, lane_ids: Any) -> list[int]:
        if not isinstance(lane_ids, list) or not lane_ids:
            raise ProtocolError("invalid_lane_ids", "lane_ids must be a non-empty list")
        lanes = [require_unsigned(value, "lane_id") for value in lane_ids]
        if lanes != sorted(set(lanes)):
            raise ProtocolError(
                "non_canonical_lane_ids",
                "lane_ids must be strictly increasing and unique",
            )
        if lanes[-1] >= self.batch_width:
            raise ProtocolError("invalid_lane_ids", "lane_id is outside this batch")
        return lanes

    def _validate_actions(self, actions: Any) -> list[list[float]]:
        if not isinstance(actions, list) or len(actions) != self.batch_width:
            raise ProtocolError(
                "action_shape_mismatch",
                f"actions must contain exactly {self.batch_width} lanes",
            )
        canonical: list[list[float]] = []
        for lane_id, action in enumerate(actions):
            if (
                not isinstance(action, list)
                or len(action) != 1
                or isinstance(action[0], bool)
                or not isinstance(action[0], (int, float))
                or not math.isfinite(float(action[0]))
                or float(action[0]) != 0.0
            ):
                raise ProtocolError(
                    "action_bounds_violation",
                    f"lane {lane_id} action must be the exact noop [0.0]",
                )
            canonical.append([0.0])
        return canonical

    def _require_replay_capacity(self) -> None:
        if len(self.operations) >= MAX_REPLAY_OPERATIONS:
            raise ProtocolError(
                "replay_log_full",
                f"session reached the {MAX_REPLAY_OPERATIONS}-operation limit",
            )

    def _validate_checkpoint_envelope(self, checkpoint: Any) -> None:
        if not isinstance(checkpoint, dict):
            raise ProtocolError("checkpoint_invalid", "checkpoint must be an object")
        require_exact_keys(
            checkpoint,
            required={
                "schema_version",
                "seed",
                "num_envs",
                "auto_reset",
                "seed_strategy",
                "task_spec",
                "lanes",
                "operations",
                "replay_digest",
            },
            context="checkpoint",
        )
        if checkpoint["schema_version"] != BATCH_CHECKPOINT_SCHEMA_VERSION:
            raise ProtocolError(
                "checkpoint_version_mismatch",
                f"checkpoint schema must be {BATCH_CHECKPOINT_SCHEMA_VERSION}",
            )
        if (
            checkpoint["seed"] != self.root_seed
            or checkpoint["num_envs"] != self.batch_width
            or checkpoint["auto_reset"] != self.auto_reset
            or checkpoint["seed_strategy"] != SEED_STRATEGY
        ):
            raise ProtocolError(
                "checkpoint_session_mismatch",
                "checkpoint seed, width, reset mode, or seed strategy differs",
            )
        validate_bound_task_spec(checkpoint["task_spec"], self.task_spec)
        if not isinstance(checkpoint["lanes"], list) or len(checkpoint["lanes"]) != self.batch_width:
            raise ProtocolError("checkpoint_invalid", "checkpoint lane count is invalid")
        if not isinstance(checkpoint["operations"], list):
            raise ProtocolError("checkpoint_invalid", "checkpoint operations must be a list")

    @abstractmethod
    def _initialize_physics(self) -> None:
        pass

    @abstractmethod
    def _reset_physics(self, lane_ids: list[int]) -> None:
        pass

    @abstractmethod
    def _step_physics(self, actions: list[list[float]]) -> None:
        pass

    @abstractmethod
    def _observations(self) -> list[list[float]]:
        pass


class FakeFreeFallSession(FreeFallSession):
    """Semi-implicit f64 free fall used to exercise the process contract."""

    def _initialize_physics(self) -> None:
        self._positions_y_m = [INITIAL_POSITION_Y_M] * self.batch_width
        self._velocities_y_m_s = [0.0] * self.batch_width

    def _reset_physics(self, lane_ids: list[int]) -> None:
        for lane_id in lane_ids:
            self._positions_y_m[lane_id] = INITIAL_POSITION_Y_M
            self._velocities_y_m_s[lane_id] = 0.0

    def _step_physics(self, actions: list[list[float]]) -> None:
        del actions
        dt_s = float(self.task_spec["control_step_s"])
        for lane_id in range(self.batch_width):
            self._velocities_y_m_s[lane_id] += GRAVITY_M_S2 * dt_s
            self._positions_y_m[lane_id] += self._velocities_y_m_s[lane_id] * dt_s

    def _observations(self) -> list[list[float]]:
        return [
            [self._positions_y_m[lane_id], self._velocities_y_m_s[lane_id]]
            for lane_id in range(self.batch_width)
        ]


class MjxWarpFreeFallSession(FreeFallSession):
    """MJX-Warp implementation of the bounded free-fall binding."""

    def __init__(
        self,
        task_spec: dict[str, Any],
        root_seed: int,
        batch_width: int,
        auto_reset: bool,
        model_path: Path,
    ) -> None:
        self._model_path = model_path
        super().__init__(task_spec, root_seed, batch_width, auto_reset)

    def _initialize_physics(self) -> None:
        import jax
        import jax.numpy as jnp
        import mujoco
        from mujoco import mjx

        jax.config.update("jax_enable_x64", True)
        if not jax.config.x64_enabled:
            raise ProtocolError(
                "precision_unavailable",
                "MJX-Warp task binding requires JAX f64 support",
            )
        self._jax = jax
        self._jnp = jnp
        self._mjx = mjx
        self._mj_model = mujoco.MjModel.from_xml_path(str(self._model_path))
        self._model = mjx.put_model(self._mj_model, impl="warp")
        naconmax = max(4, self.batch_width * 4)
        njmax = 16

        def make_data(_: Any) -> Any:
            return mjx.make_data(
                self._mj_model,
                impl="warp",
                naconmax=naconmax,
                njmax=njmax,
            )

        self._reset_data = jax.vmap(make_data)(jnp.arange(self.batch_width))
        self._data = self._reset_data
        self._step_function = jax.jit(jax.vmap(lambda data: mjx.step(self._model, data)))
        self._block_until_ready()

    def _reset_physics(self, lane_ids: list[int]) -> None:
        mask = self._jnp.zeros((self.batch_width,), dtype=bool)
        lane_index = self._jnp.asarray(lane_ids, dtype=self._jnp.int32)
        mask = mask.at[lane_index].set(True)
        self._data = self._data.where(mask, self._reset_data)
        self._block_until_ready()

    def _step_physics(self, actions: list[list[float]]) -> None:
        del actions
        self._data = self._step_function(self._data)
        self._block_until_ready()

    def _observations(self) -> list[list[float]]:
        qpos = self._jax.device_get(self._data.qpos)
        qvel = self._jax.device_get(self._data.qvel)
        return [
            [float(qpos[lane_id][1]), float(qvel[lane_id][1])]
            for lane_id in range(self.batch_width)
        ]

    def _block_until_ready(self) -> None:
        self._data.qpos.block_until_ready()


def _probe_runtime(contract: dict[str, Any]) -> tuple[str, str | None, dict[str, Any]]:
    runtime = {
        "python_version": platform.python_version(),
        "platform": platform.system().lower(),
        "machine": platform.machine().lower(),
        "jax_version": None,
        "jaxlib_version": None,
        "jax_cuda_plugin_version": None,
        "mujoco_version": None,
        "mujoco_mjx_version": None,
        "warp_version": None,
        "jax_backend": None,
        "jax_devices": [],
        "nvidia_driver_version": None,
    }
    if platform.system().lower() != contract["operating_system"]:
        return "unavailable", "host_os_mismatch", runtime
    if platform.machine().lower() != contract["architecture"]:
        return "unavailable", "host_architecture_mismatch", runtime
    required_python = tuple(int(part) for part in contract["python"].split("."))
    if sys.version_info[:2] != required_python:
        return "unavailable", "python_version_mismatch", runtime
    driver_version = _nvidia_driver_version()
    runtime["nvidia_driver_version"] = driver_version
    if driver_version is None:
        return "unavailable", "nvidia_driver_unavailable", runtime
    try:
        driver_major = int(driver_version.split(".", maxsplit=1)[0])
    except ValueError:
        return "unavailable", "nvidia_driver_invalid", runtime
    if driver_major < contract["nvidia_driver_minimum"]:
        return "unavailable", "nvidia_driver_too_old", runtime
    if importlib.util.find_spec("jax") is None:
        return "unavailable", "jax_missing", runtime
    if importlib.util.find_spec("mujoco") is None:
        return "unavailable", "mujoco_missing", runtime
    try:
        import jax
        import jaxlib
        import mujoco
        import warp
        from importlib import metadata
        from mujoco import mjx
        import mujoco.mjx.warp  # noqa: F401

        del mjx
        runtime["jax_version"] = getattr(jax, "__version__", "unknown")
        runtime["jaxlib_version"] = getattr(jaxlib, "__version__", "unknown")
        runtime["jax_cuda_plugin_version"] = metadata.version("jax-cuda13-plugin")
        runtime["mujoco_version"] = getattr(mujoco, "__version__", "unknown")
        runtime["mujoco_mjx_version"] = metadata.version("mujoco-mjx")
        runtime["warp_version"] = getattr(warp, "__version__", "unknown")
        runtime["jax_backend"] = jax.default_backend()
        runtime["jax_devices"] = [str(device) for device in jax.devices()]
    except (ImportError, OSError, RuntimeError):
        return "unavailable", "mjx_warp_import_failed", runtime
    expected = contract["packages"]
    installed = {
        "jax": runtime["jax_version"],
        "jaxlib": runtime["jaxlib_version"],
        "jax_cuda_plugin": runtime["jax_cuda_plugin_version"],
        "mujoco": runtime["mujoco_version"],
        "mujoco_mjx": runtime["mujoco_mjx_version"],
        "warp_lang": runtime["warp_version"],
    }
    if installed != expected:
        return "unavailable", "runtime_version_mismatch", runtime
    if runtime["jax_backend"] != "gpu":
        return "unavailable", "jax_backend_not_gpu", runtime
    return "available", None, runtime


def _capability_report(
    status: str,
    reason_code: str | None,
    runtime_contract: dict[str, Any],
    runtime: dict[str, Any],
) -> dict[str, Any]:
    return {
        "kind": CAPABILITY_REPORT_KIND,
        "schema_version": CAPABILITY_REPORT_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "runtime_id": RUNTIME_ID,
        "status": status,
        "unavailable_reason_code": reason_code,
        "execution_boundary": "out_of_process_python",
        "precision": "f64",
        "protocol_schema": PROTOCOL_SCHEMA_VERSION,
        "task_spec_schema": TASK_SPEC_SCHEMA_VERSION,
        "batch_checkpoint_schema": BATCH_CHECKPOINT_SCHEMA_VERSION,
        "conformance_report_schema": CONFORMANCE_REPORT_SCHEMA_VERSION,
        "scale_report_schema": SCALE_REPORT_SCHEMA_VERSION,
        "supported_task_ids": [TASK_ID],
        "supported_batch_widths": list(SUPPORTED_BATCH_WIDTHS),
        "requires_nvidia_gpu": True,
        "unsupported_features": list(UNSUPPORTED_FEATURES),
        "runtime": runtime,
        "runtime_contract": deepcopy(runtime_contract),
        "runtime_contract_schema": runtime_contract["schema_version"],
    }


def _load_runtime_contract(path: Path) -> dict[str, Any]:
    try:
        with path.open("rb") as source:
            contract = tomllib.load(source)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ProtocolError(
            "runtime_contract_invalid",
            "failed to load the pinned accelerator runtime contract",
        ) from error
    expected = {
        "schema_version": 1,
        "operating_system": "linux",
        "architecture": "x86_64",
        "python": "3.12",
        "cuda_major": 13,
        "nvidia_driver_minimum": 580,
        "official_sources": [
            "https://docs.jax.dev/en/latest/installation.html",
            "https://pypi.org/pypi/jax/0.10.2/json",
            "https://pypi.org/pypi/mujoco-mjx/3.9.0/json",
        ],
        "packages": {
            "jax": "0.10.2",
            "jaxlib": "0.10.2",
            "jax_cuda_plugin": "0.10.2",
            "mujoco": "3.9.0",
            "mujoco_mjx": "3.9.0",
            "warp_lang": "1.12.1",
        },
    }
    if contract != expected:
        raise ProtocolError(
            "runtime_contract_invalid",
            "accelerator runtime contract fields or pins do not match",
        )
    return contract


def _nvidia_driver_version() -> str | None:
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=driver_version",
                "--format=csv,noheader",
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    versions = sorted({line.strip() for line in result.stdout.splitlines() if line.strip()})
    if len(versions) != 1 or len(versions[0]) > 64:
        return None
    return versions[0]
