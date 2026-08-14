# ADR 014: Select MJX-Warp as the single accelerator target

## Status

Accepted for implementation, experimental and default-off. Reviewed against
official documentation on 2026-08-14.

## Context

The v0.4 plan requires one accelerator adapter only after comparing MJX,
Genesis World, and Isaac Lab. The adapter must preserve TaskSpec semantics and
remain outside core crates. Raw vendor benchmark numbers are not comparable to
RNE's task, so this spike measured integration cost and local feasibility rather
than inventing a throughput comparison.

The local preflight host had Windows 11, Python 3.12.10, 12 logical CPU threads,
and an NVIDIA GTX 1660 Ti with 6 GiB VRAM. `nvidia-smi` detected the GPU, while
the installed JAX 0.10.1 and PyTorch 2.12 environments both exposed CPU only.
The RNE CPU reference task separately measured the required 1/16/256/4096 lane
points and proved one identical lane-zero digest; those timings are a baseline,
not accelerator evidence.

Official integration surfaces were compared:

| Candidate | Install/runtime surface | RNE reuse | Local preflight | Boundary cost |
|---|---|---|---|---|
| MJX-Warp | `mujoco-mjx[warp]`; device `Model`/`Data`; fixed batched worlds | Reuses MuJoCo/MJCF assets and the existing MuJoCo conformance work | GPU runtime unavailable on this Windows Python environment; CPU-side contract work remains possible | One out-of-process Python adapter and one MuJoCo task binding |
| Genesis World | PyTorch plus `genesis-world`; process-global initialization; default f32 | No existing Genesis backend or scene mapping | Package not installed; installed PyTorch is CPU-only | New asset, state, actuator, contact, and determinism mapping |
| Isaac Lab | Isaac Sim plus matched Python/CUDA PyTorch and Isaac Lab | Useful task ecosystem, but no existing PhysX/Omniverse boundary | Reference requirements call for at least 32 GiB RAM and 16 GiB VRAM, exceeding the 6 GiB host GPU | Largest runtime, packaging, EULA, renderer, and CI surface |

[MuJoCo's MJX documentation](https://mujoco.readthedocs.io/en/latest/mjx.html)
states that MJX accepts MuJoCo models, represents model/data with device arrays,
supports batch dimensions, and that MJX-Warp targets NVIDIA GPU performance and
broader contact/mesh workloads. [Genesis installation
documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html)
documents its cross-platform PyTorch installation. [Isaac Lab installation
documentation](https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/index.html)
documents the Isaac Sim dependency and reference hardware requirements.

## Decision

Select MJX-Warp as the only accelerator implementation maintained for v0.4.
The adapter ID is `mjx_warp`; `adapters/mjx/accelerator.toml` is its versioned,
CI-validated selection manifest. MJX-JAX may be used in disposable research but
is not a second supported runtime. Genesis World and Isaac Lab remain rejected
for this milestone; no compatibility layer or dormant dependency is added.

The adapter runs in a separate Python process. It receives a validated TaskSpec,
versioned MuJoCo task binding, root seed, lane IDs, episode indices, and flat
action buffers. It returns flat observation buffers, reward terms, termination,
reset metadata, and accelerator evidence. CUDA, JAX, Warp, MuJoCo, and Python
types never enter `rne_core`, `rne_ecs`, `rne_world`, `rne_robot`, or
`rne_physics` public APIs.

## Promotion gates

Implementation may be called available only after all of these pass on a Linux
NVIDIA runner:

- the same TaskSpec agrees on tensor names, shape, dtype, unit, bounds, and order;
- lane zero matches CPU MuJoCo under named outcome/tolerance contracts;
- batch widths 1, 16, 256, and 4096 report hardware, precision, warm-up, and task;
- partial reset does not perturb another lane's episode sequence;
- unsupported MJCF features fail before stepping with a stable capability report;
- an injected divergence produces a replay and verified Failure Capsule;
- package and driver versions are pinned outside core crates.

Until then, the manifest status remains `experimental`, release and headless CI
validate only the boundary and contract, and RNE makes no accelerator throughput
claim.

## Consequences

RNE gets the shortest path from its existing MuJoCo/MJCF work to device batches
without turning a research framework into a core dependency. The choice gives
up Genesis's broader portable GPU story and Isaac Lab's task ecosystem for now.
That trade is deliberate: TaskSpec and evidence remain portable, so a future
milestone can revisit the decision through a new ADR without supporting three
accelerator stacks simultaneously.
