# G1 EDU partial-specification backflip

The same controller passes at **0.125 ms and 0.0625 ms** with self-collision,
120 N m knee ceilings, URDF-declared mass, 2 ms held commands and an explicitly
assumed 2 ms command delay. It is a contact-model result, not hardware validation.
See [the model contract](../../../G1_CONTACT_BACKFLIP.md) for remaining assumptions.

- `candidate.json`: selected 16-parameter controller and full replay settings.
- `step-125us/`, `step-62us/`: independent passing five-second rollouts and hashes.
- `rejected-speed.json`: a standing backflip rejected for excessive knee speed.
- `rejected-coarse-refinement.json`: a 0.5 ms success that falls at 0.125 ms.
- `standard-90nm.json`: the selected EDU candidate fails with the standard G1 cap.
- `search-input.json`, `search.json`, `search-final-population.json`: the final
  fine-step search inputs, seed, bounds, versions, source hashes and checkpoint.
  Forty evaluations completed one generation before the success callback stopped
  the run. The canonical candidate is a passing member selected for verification,
  not a claim of globally optimal cost. Earlier coarse searches initialized it.

Replay and render using the environment described in the main document:

```bash
target/research/backflip-env/bin/python -I scripts/g1_backflip_search.py --parameters docs/evidence/g1-contact-backflip/edu/candidate.json --generations 0 --output target/research/edu-replay
target/research/backflip-env/bin/python -I scripts/g1_backflip_search.py --parameters docs/evidence/g1-contact-backflip/edu/candidate.json --dt-s .0000625 --generations 0 --output target/research/edu-fine-replay
MUJOCO_GL=egl target/research/backflip-env/bin/python -I scripts/g1_backflip_render.py target/research/edu-replay --output target/research/edu-backflip.gif
```

The renderer verifies the passing flag and recording hash. It renders recorded
states only. The GIF labels the knee ceiling and enabled self-collision. The
regression test independently re-simulates the controller and compares every
recorded frame through its SHA-256. Failed candidates cannot be rendered as
successful GIFs.

To reproduce the final seeded search, reconstruct its saved initial population:

```bash
python3 - <<'PY'
import json
from pathlib import Path
source = Path('docs/evidence/g1-contact-backflip/edu/search.json')
output = Path('target/research/edu-initial-population.json')
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({'population': json.loads(source.read_text())['initial_population']}))
PY
OPENBLAS_NUM_THREADS=1 target/research/backflip-env/bin/python -I scripts/g1_backflip_joint_search.py --parameters docs/evidence/g1-contact-backflip/edu/search-input.json --population target/research/edu-initial-population.json --seed 20260925 --dt-s .000125 --workers 8 --generations 4 --output target/research/edu-search
```

Use a fresh output directory. Worker count changes scheduling, not candidate
ordering; numerical reproducibility still depends on the pinned software and
platform. A passing search candidate must also be checked at a finer timestep.
No real-robot commands, policy training or external base wrenches are involved.
