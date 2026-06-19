# Mobile Manipulator RL Artifact Schemas

This directory uses small, versioned artifacts so training runs can be resumed,
compared, and used as regression gates without depending on external packages.
The schemas below are example-level contracts for
`examples/27_mobile_manipulator_rl`; they are not yet global RNE benchmark
formats. The policy artifact schema descriptors and hashes are defined in
`policy_schema.py` and consumed by `train.py`, `compare_reports.py`, and the
regression tests.

All JSON files are UTF-8, pretty-printed, newline-terminated, and written through
a temporary file followed by atomic replace where the current tool owns the
writer. Relative artifact paths are resolved from the JSON file that contains
them.

## Versioning

Each machine-readable artifact carries an explicit `schema_version`. Readers
reject unknown schema or algorithm IDs when the artifact is consumed as
executable state, such as policy loading or CEM checkpoint resume. The policy
artifact is currently `schema_version: 2`; the other artifacts in this directory
are currently `schema_version: 1`.

Derived artifacts such as `rollout.svg`, `rollout.html`, and `index.html` are
not canonical. They can be regenerated from the JSON/CSV artifacts.

## CEM Training Checkpoint

Written by:

```bash
python train.py --checkpoint cem_checkpoint.json
```

Purpose: resume an interrupted CEM training run exactly at the next unfinished
iteration.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `algorithm` | string | `rne_mobile_manipulator_cem_reach_v1` |
| `next_iteration` | integer | next CEM iteration to execute |
| `target_iterations` | integer | total requested iterations for the saved run |
| `population` | integer | CEM population size |
| `elite` | integer | selected elite count |
| `seed` | integer | Python CEM sampler seed |
| `param_dim` | integer | policy parameter count, currently `4` |
| `targets_per_candidate` | integer | randomized targets per candidate |
| `episode_steps` | integer | max rollout steps per target |
| `mean` | array[number] | current CEM mean |
| `std` | array[number] | current CEM standard deviation |
| `best_params` | array[number] | best policy parameters seen so far |
| `best_reward` | number | best mean reward seen so far |
| `history` | array[array] | per-iteration `(iteration, mean_reward, best_reward)` rows |
| `rng_state` | array | JSON form of Python `random.Random` state |

Compatibility notes:

- `--resume` rejects mismatched `population`, `elite`, `param_dim`,
  `targets_per_candidate`, and `episode_steps`.
- `--iterations` on resume is a total target count, not an additional count.
- The checkpoint is trainer state, not a simulation replay. It does not replace
  `rne_py.VectorizedMobileManipulatorEnv` checkpoints.

## Policy Artifact

Written by:

```bash
python train.py --policy-out best_policy.json
python train.py --report-dir reports/reach
python compare_reports.py reports --best-policy-out reports/best_policy.json
```

Purpose: portable learned linear policy for evaluation or report generation.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `2` |
| `algorithm` | string | `rne_mobile_manipulator_linear_reach_policy_v1` |
| `observation_schema_hash` | string | SHA-256 hash of `observation_schema` |
| `action_schema_hash` | string | SHA-256 hash of `action_schema` |
| `observation_schema` | object | full observation descriptor expected by the policy |
| `action_schema` | object | full action descriptor expected by the policy |
| `policy_features` | array[string] | observation fields read by the linear policy |
| `policy_outputs` | array[string] | action fields written by the linear policy |
| `normalization` | object | input normalization contract, currently identity |
| `action_scaling` | object | output scaling/clipping contract |
| `task_compatibility` | object | compatible RNE task family and task names |
| `engine_compatibility` | object | Python/RNE API expected by the artifact |
| `param_dim` | integer | policy parameter count, currently `4` |
| `action_limit_rad_s` | number | shoulder/elbow action clipping limit |
| `params` | array[number] | linear policy parameters |
| `best_reward` | number | best training reward associated with the policy |
| `training_iterations` | integer | number of CEM iterations behind the artifact |

Compatibility notes:

- `train.py --policy-in` rejects unknown schema, algorithm, `param_dim`, or
  `action_limit_rad_s`; it also rejects observation/action schema hash,
  feature/output, normalization, scaling, task, or engine compatibility
  mismatches.
- `compare_reports.py` validates that a report's `policy.json` exists, contains
  the required fields, matches the manifest's policy schema, algorithm, and
  schema hashes, has embedded observation/action descriptors matching those
  hashes, and has `params` length equal to `param_dim`.
- The policy format is intentionally narrow. It is not a generic neural-network
  weight container.

## Rollout CSV

Written by:

```bash
python train.py --rollout-csv rollout.csv
python train.py --report-dir reports/reach
```

Purpose: canonical per-step trajectory data for plotting and HTML replay.

Header fields:

```text
step,base_x,base_y,base_yaw,ee_x,ee_y,ee_z,target_dx,target_dy,target_dz,shoulder_action,elbow_action,reward,total_reward,done
```

Semantics:

- `step` is zero-based within the recorded rollout.
- `base_*` and `ee_*` are the observation before applying that row's action.
- `target_d*` is the target offset in the observation used by the policy.
- `reward` is the reward delta produced by the action.
- `total_reward` is the episode return after the action.
- `done` is the post-step termination flag.

## Report Manifest

Written by:

```bash
python train.py --report-dir reports/reach
```

Purpose: index one report bundle and provide the fields needed for leaderboard
ranking.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `policy_algorithm` | string | policy algorithm ID |
| `policy_schema_version` | integer | policy schema version used by `policy.json` |
| `observation_schema_hash` | string | hash of the policy observation schema |
| `action_schema_hash` | string | hash of the policy action schema |
| `best_reward` | number | best training reward |
| `training_iterations` | integer | CEM iterations behind the policy |
| `rollout_rows` | integer | number of rows in `rollout.csv` |
| `final_total_reward` | number | final total reward from the recorded rollout |
| `final_target_error` | number | final target error from the recorded rollout |
| `artifacts` | object | relative artifact paths |

Current `artifacts` keys:

| key | default path | meaning |
|---|---|---|
| `index` | `index.html` | report landing page |
| `policy` | `policy.json` | policy artifact |
| `rollout_csv` | `rollout.csv` | canonical rollout data |
| `rollout_svg` | `rollout.svg` | derived static visualization |
| `rollout_html` | `rollout.html` | derived interactive replay |

`compare_reports.py` requires `index`, `policy`, and `rollout_csv` to be
present and to point at existing files. All artifact paths must be non-empty
relative paths that stay inside the report directory; absolute paths and `../`
escapes are rejected.

## Leaderboard JSON

Written by:

```bash
python compare_reports.py reports --json reports/leaderboard.json
python sweep.py --out reports/sweep
```

Purpose: machine-readable ranked results across report bundles.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `reports_considered` | integer | number of report manifests ranked |
| `ranking` | array[object] | ordered tie-breaking criteria |
| `reports` | array[object] | ranked report rows |

Ranking criteria:

1. `final_target_error` ascending
2. `final_total_reward` descending
3. `best_reward` descending
4. `report` ascending

Each `reports` row contains:

| field | type | meaning |
|---|---|---|
| `rank` | integer | one-based rank |
| `report` | string | report directory name |
| `policy_algorithm` | string | policy algorithm ID |
| `policy_schema_version` | integer | policy artifact schema version |
| `observation_schema_hash` | string | hash of the policy observation schema |
| `action_schema_hash` | string | hash of the policy action schema |
| `final_target_error` | number | ranking primary metric |
| `final_total_reward` | number | ranking secondary metric |
| `best_reward` | number | training best reward |
| `training_iterations` | integer | CEM iterations behind the policy |
| `rollout_rows` | integer | rollout CSV rows |
| `index_path` | string | path relative to the leaderboard JSON directory |
| `policy_path` | string | path relative to the leaderboard JSON directory |
| `manifest_path` | string | path relative to the leaderboard JSON directory |

## Best Report JSON

Written by:

```bash
python compare_reports.py reports --best-report-out reports/best_report.json
python sweep.py --out reports/sweep
```

Purpose: compact machine-readable pointer to the selected winning report.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `reports_considered` | integer | number of report manifests ranked |
| `ranking` | array[object] | same criteria as `leaderboard.json` |
| `best` | object | same row shape as one `leaderboard.json.reports` entry |

`best` is intentionally the same payload as `leaderboard.json.reports[0]` when
both files are written to the same directory.

## Sweep Outputs

`sweep.py --out reports/sweep` writes:

| path | canonical | meaning |
|---|---:|---|
| `seed_XXXX/checkpoint.json` | yes | CEM resume state for that seed |
| `seed_XXXX/manifest.json` | yes | report manifest for that seed |
| `seed_XXXX/policy.json` | yes | policy artifact for that seed |
| `seed_XXXX/rollout.csv` | yes | rollout data for that seed |
| `leaderboard.csv` | yes | tabular leaderboard |
| `leaderboard.json` | yes | machine-readable full leaderboard |
| `best_policy.json` | yes | copied policy from the winning report |
| `best_report.json` | yes | metrics and paths for the winning report |
| `sweep_manifest.json` | yes | sweep configuration, expected inputs, outputs, and gates |
| `leaderboard.html` | no | derived leaderboard page |
| `seed_XXXX/index.html` | no | derived report landing page |
| `seed_XXXX/rollout.svg` | no | derived static rollout visualization |
| `seed_XXXX/rollout.html` | no | derived interactive rollout replay |

The sweep compares only the expected `seed_XXXX/manifest.json` files for the
requested `--runs` and `--seed-start`. Stale reports elsewhere under `--out` do
not affect ranking or quality gates.

## Sweep Manifest JSON

Written by:

```bash
python sweep.py --out reports/sweep
```

Purpose: record the sweep configuration and provenance after a successful
training and compare pass.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `runner` | string | runner path, currently `examples/27_mobile_manipulator_rl/sweep.py` |
| `config` | object | requested sweep parameters |
| `seeds` | array[integer] | expected seed values for this sweep |
| `quality_gates` | object | gate thresholds passed to `compare_reports.py` |
| `expected_manifests` | array[string] | manifest paths relative to `sweep_manifest.json` |
| `outputs` | object | sweep output paths relative to `sweep_manifest.json` |
| `commands` | object | train and compare commands used by the run |

`sweep_manifest.json` is written only after `compare_reports.py` succeeds. In
`--dry-run` mode the command lines are printed, but the manifest is not written.

## Quality Gates

`compare_reports.py` and `sweep.py` can fail with a non-zero exit status when:

- fewer than `--require-reports-at-least` manifests are ranked;
- the winning `final_target_error` is greater than
  `--require-final-error-at-most`;
- the winning `final_total_reward` is less than
  `--require-final-reward-at-least`.

These gates are intended for short regression checks. Longer multi-seed learning
quality gates should use stable seed sets and aggregate metrics before being
promoted to CI.
