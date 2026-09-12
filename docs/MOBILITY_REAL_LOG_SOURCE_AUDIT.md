# Mobility real-log source audit

Status: source screening and bounded NCLT, F1TENTH and DDMR inspection, 2026-09-08.
Data resides on external storage; no physical calibration qualification completed.
Uninspected candidate descriptions are not verified channel manifests.

## Selection and limits

### Additional electrical-identification candidate (2026-09-09)

The authors' [OpenMCT DC motor dataset, version 1](https://data.mendeley.com/datasets/5xvg43r9r8/1)
(DOI `10.17632/5xvg43r9r8.1`, published 2026-05-11) declares CC BY 4.0,
13 raw logs and over 52,000 rows. Its description lists PWM, speed, loop interval,
raw/filtered current sensing and digital-multimeter current when enabled, together
with calibration, APRBS, PI/discrete-controller and chirp experiments. This is a
bench motor candidate, not a differential/skid or Ackermann vehicle validation.
The linked SSRN article returned HTTP 403 during screening; its contents were not
reviewed. At initial screening, no raw file, calibration table or processing script
had been downloaded or executed, and file sizes/hashes were unknown. Subsequent
bounded inspection and raw-log reproduction are recorded below.

The public site's client bundle subsequently established the anonymous listing
route `/public-api/datasets/5xvg43r9r8/files?folder_id=root&version=1`
(Accept `application/vnd.mendeley-public-dataset.1+json`) and folder route
`/public-api/datasets/5xvg43r9r8/folders/1`. These returned metadata successfully;
the guessed `/versions/1` route did not exist. Root documentation is individually
available: `DATA_DESCRIPTION.md` 15,112 bytes, `DATASET_METADATA.md` 2,535 bytes,
`LICENSE` 385 bytes and `README.md` 2,841 bytes. The listing declares SHA-256
`f015801923f2fd17b833159f1de2f64b60450913d5ad1c774a5a3118e86f5479`
for `DATA_DESCRIPTION.md` (file ID `85b0dbe5-c0ec-4a7b-bcc9-82aa44688b22`).
This is server-declared metadata, not yet a locally verified content hash.
Folder metadata separates current calibration, static characterization,
system identification, continuous/discrete validation and optional characterization.
Read these small documentation files before selecting any experiment log; the
root documentation sizes do not bound the experiment folders or complete dataset.

Documentation-only inspection subsequently read the data description, hardware
metadata, calibration README and calibration MATLAB source (without executing it).
The declared hardware is Teensy 4.0, DRV8874 IPROPI, TSINY ts-25GA370H-20 and
Siglent SDM3045X. Speed is RPM, `DT_ms` is a loop interval, and PWM is a command;
`CURRENT_RAW` is ADC counts while `CURRENT_AVG` already uses firmware filtering
and an earlier calibration. Missing DMM fields use `nan` and sample ID `-1`.
The DMM is asynchronous and reused IDs do not represent independent measurements.

The inspected [calibration script](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/2ba66e22-88b5-4fd7-88ea-59433626db22/file_downloaded)
constructs time by cumulative loop intervals, takes absolute DMM current, selects
age <= 10 ms and one minimum-age row per DMM ID, then removes residuals beyond
4 * 1.4826 * MAD after an initial linear fit. Its final RMSE is computed on the
retained fitting rows, not independent holdout. Negative calibrated values are
clipped for displayed output. An RNE audit must retain rejected-row counts,
unclipped residuals and DMM identity; this processing is not signed current or
clock-calibration evidence. The fit mask also lacks an explicit nonnegative-age
check, so raw age validity needs independent inspection.

The calibration folder listing declares `raw_data.txt` 258,643 bytes, file ID
`187827aa-3750-4d23-b513-ddf00d64480c`. Next acquire this individual log with a
300,000-byte hard bound and verify its server-declared SHA-256, then audit all
rows before fitting.

Bounded acquisition completed: the exact 258,643 bytes matched server SHA-256
`9dfa5b4ffaea999ef3792537b3a17611feb6ee3da0add47c63857cb11f0179a3` and were saved
without overwrite at
`E:\RoboSim-external-data\mobility-openmct-5xvg43r9r8-v1\current-calibration-raw.txt`.
A preliminary PowerShell CSV inspection (not yet a strict production reader)
found 4,558 rows, two missing DMM rows, 3,947 distinct valid DMM IDs and 609 reused
rows. Reused IDs retained identical DMM time/current in this inspection. There
were no negative ages; valid ages ranged 0.8–57.6 ms. Nonnegative ADC and age
0–10 ms selected 1,952 rows with 1,952 distinct DMM IDs before residual rejection.
All loop-interval fields were exactly 20 ms; cumulative interval sum is 91.16 s,
not an independently measured acquisition duration. ADC counts ranged 0–296.
These observations do not yet reproduce the fit, certify timing or provide an
independent validation capture. The next reader must validate exact headers,
column count, finite required channels, missing-DMM tuples, integer IDs and
source ordering rather than relying on permissive `ConvertFrom-Csv` parsing.

`recorded_openmct::read_openmct` now implements an offline strict reader with
8 MiB/100,000-row/1,024-byte-line bounds and exact-byte hashing. It retains signed
DMM current, original RPM, PWM commands, raw ADC and filtered source current
separately. The documented missing tuple becomes `None`, never zero. Reused DMM
IDs must retain bit-identical current/time; ID or DMM-time regression is rejected.
No age selection, absolute-value conversion, calibration, filtering or resampling
occurs in ingestion. Metadata dates are retained as declarations, not converted
to a simulation clock. Focused synthetic tests cover preservation and rejection;
validation is in progress. The read-only `openmct_audit` example has now read the
physical file successfully through this strict reader: 4,558 rows, two missing
DMM rows, 3,947 distinct IDs, 609 reused rows, age range 0.8–57.6 ms and the exact
SHA-256 above matched the preliminary audit. The first two synthetic tests and
all-target Clippy passed. After adding byte/row/line-bound and read-error tests,
all three focused tests passed (0.33 s), together with all-target Clippy. Full
workspace validation subsequently passed as recorded below. This is successful
ingestion, not calibration acceptance.

```text
cargo run -p rne_mobility_benchmark --example openmct_audit -- <external raw-log.txt>
```

The frozen reader implementation (Git blob
`4f204f9b36401ddbd13565bdc673abe7c5397a90`) completed `cargo run -p xtask -- ci`
with exit 0. Mobility library: 167 passed, zero failures, one ignored long training
job (182.73 s). Workspace lint/tests, headless, OSS parity, 361 fuzz cases and
Behavior CI 10/10 seeds passed. External log:
`E:\RNE-build\m3c-sensor\openmct-ingestion-v1-ci.log`, SHA-256
`b5e8bf2b92538f82526819353828dd744aed4fc76bb754c6f195571a99619feb`.
This default-feature run does not establish MuJoCo feature coverage. Negative
results remain: heading CEM score -10 equals baseline; clutter PPO -1.37 is below
random 2.01; mobile CEM does not place; flagship evidence is `cross_backend=false`.

A separate read-only PowerShell recomputation on the acquired raw log reproduced
the author's selection and linear/MAD-refit arithmetic: 1,952 candidates, 1,654
retained and 298 rejected; slope 1.1860067240413787 mA/count, intercept
-1.4498528146829202 mA. Unclipped retained-fit RMSE was 5.037074098976796 mA,
whereas all-candidate RMSE under that same refit was 12.773130702197356 mA.
Initial OLS all-candidate RMSE was 12.413588291154817 mA. This independent
implementation check used permissive CSV parsing after strict ingestion succeeded;
it is not yet a reusable RNE calibration API or an independent validation capture.
Do not interpret rejected samples as known sensor faults or fit residuals as a
calibrated noise distribution. A future tested reproduction must retain every
candidate and its selection reason, and report excluded and retained errors.

`recorded_openmct::calibration::reproduce_current_calibration` now implements the
fixed published selection/MAD-refit protocol in Rust. Every original row keeps
a missing/stale/reused/residual-rejected/fit decision. Candidate indices and
unclipped final-law residuals remain available, together with both initial and
refitted laws in A/count and A, and all-candidate/retained RMSE in A. This is
descriptive reproduction on a single recording, not an independent calibration
validation API or a signed-current model. Focused reader/calibration tests pass
(6 tests, including MAD rejection visibility, zero MAD, insufficient/rank-deficient
excitation and numerical overflow); focused crate Clippy passes. The read-only
`openmct_calibration_reproduction` example also ran against the external raw log:
4,558 rows, 1,952 candidates, 1,654 retained and 298 residual-rejected. The Rust fit
is 0.0011860067240414037 A/count with intercept -0.0014498528146834992 A;
retained RMSE is 0.005037074098976795 A and all-candidate RMSE is
0.012773130702197143 A. These agree with the independent arithmetic reproduction
above to floating-point precision. This is still the same recording, not an
independent validation dataset.

Calibration reproduction CI (2026-09-09): `cargo run -p xtask -- ci` completed
with exit code 0, covering formatting, dependency boundaries, workspace Clippy,
workspace tests, smokes, headless checks, OSS parity, 361 fuzz cases across nine
boundaries and Behavior CI 10/10. The mobility library passed 170 tests with one
explicitly ignored long-training test (204.14 s); the fixed CLI test passed
(1.33 s), NCLT audit passed three tests, and suspension CLI passed (2.31 s).
The calibration module Git blob was `4edfdd0ae0748a19a2e15cdaef40898d4a8634dd`.
Log: `E:\RNE-build\m3c-sensor\openmct-calibration-v1-ci.log`, SHA-256
`8555f89b79d3fbb664f1aaa19c3aceca351826ae7ace0eff18199489627e9eab`.
This default-feature CI does not claim a new MuJoCo-feature run or physical
qualification. Negative observations remain: heading CEM matched its -10
baseline; mobile clutter CEM grasped but did not place; flagship evidence reports
`cross_backend=false`. Clutter PPO reported random -1.52 / trained -1.37,
and mobile clutter PPO random -1.84 / trained -1.61. These smoke scores do not
establish general learning performance or sim-to-real transfer.

Next-run screening (2026-09-09): the official 10 ms identification folder lists
`raw_data.txt`, file ID `6f95a940-fe9a-483b-bdbd-a47f5fa3acda`, 173,345 bytes,
server-declared SHA-256
`198770b1bb27f13617531b0fbbc6ade83b69e3032562f19dc33c8bb7f19e3f24`.
Its [README](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/28b1dfba-2e5d-4df6-bcde-ccea0ec343d4/file_downloaded)
was read and declares 3,089 samples, nominal 10 ms timing, PWM command range
0–227 and measured speed range -6–268 RPM. It reports GUI continuous-transfer
fits, including `32.87 / (s + 28.9)`; these are author-reported fits, not locally
reproduced or independent validation results. The raw file has not yet been
downloaded or hash-verified. Before applying the frozen current calibration,
inspect whether it contains usable DMM references; before dynamics validation,
select a different capture for holdout and verify the sampling/command convention.
PWM-to-speed fitting alone cannot identify physical voltage/current constants.

The 10 ms raw log was subsequently downloaded with a 200,000-byte streaming
bound, exact size and SHA-256 verification, and create-new storage at
`E:\RoboSim-external-data\mobility-openmct-5xvg43r9r8-v1\identification-10ms-raw.txt`.
The strict Rust reader accepted all 3,089 rows: one missing DMM row, 1,401 distinct
DMM IDs, 1,687 repeated-ID rows, ages 0.7–47.7 ms, and all declared loop intervals
10 ms. Constant declared intervals do not independently establish actual timing.

A read-only arithmetic check applied the frozen calibration-recording Rust law
to this different capture, without fitting on it, clipping predictions or rejecting
residuals. The same age <= 10 ms/freshest-per-ID selection left 600 references.
Current-magnitude RMSE was 0.03386975527905495 A, MAE 0.009484439084428132 A,
mean reference-minus-prediction residual -0.0005510977590963855 A and maximum
absolute residual 0.5413756008579356 A. This substantially exceeds the retained
calibration-recording RMSE; it must not be replaced by a refitted or trimmed score.
It is a separate-capture diagnostic, not a preregistered acceptance test, an
independent instrument calibration, or qualified physical current fidelity.
Asynchronous DMM integration and reused samples limit temporal correspondence;
the source of the large errors has not been established.

`recorded_openmct::evaluation::evaluate_current_calibration` now implements a
training-only fit followed by frozen-law evaluation. Every evaluation row retains
its original DMM identity/time/age, unclipped prediction, optional residual and
selection flag. Selected-reference RMSE and maximum error use no residual rejection.
Exact-byte and identical-numeric-row duplicate captures are rejected, but this
does not certify independent acquisition or detect every overlapping capture.
The first seven focused tests passed. The read-only
`openmct_calibration_evaluation` example ran on both physical captures and reproduced
600 selected references, RMSE 0.0338697552790549 A and maximum error
0.541375600857936 A. It emits every evaluation row as JSON on stdout. Expanded
selection/absence/overflow tests are running; full CI for this new API is pending.

The largest selected residual occurs at zero-based source row 1,501: ADC 498,
predicted magnitude 0.5891814957579355 A versus DMM -0.0478058949 A (ID 1,658,
time 14.995629 s, age 9.7 ms). Immediately preceding rows change PWM from 227
to zero, followed by reported speed falling from 256 RPM. This establishes
coincidence with a command transient, not a proven causal explanation. Evaluation
ADC values span 0–623, exceeding the calibration raw range 0–296; even that raw
range is not the fitted-row support interval. The evaluation API now reports the
inclusive retained-fit ADC interval, each original ADC value and an extrapolation
flag. The interval uses only `Fit` rows, not residual-rejected/missing/stale/reused
training rows. Neither extrapolation nor a negative prediction removes a row
from error aggregation. Interval membership is not an accuracy guarantee.
All nine focused tests and crate all-target Clippy passed after this extension.
The physical-log run reports a retained-fit ADC interval of **0–152 counts**, not
the raw calibration interval 0–296. Of 3,089 evaluation rows, 35 are outside this
interval; 12 of the 600 selected DMM references are outside. All remain in the
untrimmed RMSE (0.03386975527905494 A) and maximum-error
(0.5413756008579356 A) calculation.

Frozen-evaluation CI (2026-09-09): `cargo run -p xtask -- ci` completed with
exit code 0. Formatting, dependency boundaries, workspace all-target Clippy,
workspace tests, smokes, headless checks and OSS parity passed; fuzz covered 361
cases across nine boundaries and Behavior CI passed 10/10 seeds. Mobility passed
173 tests with one ignored long-training test (189.76 s); fixed CLI passed
(1.41 s), NCLT audit passed three tests and suspension CLI passed (2.24 s).
Evaluation module Git blob: `cf55ef425a47bc8514b636bf0c7a293d8b87eb49`.
Log: `E:\RNE-build\m3c-sensor\openmct-frozen-evaluation-v1-ci.log`, SHA-256
`c4417585ffb04ce17cb1cd7ad19d7f5fcdb29f4601df7e24028abf42220bf534`.
This is default-feature coverage, not a new MuJoCo-feature qualification.
Negative observations remain explicit: heading CEM score -10 equals baseline,
mobile CEM grasped but did not place, and flagship reports `cross_backend=false`.
Clutter PPO reported random -1.68 / trained -1.37; mobile clutter PPO reported
random -2.08 / trained -1.61. Passing smoke execution does not certify learning
quality or physical fidelity. The separately recorded real-current error remains
unqualified and is not overridden by this CI result.

Speed-dynamics protocol inspection (2026-09-09): the identification parent
[README](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/65d00740-e209-4d52-9105-76050d0a5e87/file_downloaded)
and [summary script](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/c21b66ea-b527-49ba-b4cc-57034abcba9e/file_downloaded)
were read without executing scripts. They specify APRBS, PWM-count input and
RPM output. Crucially, the script transcribes GUI coefficients/metrics rather
than fitting the raw log. It reconstructs row time as zero followed by cumulative
preceding `DT_ms`. GUI datapoint counts also differ from retained raw-log counts
(10 ms: GUI 6,000 versus raw 3,089). A local raw-log fit must not claim exact
reproduction of GUI training data or its reported accuracy.

The separate continuous-PI 10 ms
[validation README](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/62ccc3d6-a181-4e9d-bc18-5014324461fa/file_downloaded)
declares a 2026-05-05 capture with 2,084 samples and a different identified plant,
`32.96 / (s + 29.12)`, versus `32.87 / (s + 28.9)` in the earlier identification
folder. Its plotting wrapper delegates normalized-step analysis and disables
automatic displayed metrics; it is not independent motor-parameter identification.
The official listing gives `raw_data_10ms.txt` as 119,139 bytes, file ID
`caca4c05-a564-4ccb-a171-bcaa057dca3a`, server-declared SHA-256
`80b817f3869e7dbc089d1f1f1e9b0eed96899a7ecffcdb13ac80ed7b1cef27ea`.
This validation raw file is not yet downloaded. Before using it, inspect command
and speed row alignment and preserve closed-loop feedback confounding. A frozen
PWM-to-speed model can be checked on recorded inputs, but this alone neither
validates the deployed controller nor identifies resistance/torque constants.

The PI raw file was subsequently acquired under a 150,000-byte streaming bound,
matched the exact declared size/hash, and was stored create-new at
`E:\RoboSim-external-data\mobility-openmct-5xvg43r9r8-v1\pi-validation-10ms-raw.txt`.
The strict Rust reader accepted 2,084 rows with two missing DMM rows, 983 distinct
IDs and 1,099 reused-ID rows, ages 0.8–33.5 ms. All declared intervals are 10 ms.

A read-only preliminary arithmetic check froze the earlier GUI model
`32.87/(s+28.9)` for both logs. The explicit assumed convention holds preceding-row
PWM over the preceding declared interval, using exact scalar zero-order-hold
propagation: `a=exp(-28.9*dt_s)`, `b=(32.87/28.9)*(1-a)` and
`speed[k+1]=a*speed[k]+b*PWM[k]`. Free-running prediction initializes once from
the first measured speed; one-step prediction uses the preceding measured speed.
No coefficients were fitted, no delay was optimized and no residuals were removed.
Over 3,088 identification transitions, free-running/one-step/persistence RMSE was
3.4710961826986626 / 3.721353187124806 / 6.00792869913467 RPM. Over 2,083 separate
PI transitions it was 3.4978654689956237 / 3.5552266569535935 /
5.954540540502496 RPM. These preliminary numbers are not a certified clock
alignment, independent GUI fit reproduction, physical parameter identification,
or replay of the closed-loop controller. A tested reusable response evaluator
must preserve the model and alignment assumptions with per-transition evidence.

`recorded_openmct::response::evaluate_speed_response` now implements this explicit
preceding-row ZOH convention with a supplied steady-state gain and positive time
constant. It retains all transition indices, durations, PWM inputs, measured speeds,
free-running/one-step predictions and residuals, plus persistence residuals and
untrimmed aggregate RMSE. `exp_m1` avoids cancellation in short-interval input gain.
It does not fit parameters, infer timing, substitute REF for PWM, or qualify
physical constants. The first ten focused tests passed. The read-only
`openmct_speed_response` example now emits the model, alignment convention,
initial speed, every transition and aggregate errors as JSON. With gain
1.1373702422145329 RPM/PWM-count and time constant 0.03460207612456748 s,
Rust exactly reproduced the preliminary reported RMSE values above for both
retained real logs. Additional tiny-interval, signed-input, invalid-coefficient,
insufficient-data and overflow tests passed: eleven focused tests total, with
all-target crate Clippy passing. The subsequent `cargo run -p xtask -- ci`
completed with exit code zero on 2026-09-09. Mobility library tests passed
175/175 with one additional ignored test (171.38 s), plus the fixed CLI,
three NCLT audit tests and suspension CLI test. Workspace Clippy/tests,
smokes, headless checks, OSS parity, fuzz-smoke (361 cases across nine boundaries)
and Behavior CI (10/10 seeds) passed. This was the default CI configuration,
not a new MuJoCo-feature or cross-backend qualification run.

Evidence log: `E:\RNE-build\m3c-sensor\openmct-speed-response-v1-ci.log`, SHA-256
`e118c0940ca3fce9824383afc789c6245867a8caf4706a404dd374873b7cd617`;
response source Git blob `95cc2aa7d04c1553ea9bfb82d4d54aae7f686e61`.
Retain limitations despite the successful process: heading CEM score/baseline
both -10; clutter PPO random/trained -1.39/-1.37; mobile CEM grasped but did
not place; mobile PPO random/trained -1.88/-1.61; flagship evidence reports
`cross_backend=false`. These smoke outcomes do not establish the full Mobility
Foundation goal or independently validated physical motor constants.

Validation-method follow-up (2026-09-09): MathWorks' primary documentation on
[model validation](https://www.mathworks.com/help/ident/ug/validating-models-after-estimation.html)
distinguishes estimation from independent validation data and warns about
reusing estimation data. Its
[residual analysis guidance](https://www.mathworks.com/help/ident/ug/what-is-residual-analysis.html)
also checks residual autocorrelation and correlation with past inputs; feedback
can create correlation with future inputs. Consequently the next OpenMCT
identification slice must retain signed lag conventions and separate the PI
capture's closed-loop provenance from the identification capture. Aggregate RMSE
alone is insufficient. The already-inspected PI capture is a development
evaluation, not an untouched final test for subsequent model selection. Reserve
a further capture before tuning; record training-only preprocessing and frozen
coefficients. These are remaining requirements, not implemented diagnostics or
an assertion that independent physical validation has been achieved.

### Reserved OpenMCT speed-response test capture

Before any new RNE speed-model fitting, reserve the dataset-v1 continuous-PI
20 ms capture as the next untouched raw-response test (2026-09-09). The official
[file listing](https://data.mendeley.com/public-api/datasets/5xvg43r9r8/files?folder_id=2456a5ce-aa48-4e77-bb9d-58257f896078&version=1)
identifies `raw_data_20ms.txt`, file ID
`4b6053d0-e7e7-4294-be7d-a8b7f11af401`, 56,531 bytes, publisher SHA-256
`40de82b4d4e18099add91af937c5223b09d0140ff3f94bdb31ebec44ac27b356`.
The bytes have not yet been downloaded or analyzed; this is a publisher digest,
not a locally verified acquisition hash. Do not inspect its response or replace
this capture based on model performance before freezing the next model.

Only its [README](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/aab0f579-fc57-48a6-aae9-a7ccec37d1fc/file_downloaded)
and metadata were inspected: declared acquisition 2026-05-05 19:29:59,
980 rows, duration 19.580 s, 20 ms PI control, reference 0–230 RPM.
The README exposes author model/tuning information, so this is not a fully
blinded experiment. Those coefficients must not enter RNE fitting or selection.
Different control rate/controller introduce a distribution shift, not proof of
independent hardware, timing accuracy or unbiased closed-loop identification.

The next fixed procedure is to fit only `identification-10ms-raw.txt`, using
the explicit preceding-row PWM convention, and report the already-inspected
10 ms PI log as development evaluation. Freeze coefficients, preprocessing,
model-selection rules and continuous-time propagation before opening the
reserved 20 ms raw log. Preserve every scored transition and baseline; report
errors in RPM and lag durations in seconds. Do not treat a discrete 10 ms pole
as a 20 ms pole. Structural changes after the reserved evaluation consume that
test and require a new untouched capture for a subsequent final test. This
reservation sets no physical-qualification threshold and authorizes no claim
that a fitted PWM model identifies motor electrical or tire parameters.

The initial implementation in `recorded_openmct::identification` fixes a
zero-offset two-regressor OLS model before inspecting reserved raw responses.
It uses every adjacent training transition with exactly uniform declared DT,
without centering or fitting a delay. The report retains the unmodified pole,
input gain and normalized determinant. Only poles strictly between zero and one
can be converted to the existing continuous first-order evaluator; other fitted
poles remain evidence rather than being clamped. Synthetic tests cover known
coefficient recovery, deterministic repetition, an unstable fit, clock mismatch
and absent excitation; the focused synthetic test passed. The
`openmct_speed_identification` example accepts only a training path and prints
the fitted coefficients, source hash and continuous-model conversion outcome.
An unrealizable pole remains in its JSON with a conversion error, not a silently
substituted model. The first real training-log run and reserved-log evaluation
described below have now completed.

The first training-only run completed after all-target Clippy passed. On the
3,088 transitions of source SHA-256
`198770b1bb27f13617531b0fbbc6ade83b69e3032562f19dc33c8bb7f19e3f24`,
the zero-offset OLS fit returned pole `0.7305068777047915`, discrete PWM gain
`0.3059294116463335` RPM/count and normalized determinant
`0.03751919709285456`. At declared interval 0.01 s, conversion gives gain
`1.1352030398431168` RPM/count and time constant `0.031845446885272785` s.
Freeze these coefficients for the development PI evaluation; do not replace
them with the author's GUI coefficients. This fit is empirical and potentially
biased by measured-speed noise; the result does not qualify motor constants.

Development PI evaluation with those unchanged coefficients returned free-run
RMSE 3.18574072389137 RPM, one-step RMSE 3.51293598321567 RPM and persistence
RMSE 5.9545405405025 RPM. Persistence uses the preceding measurement, so its
observation budget matches one-step prediction, not the uncorrected free run.
All 13 OpenMCT tests and all-target crate Clippy passed after the boundary-test
expansion (2026-09-09).

Freeze for the reserved 20 ms capture: identification source Git blob
`afc671769e05d9b4ef89016f3737c5f01ac08a4e`, evaluator blob
`95cc2aa7d04c1553ea9bfb82d4d54aae7f686e61`, and the coefficients above. There
is one model candidate, no fitted offset/delay or preprocessing, no selection
using reserved errors, and no acceptance threshold optimized against that log.
Initialize the free run only from its first measurement, use each preceding
declared DT for exact continuous-model propagation, and retain all transitions.
Report all three RPM errors without calling this a physical-validation pass.

The reserved raw capture was subsequently acquired under a 60,000-byte streaming
cap to `E:\RoboSim-external-data\mobility-openmct-5xvg43r9r8-v1\pi-reserved-20ms-raw.txt`.
Its 56,531 bytes match the publisher SHA-256 above. Both frozen source blobs
were rechecked before evaluation. The first reserved evaluation completed with
979 transitions, all declared intervals 0.02 s, no dropped transitions and no
coefficient change: free-run RMSE 3.82074878375445 RPM, one-step RMSE
3.92807772783069 RPM, persistence RMSE 10.0218352010364 RPM. These values describe
this capture, not confidence bounds or an independently calibrated accuracy
guarantee. This capture is now consumed for this frozen model; subsequent tuning
cannot claim it as an untouched test. The residual-lag API below is diagnostic
only and does not add a statistical acceptance gate. The physical-qualification
flag remains false.

The tested `diagnose_one_step_residual_lags` API and read-only
`openmct_speed_residuals` example now center all 979 one-step residuals and
preceding-row PWM inputs by their respective full-record means. At
transition-index lag one, Rust reproduces residual autocorrelation
-0.230733112785153 and residual/past-input correlation 0.202307041918555.
The selected input is 0.04 s before the target measurement because each
transition already carries a one-row-old input. Numerators use the 978
overlapping pairs; denominators use full-record centered energies (their
geometric mean for cross-correlation). Lags zero through five report input ages
0.02, 0.04, 0.06, 0.08, 0.10 and 0.12 s, with residual/input correlations
0.0273660, 0.2023070, 0.1296043, 0.0881915, 0.0648940 and 0.0537172.
Nonuniform captures retain per-lag minimum, mean and maximum input age rather
than pretending row lag is fixed time. Tests reject invalid intervals,
constant residuals, constant PWM and out-of-bound work. These are descriptive
correlations, not whiteness-test p-values or confidence-qualified rejection
thresholds. No model was changed after seeing the reserved-capture diagnostics.
All 14 OpenMCT tests and all-target mobility-benchmark Clippy passed after this
addition (2026-09-12). The subsequent default `xtask ci` log reaches its final
successful stage: Mobility completed with 178 passed, zero failed and one
ignored (163.14 s), followed by doc-tests, smoke/headless checks, OSS parity,
fuzz-smoke (361 cases across nine boundaries) and Behavior CI (10/10 seeds).
The response source blob is `56b8f836a16e9712e2533f0edd3b97c974b40461`
and CLI blob is `060dfce08d648c1e40c6f37417060cc1ccb77912`.
Evidence log `E:\RNE-build\m3c-sensor\openmct-speed-residuals-v1-ci.log`
is 246,131 bytes with SHA-256
`b079eae86be2b804df978a83251f36073267eaee63b8cdaa7ed65adaf9e3ca92`.
No cargo/xtask process remains after reconnecting; the interactive exit-code
record was not retained. The log reaches Behavior CI without a later failure.
Negative evidence remains visible: heading CEM equals its -10 baseline, clutter
PPO reports random/trained -1.45/-1.37, mobile CEM grasps but does not place,
mobile PPO reports -2.21/-1.61 and flagship reports `cross_backend=false`.
This default run is not MuJoCo-feature or physical qualification.

Identification CI evidence: workspace Clippy passed; the Mobility library
completed with 177 passed, zero failed and one ignored (201.94 s), followed by
the fixed CLI (1.82 s), three NCLT audit tests and suspension CLI (2.60 s).
The saved run continued through doc-tests, all smoke/headless checks, OSS parity,
fuzz-smoke (361 cases across nine boundaries) and the final Behavior CI (10/10
seeds). No cargo/xtask process remains after reconnecting to the task. Evidence
log `E:\RNE-build\m3c-sensor\openmct-speed-identification-v1-ci.log` is 246,235
bytes with SHA-256
`0e20c76764b5440beb06f9517f6f7987928bd6e1df95608ba792984a972fc311`.
The original interactive session's exit-code record was not retained across the
reconnection; the log itself reaches the CI script's final successful stage and
contains no subsequent failure. This was the default configuration, not a new
MuJoCo-feature or cross-backend qualification run.

Next acquisition gate: obtain an explicit file listing and bounded individual raw
logs on external storage; inspect units, measured versus commanded voltage, motor
and load identity, clock construction, current-reference synchronization and
calibration residuals before choosing identifiable parameters. Freeze separate
excitation/validation runs before fitting. A PWM channel alone is not measured
terminal voltage; do not infer physical electrical constants from it without the
missing conversion and instrumentation evidence.

### PWM command / physical plant boundary

The backend-neutral motor plant continues to accept requested terminal voltage in
volts. `PwmMotorCommandFrontendSpec` now provides a separate, deterministic map
from signed controller counts and a runtime bus voltage to that request. It clamps
counts at an explicit full scale, applies an explicit command polarity, and reports
ideal average voltage, averaged bridge loss and the post-loss voltage request.
`DcMotorSpec` then independently applies the motor-terminal voltage/current limits
and electrical failure mode. Thus an empirical RPM/PWM-count gain or time constant
cannot silently become resistance, torque constant, back-EMF constant or inertia.

The physical basis for keeping this boundary explicit is the TI
[DRV8874 data sheet](https://www.ti.com/lit/ds/symlink/drv8874.pdf): its input can
be static or PWM, and its PH/EN and PWM truth tables distinguish drive, brake,
coast and high-impedance states. TI's
[12–24 V brushed-DC reference design](https://www.ti.com/lit/ug/tiduaw3/tiduaw3.pdf)
also treats the motor-side voltage/current as PWM waveforms when reporting delivered
power. These sources do not qualify the OpenMCT capture's unmeasured bus voltage,
switch drops or decay behavior.

The implemented v1 map is intentionally only a switching-cycle average:
`V_request = signed_duty * max(V_bus - V_bridge_drop, 0)`. Bridge loss is declared
at an operating point and averaged over energized duty. PWM ripple, switching
frequency, decay/recirculation mode, current regulation, battery sag, dead time and
thermal dependence remain outside this contract. The frontend does not introduce
a synthetic command deadband. Focused tests cover count saturation, polarity,
loss greater than bus voltage, invalid evidence, and the handoff into the unchanged
voltage-driven motor evaluator. Physical use still requires measured bus voltage
and an identified bridge-loss model; the retained OpenMCT PWM/speed logs do not
supply them.

### PMDC geared-motor voltage/current candidate

The 2022 Data in Brief article
[Direct current geared motor data](https://pmc.ncbi.nlm.nih.gov/articles/PMC8752902/)
and its immutable Mendeley dataset v2
[2rkpsss6fd](https://data.mendeley.com/datasets/2rkpsss6fd/2) are the first inspected
candidate that materially closes OpenMCT's command/voltage gap. The authors declare
two identical-model PMDC worm-geared AGV motors, a VNH2SP30 H-bridge, ACS712 current
measurement, divided measurements of both motor terminals, an 1,800-pulse/revolution
incremental encoder, Arduino Mega 2560 acquisition at 100 Hz, and CC BY 4.0 data.
Every experimental table exposes source time in microseconds, encoder count, derived
RPM, raw and converted current, raw and converted voltages at both terminals, their
motor-terminal potential difference, motor state, and PWM command. Experiments include
no-load PRBS7/PRBS9 and periodic inputs plus step tests without load and with declared
1–4 kg shaft loads.

The official API reports one file, `MotorsData_rev.xlsx`, 28,724,353 bytes, file id
`d5996ce4-77b2-4246-aa60-b4ccf26e5770`, and SHA-256
`85203c4b3ad6fbdd05221e1be7fd41ce733376c0f316d7fd5542b604a6854605`.
It was acquired with a 30 MiB streaming cap and create-new destination at
`E:\RoboSim-external-data\mobility-pmdc-2rkpsss6fd-v2\MotorsData_rev.xlsx`;
local size and digest match. The archive has 37 ZIP members and 176,571,375
uncompressed bytes, so readers must enforce both compressed and expanded bounds and
must not expand it onto the internal SSD.

Read-only structure inspection found 23 sheets: one description sheet, paired Motor A/B
sheets for PRBS7, PRBS9, sine, triangle and square inputs, and paired loaded/unloaded
step sheets. The ordinary waveform/PRBS trials are vertically concatenated complete
runs; PRBS9 Motor A has eleven explicit `time` headers. Step trials form a sparse
two-dimensional grid of 13-column blocks: 122/152 no-load runs for A/B and 283/315
loaded runs for A/B. No source cells contain formulas except `StepNoLoad-MotorA!C256`,
whose expression is `(A256-B256)/1000000`. A strict importer must reject or explicitly
preserve that non-raw cell rather than silently trusting a spreadsheet cached value.

Before inspecting response values beyond structural header rows, freeze the first
identification split within the same physical Motor A: PRBS9 complete trials 1–8 are
training, trial 9 is development, and trials 10–11 are untouched final tests. Never
split adjacent rows from one run across these roles. PRBS7, periodic waveforms, Motor B,
and every step/load sheet remain outside initial model selection. This split does not
pre-authorize a model form or pass threshold.

This source is better evidence, not yet physical qualification. Terminal voltage and
current are Arduino/frontend measurements with source-derived conversion; their static
and dynamic calibration uncertainty, simultaneous ADC phase, anti-alias behavior and
clock accuracy still require audit. The concentrated load mass is not a measured torque;
shaft geometry, direction and load dynamics must be resolved before using loaded trials
to identify torque constant, friction, gearbox efficiency or inertia. The initial importer
must retain raw and converted channels, motor state, every sample clock and complete-run
identity, and must report formula cells and malformed blocks without repairing them.

`scripts/audit_pmdc_source.py` now makes that source boundary reproducible without
extracting or modifying the workbook. It verifies the published byte count and SHA-256
before XML parsing, rejects unsafe/duplicate/encrypted ZIP members, bounds the archive at
64 members, 64 MiB per expanded member and 256 MiB total expansion, requires the exact
23-sheet local-relationship layout, and preserves every detected trial header and formula
with its cached value. Synthetic contract tests cover traversal, expansion, external
relationships, incomplete channel headers, formula retention and same-size hash
substitution. Run it with bytecode disabled and place its output on external storage:

```powershell
python -B scripts/audit_pmdc_source.py `
  E:\RoboSim-external-data\mobility-pmdc-2rkpsss6fd-v2\MotorsData_rev.xlsx `
  --output E:\RoboSim-external-data\mobility-pmdc-2rkpsss6fd-v2\rne-pmdc-source-audit-v1.json
```

The first real-source run reports 37 members, 176,571,375 expanded bytes and audit
SHA-256 `a9c89c4f684510f69673b8da1e30422afeb66069297a9a591df1656249eeb63f`.
Its frozen PRBS9 Motor A header identities are `A2` through `A14079` for training,
`A16090` for development, and `A18101` plus `A20112` for untouched final evaluation.
It independently redetects only `StepNoLoad-MotorA!C256` as a formula. The report keeps
all physical/calibration/clock/identification qualification flags false; this is source
integrity evidence, not a motor fit or a claim that spreadsheet-derived channels are
ground truth.

`scripts/convert_pmdc_prbs9.py` performs the next lossless boundary step. Its CLI only
accepts `training` or `development`; there is deliberately no final-partition selector.
It stops before the first final annotation/header, validates each selected run annotation
and exact 12-channel header, rejects formula or incomplete data rows, and writes every
source cell as its original lexical string together with source row and stable run ID.
It applies no timestamp adjustment, numeric parsing, interpolation, unit conversion or
calibration. Exclusive output creation prevents accidental replacement:

```powershell
python -B scripts/convert_pmdc_prbs9.py `
  E:\RoboSim-external-data\mobility-pmdc-2rkpsss6fd-v2\MotorsData_rev.xlsx `
  --partition training `
  --output E:\RoboSim-external-data\mobility-pmdc-2rkpsss6fd-v2\prbs9-motor-a-training-v1.jsonl
```

The retained training artifact contains eight runs of 2,009 samples (16,072 records),
has file SHA-256 `37fe6d4bb5e2645aac171cc699e32bec64a3883f2a44a396049e005a1f2651b2`,
and its canonical record stream hashes to
`8de4971cbfc8bfd26e3244ead3357ad6950f56a8d57759a7ceab44f0feb12e5b`.
The separately created development artifact contains one 2,009-sample run, has file
SHA-256 `3a5a84afa45b49c5158287a222843cd66e3b338233b62f33d67db3ae7f23130f`,
and record-stream SHA-256
`a9e559ee901221a3663274c4f20939b202491426145580d8e1bd278aa0d04dec`.
Both live beside the source on external storage. The initially encountered single-cell
`Trial Description: PRBS9` rows are preserved as run annotations rather than silently
dropped or misclassified as samples. The final two runs remain sealed.

`scripts/audit_pmdc_channels.py` verifies the training JSONL record count and digest
before numeric inspection, then audits only source-internal timing and conversion
consistency. The first training audit is 4,060 bytes, has file SHA-256
`a2446c73a849fd7db66f5db25510eb7c5cd8e887e8ae8da5a7b09edbe65df860`,
and report audit SHA-256
`49a3ede7af3b6dd9f02efa33355d9dfd86327661c487ea462fd27eed745377be`.
All eight run clocks are strictly increasing. Their interval median is 10,004 us,
the global observed interval range is 8,888--10,040 us, and 1,767--1,779 of the
2,008 intervals per run are not exactly 10,000 us. The source therefore supports a
nominal 100 Hz acquisition statement but not an assumption of an exact 10 ms grid.

Across the training rows, ordinary least-squares reconstruction of the source-derived
voltage columns from their raw ADC columns has 2.764 mV RMS residual for A1 and
2.751 mV RMS for B1. `MotorVoltage - (VoltageB1 - VoltageA1)` has 4.669 mV RMS and
10 mV maximum absolute residual, consistent with finite displayed precision but not an
independent voltage calibration. The analogous raw-current to `Current` affine fit has
0.187 A RMS and 2.016 A maximum absolute residual, which is material and forbids treating
the displayed current as a single exact affine transform without further source-method
audit. These are self-consistency diagnostics only: ADC reference accuracy, divider and
sensor tolerances, sampling phase, anti-alias response and external instruments remain
unqualified, and every physical-accuracy flag stays false.

The [AutoDRIVE Nigel author repository](https://github.com/Tinker-Twins/AutoDRIVE-Nigel-Dataset)
is a separate Ackermann candidate with timestamp, steering, tick-count and inertial
columns. Its README declares approximately 1.50 GB for the camera-free dataset
and 66 GB for the full dataset. Neither was downloaded. The linked Zenodo record
returned HTTP 429. Physical versus simulator capture provenance and calibration
are not established by the inspected README, so it is not accepted as real-log
qualification. Do not clone the full repository merely to inspect its schema.

| Primary source | Potential RNE use | Evidence gap / decision |
| --- | --- | --- |
| [Michigan NCLT](https://robots.engin.umich.edu/nclt/index.html) | Wheel/IMU replay and estimator timing checks | First bounded format-inspection candidate; not independent drivetrain or suspension validation. |
| [Driving Data of a Real F1tenth Car](https://zenodo.org/records/12536536) | Velocity-command response identification candidate | One bag inspected with independent readers; electrical channels and reference timing/units remain unqualified. See the acquired evidence below. CC BY 4.0 declared. |
| [KAIST Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset/home) | Navigation sensor replay candidate | Official page lists LiDAR, stereo and position sensors, but does not establish the actuator channels needed here. It declares CC BY-NC-SA 4.0; do not bundle under RNE's license. Not selected for dynamics identification. |

NCLT's official update history says left/right wheel velocities were added to sensor
archives in August 2018. Its March 2019 update says the approximately 100 Hz
ground-truth poses use odometry interpolation between SLAM graph poses. Thus those
interpolated poses are not independent wheel-odometry reference measurements.
The page provides a script to inspect the original graph-node poses. Its 2013-01-10
entry lists a 21 MB sensor archive, separate from much larger images and LiDAR.
The page declares ODbL and Database Contents licensing. Verify the actual archive
and applicable terms before acquisition; listed size is not an enforced byte limit.

For NCLT, inspect original node timestamps and the reference construction first.
Even graph-node poses require an audit of shared estimator inputs before being
called independent truth. Do not differentiate an odometry-interpolated trajectory
and report the result as independently measured velocity or slip. Published wheel
velocity is not automatically raw encoder counts. A Segway capture also does not
establish skid-steer, trailing-caster, or Ackermann validity.

## Acquired DDMR format evidence

The author repository's [pinned CSV](https://github.com/RAI-Techno/ddmr_control/blob/6291b0d7faa5c7b7deb475e833c115c84d7123da/Data%20and%20Codes/Data.csv)
was acquired with a 20,000,000-byte streaming bound on 2026-09-08.
The exact 19,461,206 bytes matched Git blob SHA-1
`427680e90bfde9d41b09068c78dafc807273e907` (including the Git blob header).
Local SHA-256 is `c278dde8bfc38974bb2b1cc054160349da51456817c898ea2e052656aa75f60e`.
Original file: `E:\RoboSim-external-data\mobility-ddmr-6291b0d7\Data.csv`.
No notebooks or model weights were executed.

A complete read-only CSV scan found 338,550 rows, each with five finite numeric
fields: time (s), left/right voltage (V), left/right speed (rad/s). No malformed
rows or non-increasing times occurred. Times span 0 to 3385.4900000000002 s.
All adjacent differences match 0.01 s within 1 ns (decimal extrema
0.0099999999997 and 0.0100000000003 s). Both voltage columns range from
-5.808 to 10.664 V. Left speed ranges from -16.379961696524322 to
33.17992241090824 rad/s; right from -17.639958750103116 to
32.759923393048645 rad/s. Values and timestamps were not resampled or repaired.

These are file-format checks only. A regular time column does not establish
hardware capture timing. Commanded versus measured terminal voltage, encoder
processing, clock construction, capture instrumentation and dataset-specific
license scope still need qualification. There are no current or independent
body-reference columns in this CSV. Do not use these checks to claim electrical,
slip, suspension or real-world model accuracy. Keep raw data external and do not
bundle it into the repository.

### DDMR capture and evaluation qualification

Read-only source inspection on 2026-09-08 reached the following decision:
**retain as an unqualified command-to-wheel-response candidate, not an electrical
parameter or vehicle-slip calibration dataset.**

The pinned [identifier notebook](https://github.com/RAI-Techno/ddmr_control/blob/6291b0d7faa5c7b7deb475e833c115c84d7123da/Data%20and%20Codes/System_Identification_Model.ipynb),
code cell 4, consumes the first 336,000 rows, ignores the time column, constructs
150-row two-voltage windows, and targets the two speeds at each window's last row.
It prepends an artificial zero-input/zero-target example. It then splits the
constructed windows chronologically (60% training, then 67% of the remainder for
validation), without a boundary purge. Adjacent windows across a partition share
149 input rows. This is shared input history, not proof that target values leaked.
The reported R-squared is not independent-capture or free-running physical-model
validation. The notebook neither records hardware data nor establishes voltage
measurement or encoder decoding. No notebook code or weights were executed.

The complete GitHub file trees were inspected for `ddmr_control` at
`6291b0d7faa5c7b7deb475e833c115c84d7123da`, the linked
[exploration project](https://github.com/RAI-Techno/drl_autonomous_exploration/tree/2624fe1a9d04770552bc6acea9b6e963bdc00a4e),
and its linked [LilyBot project](https://github.com/RAI-Techno/lilybot/tree/eb059ec6564267c7e245cdde75a1617f18d1f7b7).
The trees were not truncated. No DDMR capture firmware or CSV recording script
was identified there. LilyBot's `lily_go.launch` starts Gazebo with simulated
time, not an instrumented real motor acquisition path. Its parts list links a
Yahboom ROS expansion board and DFRobot FIT0493 motor. This narrows hardware
candidates but does not pin the components/firmware used for the CSV capture.
The DDMR root LICENSE identifies Apache License 2.0; no separate dataset license
file appeared in the inspected tree. Keep the data external pending redistribution
review rather than relabeling it as RNE-owned data.

The [motor vendor](https://www.dfrobot.com/product-1462.html) lists 34:1 gearing
and quadrature feedback with 374 pulses per output revolution. Whether acquisition
counts one or multiple edges, and whether speed uses a fixed interval, remain
unknown. Do not substitute a guessed 374 or 1496 counts/revolution into the recorded
sensor contract. The [board documentation](https://www.yahboom.net/study/ROS-Driver-Board)
lists encoder capture and PWM control tutorials, but does not bind a firmware
version, configuration or voltage conversion to this dataset.

The `recorded_ddmr` module now provides a bounded, source-hashed offline reader
preserving the five recorded columns and unknown capture/receipt semantics.
It retains the original time token alongside f64 seconds, accepts the exact
unquoted numeric header with optional UTF-8 BOM and LF/CRLF, and rejects malformed,
nonfinite, unordered or oversized input. Limits are 32 MiB, 500,000 records and
512 bytes per data line. No automatic time conversion or voltage calibration occurs.

`DdmrSeries::split` takes explicit raw-row cut indices before window construction.
Each read-only partition builds its own past-only prediction windows. Positive
history and future-horizon lengths are mandatory; a horizon of one targets the
next recorded row, not necessarily a fixed number of seconds. Short partitions
fail instead of yielding an empty evaluation. No warm-up history crosses a split.
The `ddmr_source_check` example reports source integrity and window counts only:

```text
cargo run -p rne_mobility_benchmark --example ddmr_source_check -- <Data.csv> 203130 270840 150 1
```

For the pinned 338,550-row source these explicit cuts are 60%/20%/20%. They are an
ingestion smoke configuration, not a tuned model or a frozen performance result.
Synthetic tests cover source spelling/negative zero, line-ending hash differences,
disjoint histories and future labels, split/window boundary errors, byte/row/line
bounds, invalid numeric input and I/O errors.

On 2026-09-08 the three focused reader/split tests and crate all-target Clippy
(`-D warnings`, default features) passed. The example read all 338,550 real rows,
reported the exact byte count and SHA-256 recorded above, and generated 202,980
training windows plus 67,560 each for validation and test. It fitted no model and
reported all physical-calibration, timing and voltage qualification flags false.
This implementation postdates the full CI for commit `3dbc897`; that earlier
full CI is not regression evidence for this reader.
The subsequent MuJoCo-enabled crate library regression completed with 133 passed,
0 failed and 2 intentionally ignored long-job tests. This run covered the reader
and split, before adding the response identifier.

The exploratory response identifier and its fixed protocol are described in
[`MOBILITY_DDMR_RESPONSE_EXPERIMENT.md`](MOBILITY_DDMR_RESPONSE_EXPERIMENT.md).
Any response fit must use raw-row-disjoint chronological training/validation/test
segments, build windows only inside each segment, and disclose that one capture is
not independent-session validation. Freeze model choice on training/validation
only and compare held-out SI-unit errors with a persistence baseline. Do not fit
motor resistance, torque constant, current dynamics or tire slip from these five
columns. Physical parameter acceptance still needs a qualified acquisition source.

## Acquired NCLT archive

Acquired directly from the official page's
[2013-01-10 sensor archive](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/sensor_data/2013-01-10_sen.tar.gz):

- external path: `E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10_sen.tar.gz`;
- exact length: 21,669,014 bytes; download enforced a 32 MiB streaming cap;
- SHA-256: `3ec1a5ac27ee716e6e5da0cd4e8fee96241d6a21ac63d14a844c40d2410d5ffb`;
- no filesystem extraction, images, or LiDAR acquisition; listing shows regular
  sensor CSV files, one README and their containing directory;
- checksum records the bytes received, not a publisher signature or attestation.

The archive README specifies `wheels.csv` as timestamp, left speed, right speed
(m/s), and `kvh.csv` as timestamp plus heading (rad). KVH heading must not be
silently treated as angular velocity. The README does not itself specify the wheel
timestamp unit, capture/receipt distinction, calibration uncertainty or raw encoder
counts. Resolve these before publishing a normalized sensor contract.

A complete in-memory scan of `wheels.csv` via `tar -xOf` found:

| Check | Observed value |
| --- | --- |
| Rows, each with three parseable fields | 42,276 |
| First / last integer timestamp | 1357847237325466 / 1357848263256859 |
| Minimum / maximum adjacent timestamp difference | 14 / 201267 (source timestamp units) |
| Non-increasing timestamps | 0 |
| Non-finite wheel speeds | 0 |
| Maximum absolute wheel speed | 2.334337 m/s |

These checks establish format and ordering only. They do not prove physical
accuracy, sensor jitter, latency, dropout behavior or synchronization. Retain the
irregular timestamps instead of assigning a constant sample period. The initial
three-row preview closed its pipe early and `tar` reported a broken pipe; the
subsequent complete scan checked `tar` exit status zero before computing this table.

Official scripts were read as text, not executed:

- [read_ground_truth.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_ground_truth.py)
  obtains graph-node times from the covariance file and samples the trajectory at
  those times. It labels position NED, not RNE's world frame.
- [read_odom.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_odom.py)
  reads timestamp, position and roll/pitch/heading; it is not a wheel-count reader.
- [read_ms25.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_ms25.py)
  identifies timestamp, magnetic field, acceleration and rotation-rate columns,
  with plot units microseconds, Gauss, m/s squared and rad/s respectively.

The paper's Sections 3, 4 and 7 and Tables 4/8 were inspected in the
[author-hosted manuscript](https://s3.us-east-2.amazonaws.com/publications.perl.engin.umich.edu/ncarlevaris-2015a.pdf).
UTIME denotes Unix microseconds. The body axes are forward/right/down, with its
origin at the axle center; the IMU has identity mounting rotation but a nonzero
lever arm. Microstrain values use its internal filter. Odometry fuses wheel,
FOG and IMU inputs: it is not an independent reference for their validation.
`odometry_mu.csv` represents relative image-event motion, whereas the 100 Hz file
is relative to the run start. Do not integrate or compare these interchangeably.

The 2018 wheel addition inherits the dataset timestamp convention; its README does
not independently document acquisition-clock semantics. No measured arrival latency
or synchronization uncertainty is inferred. A source-time replay must explicitly
distinguish its scheduling policy from physical sensor latency.

## Executable ingestion

`rne_mobility_benchmark::recorded_nclt` now reads wheel and `ms25` CSVs through
bounded `Read` inputs. Each requires nonempty input, exact column counts, 16-digit
integer timestamps, strictly increasing times and finite numeric values. Limits are
64 MiB per input, one million rows and 1,024 bytes per line. Invalid inputs fail as
a whole. The output binds parsed samples to SHA-256 of the exact uncompressed bytes,
including original line endings. Synthetic tests do not include redistributed data.

Values remain in source coordinates and units (including magnetic field in Gauss).
No resampling, gravity compensation, lever-arm correction, encoder-count synthesis,
or measurement-noise estimation occurs. The additive
[source-time DataBus replay](MOBILITY_RECORDED_REPLAY_V1.md) preserves that boundary;
measured-velocity state estimation remains separate follow-up work.

The CLI is read-only and reports timestamps, interval extrema and a source digest:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
cargo run -p rne_mobility_benchmark --bin rne-nclt-audit -- wheels E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\wheels.csv
cargo run -p rne_mobility_benchmark --bin rne-nclt-audit -- imu E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\ms25.csv
```

Only these two regular CSV members were subsequently extracted from the hash-checked
archive into a previously absent external directory: 1,483,062 and 8,951,816 bytes.
The initial no-extraction audit above describes the earlier inspection stage.
The default benchmark binary remains `rne-mobility-benchmark`; adding this audit
tool does not change existing `cargo run -p rne_mobility_benchmark` behavior.

Both real files passed the CLI. Wheel counts/times match the initial scan. IMU
has 48,324 rows spanning 1357847237276758 through 1357848263255151 microseconds,
with adjacent intervals from 3,134 to 72,076 microseconds. Both file digests match
an independent PowerShell `Get-FileHash -Algorithm SHA256` computation:

- wheels: `5387898ac211c17502c23e0a231ced7b479c3123717317b3527ff9bd66d0a029`;
- IMU: `9e504bbbc410c53a25c2da670fd2f03d3228154f1016026bd09e48123380498f`.

Passing the ingestion checks is not a sensor-quality or dynamics acceptance gate.

Validation of this ingestion slice: formatting and crate Clippy passed, the four
reader tests and one audit-summary test passed, and MuJoCo-enabled tests passed
(101 library tests plus one test in each CLI). The full `xtask ci` completed with
exit code zero, including headless, OSS parity, 361 fuzz cases and 10/10 Behavior
seeds. Logs are external: `E:\RNE-build\m3c-sensor\nclt-mujoco-tests.log` and
`E:\RNE-build\m3c-sensor\nclt-ci.log`. Two repeated audit invocations for each real
CSV produced identical output, with hashes independently checked by PowerShell.

Next connect qualified source-frame samples to explicit replay scheduling and
estimator inputs, with frame/time tests. No physical identification result is claimed.

### Replay and estimator boundary review

Inspection of `rne_ai::WheelImuOdometry::update` shows that it requires
`IncrementalEncoderFeedback` and integrates differences of `raw_count`, not a wheel
velocity field. Its `WheelImuOdometryConfig` also requires counter resolution and
width. NCLT cannot satisfy this input contract without invented measurements.
The older `WheelEncoderSample` also requires a realized angular position, which the
recorded wheel-speed file does not provide. Neither payload is an honest shortcut.

Similarly, `ImuFeedback` declares raw specific force, sample-phase error and known
saturation status. Internally filtered `ms25` values and unavailable status metadata
must not be relabeled nominal raw feedback. Keep the existing incremental-encoder
estimator unchanged until an explicit measured-velocity estimation path is designed.

The replay contract and remaining estimator requirements are:

- publish source-typed wheel-speed and filtered-IMU payloads through the existing
  extensible `FramePayload`/DataBus boundary, without adding NCLT dependencies to core;
- retain source Unix timestamps in payloads and use one shared integer origin for
  both streams, converting elapsed microseconds to nanosecond ticks with checked
  subtraction and multiplication; separate stream origins would erase real skew;
- declare replay availability delay as a chosen transport experiment, never measured
  latency; preserve original source values, frames, filtering and unknown quality;
- validate both streams before publishing, use deterministic tie ordering, and test
  missing inputs, distinct start times, ties, delayed availability and reset/replay;
- drive consumers through `latest_available`, never `latest`, and preserve source
  digests plus replay policy in evidence. Reconstruct from original bytes when
  provenance matters: public parsed structs can be modified after ingestion;
- add a separately identified wheel-speed/gyro integration path with explicit
  interpolation, gap, frame and uncertainty policies before claiming estimator replay.

This is an integration requirement, not an additional completed benchmark gate.

## Identification design

The F1TENTH record's [official API](https://zenodo.org/api/records/12536536)
lists nine bags, from 84,985,680 to 192,694,601 bytes, and declares CC BY 4.0.
The [README](https://zenodo.org/api/records/12536536/files/ReadMe.md/content)
describes separate continuous runs, body-frame `/cmd_vel` commands and world-frame
`/vrpn_client_node/Car_2_Tracking/twist` VICON measurements. Its opening paragraph
instead spells the command topic `/vel_cmd`; verify actual bag connections rather
than choosing from prose. That initial metadata inspection did not download a bag;
the subsequent bounded acquisition below supersedes the metadata-only status.

This supports investigating aggregate command-to-motion response. It does not yet
establish measured steering, motor voltage/current, wheel forces or clock uncertainty.
For longitudinal signed speed, a world-frame velocity norm loses reversal and
lateral-motion information. Require orientation/frame evidence for projection or
declare a narrower speed-magnitude metric, without calling it signed velocity.
Inspect measurement derivation and timestamp alignment before treating VICON output
as a qualified reference. Split by entire runs before identification.

[Gonultas et al., IROS 2023](https://arxiv.org/abs/2308.03898v2) reports
gradient-based identification of a front-steered vehicle and real F1TENTH lane-keeping
validation. This supports including a control-level validation experiment after
fitting; it does not establish that the separate Zenodo record above has the same
vehicle, measurements, or parameters. Neither source is evidence that RNE currently
matches real dynamics.

Before implementing a dataset-specific importer, resolve:

- immutable source/version, exact filenames, lengths, checksums and license;
- measured versus commanded steering and drive quantities, units and sign;
- clock domains, capture versus logging time, synchronization uncertainty,
  missing intervals and resampling already performed;
- measured versus estimated pose/velocity and all inputs used to build references;
- vehicle dimensions, drive topology, mass and available calibration artifacts;
- excitation sufficient for the chosen parameters, including confounded or
  unobservable parameters that must remain fixed or explicitly unidentified.

Without voltage/current and calibrated drive information, trajectory agreement
alone cannot identify motor electrical parameters. Do not manufacture missing
force, current, steering measurements or timestamp uncertainty from model output.
Without independent reference measurements, report replay consistency rather than
physical accuracy.

### F1TENTH bounded acquisition and preliminary byte audit

One run was acquired from the official API content link, with a 90,000,000-byte
download cap, into the previously absent external directory
`E:\RoboSim-external-data\mobility-f1tenth-12536536`. No ROS runtime, bagpy, image
extraction or additional package installation was needed. The bag itself includes
scan messages; they were not extracted into a separate dataset.

- File: `ex-hard-r2_2023-06-12-19-59-52.bag`, exactly 84,985,680 bytes.
- Official MD5, verified: `1f0e930d8d0bf2106fac9d34a94b9c21`.
- Locally computed SHA-256:
  `3ba7b5c13da68227bf8af27370e7205f142dca4ca3340c8b1b51feba70cc22ac`.
- Attribution: Giannis Badakis, Michalis Galanis and Zengjie Zhang,
  *Driving Data of a Real F1tenth Car*, Zenodo record 12536536, CC BY 4.0.

A read-only, bounded standard-library byte audit, following the record layout in
the [official ROS implementation](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/bag.py),
found ROS bag v2, 105 uncompressed chunks and 25 connections. Message definitions
were treated as text, not executed. Selected message counts from walking chunks
agree with the connection-index totals:

| Topic | Type | Messages |
| --- | --- | ---: |
| `/cmd_vel` | `geometry_msgs/Twist` | 9,853 |
| `/commands/motor/speed` | `std_msgs/Float64` | 9,855 |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 6,032 |
| `/vrpn_client_node/Car_2_Tracking/pose` | `geometry_msgs/PoseStamped` | 13,426 |
| `/vrpn_client_node/Car_2_Tracking/twist` | `geometry_msgs/TwistStamped` | 13,402 |

The VESC embedded definition includes input voltage, motor/input current, electrical
RPM and duty cycle. Thus telemetry is present, contrary to what the metadata alone
could establish. Presence is **not qualification**: preliminary decoding found input
voltage and PCB temperature identically zero, duty cycle from -0.194 to 0.569 despite
the embedded comment's 0-to-1 range, and fault values spanning 0 through 255 despite
only codes 0 through 6 being declared. Treat these as unresolved decoding/driver/
record-quality anomalies, not physical fault diagnoses or usable motor calibration.

VICON pose/twist report `world`; VESC's frame string is empty. For VICON pose,
`bag_time - header_stamp` ranges from -63.775767420 to -62.780972095 s; for VICON
twist, -63.775357363 to -62.906228216 s; for VESC, -63.775798446 to -63.445731478 s.
These are differences between recorded clock fields, not negative physical latency.
The unstamped command messages have no capture timestamp. Do not erase these
differences by independently zeroing each stream or fitting an unexplained offset.

Next, validate decoding through an independent reader and synthetic format fixtures,
inspect the acquisition/driver time conventions, and determine whether a justified
common-clock mapping exists. Verify VICON velocity derivation and orientation before
body projection. Electrical RPM needs pole-pair/transmission/sign calibration; servo
commands are not measured steering. Do not fit voltage-driven electrical parameters
from the zero-voltage channel. No parameter fit, held-out score or physical pass is
claimed from this preliminary inspection.

Independent reader check: the already available `rosbags` 0.11.3 ROS1 reader and
its generated typed deserializer reproduced all five selected message counts and
all three header-versus-bag timestamp ranges exactly. It independently confirmed
6,032/6,032 zero input-voltage values and 5,861/6,032 fault values outside codes
0 through 6 (observed range 0 through 255). This rules out the initial manual byte
parser as the sole explanation, not driver/firmware or recording defects.
The missing `lz4` 4.4.5 dependency (99 kB wheel) was installed without pip caching
only under the external acquisition directory's `reader-deps`; no existing Python
environment was modified. Calls used `-B` to avoid bytecode cache writes. Reader
agreement does not qualify the channel or resolve clock synchronization.

Reference angular velocity also needs qualification. The
[published Noetic driver source](https://docs.ros.org/en/noetic/api/vrpn_client_ros/html/vrpn__client__ros_8cpp_source.html)
converts `vel_quat` to roll/pitch/yaw and assigns these directly to twist angular
fields without dividing by `vel_quat_dt`. Its header stamp can use either server
time or ROS current time. The bag's exact deployed driver version and configuration
are not established, so this is a concrete investigation lead, not proof that the
same defect produced this capture. Do not treat the angular fields as calibrated
rad/s, silently apply a guessed sample-rate multiplier, or infer synchronization
from the topic/type names. Independently checked pose differences and documented
server/driver conventions are required before selecting a yaw-rate reference.

The independent audit is reproducible with `scripts/audit_f1tenth_source.py`.
It verifies the pinned size and SHA-256 before parsing; it rejects different
captures instead of silently assuming their schemas or calibration. It checks
decoded counts against the source connection index and prints a stable JSON digest.
Only the five listed channels are deserialized; no dataset is exported or fitted.
Use the external dependency directory and disable bytecode writes:

```powershell
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = $env:TEMP
$env:PYTHONPATH = 'E:\RoboSim-external-data\mobility-f1tenth-12536536\reader-deps'
python -B scripts/audit_f1tenth_source.py E:\RoboSim-external-data\mobility-f1tenth-12536536\ex-hard-r2_2023-06-12-19-59-52.bag
python -B -m unittest discover -s scripts -p test_audit_f1tenth_source.py -v
```

The chosen Python must provide `rosbags==0.11.3` and its dependencies. Synthetic
audit tests do not import rosbags and do not require physical data. The captured
report is `E:\RNE-build\m3c-sensor\f1tenth-independent-audit.json`, with digest
`f0427731ef2388e97dc29fa1906385fa18db57786a4409c9e97673193d418710`.

### Pose/twist consistency diagnostic

`scripts/audit_f1tenth_reference.py` uses the same pinned source and independent
reader, preserving header timestamps and the declared `world` frame. It compares
successive pose differences with the latest twist at or before the interval end.
The policy was fixed before evaluating: pose intervals 1..50 ms and twist age at
most 20 ms. These are retrospective interval averages versus endpoint samples,
not identical measurements or online observations. No command-time mapping is used.

There are 13,225 paired intervals, 148 excluded pose intervals and 52 missing/stale
twist pairs. World-planar velocity-difference RMS is 0.850751351 m/s. Pose-heading
rate RMS is 1.466426217 rad/s, while recorded angular-z RMS is 0.012536505 in its
unqualified units. The unscaled angular difference RMS is 1.455188522. A diagnostic
through-origin scale is 104.952102266; it is neither applied nor accepted as a
calibration. Timing, finite-difference noise, interval-versus-point comparison and
driver conventions may contribute. These results do not establish independent
accuracy and do not identify the capture's exact driver version or fault cause.

Run with the same external environment as above:

```powershell
python -B scripts/audit_f1tenth_reference.py E:\RoboSim-external-data\mobility-f1tenth-12536536\ex-hard-r2_2023-06-12-19-59-52.bag
python -B -m unittest discover -s scripts -p 'test_audit_f1tenth*.py' -v
```

The initial eight synthetic audit tests passed. Schema-v1 evidence is
`E:\RNE-build\m3c-sensor\f1tenth-reference-audit.json`, digest
`ef74cece68e453d8d24ede31e27a350bfaf738b5caeb6f36a25d365b93eecacb`.
Next, distinguish sample/clock jitter from velocity derivation effects with
interval-integrated comparisons; do not select a larger smoothing window merely
to make residuals pass. Independent reference calibration and command-clock
qualification remain prerequisites for physical identification.

The schema-v2 diagnostic additionally integrates zero-order-held world-frame
twist over each complete pose interval, splitting at source events and the same
20 ms expiry. Any positive-duration uncovered part rejects the entire pair; an
observation at the endpoint cannot fill an earlier gap. The original endpoint
diagnostic remains in the report without changing its policy or metrics.

For 13,194 fully observed intervals (148 interval exclusions, 83 coverage exclusions),
planar displacement-difference RMS is 0.003055050 m. This has different units and
a different accepted population from the 0.850751351 m/s endpoint comparison;
do not describe the two numbers as an accuracy improvement. Pose-heading increment
RMS is 0.012544661 rad, whereas integrated recorded angular-z RMS is 0.000127213
in unqualified integrated units. The exploratory through-origin angular scale is
95.680780098 and remains unapplied. The angular discrepancy persists with the
interval-integrated comparison; its cause and a valid correction remain unproven.

All 11 synthetic audit tests passed. Schema-v2 evidence is
`E:\RNE-build\m3c-sensor\f1tenth-reference-integral-audit.json`, digest
`c9050e373ff763e0e9ea390adbf5ca185d5383f6b9a5925e1f6beb72b32fa49d`.
Neither diagnostic performs clock synchronization, independent reference
qualification, electrical identification or an acceptance-threshold fit.

The existing [suspension identification gate](MOBILITY_SUSPENSION_IDENTIFICATION_V1.md)
requires strut displacement, velocity and generalized force plus acquisition evidence.
None of these candidate descriptions establishes that contract. Keep its physical
dataset status pending; do not relax it to accept generic driving trajectories.

## Bounded execution plan

1. Inspect NCLT's official format-reading scripts as text, without running them.
   Resolve exact sensor archive URL and licensing; inspect file listing before extraction.
2. Acquire only selected small sensor/reference files under an explicitly validated
   external SSD root. Set a hard streaming byte cap, use create-new destinations,
   hash original bytes, and record final URL and acquisition metadata. Reject path
   traversal, links and extraction-size overruns. No camera/LiDAR download for this slice.
3. Produce a channel/timestamp audit before normalization. Implement the offline
   importer outside core crates, with malformed-input, unit, ordering and deterministic
   replay tests. Preserve original timestamps; leave unavailable latency unknown.
4. Revisit the F1TENTH record metadata when available. Select a dynamic-model experiment
   only after verifying input/output semantics and identifiability. Split by complete
   runs before fitting; reserve a separate final test, not adjacent time samples reused
   across tuning and evaluation. Freeze SI metrics and acceptance budgets in advance.
5. Feed validated parameters to both backends and report real-reference residuals
   separately from Rapier/MuJoCo agreement. Agreement between backends is not a
   substitute for agreement with independently measured behavior.

Raw captures, converted datasets, caches and reports stay on the external SSD and
are not committed. CI uses small synthetic format fixtures clearly labeled synthetic;
passing those tests never changes physical-validation status. This plan adds no ROS
dependencies to core crates and does not require rendering.
