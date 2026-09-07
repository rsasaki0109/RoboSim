# IPIN 2024 static IMU source audit

Status: source acquired and inspected on 2026-09-08. Bounded irregular statistics
pass focused tests and the six-axis diagnostic is reproducible. Physical noise
profiles, thermal calibration and complete regression remain pending.
These measurements are not evidence of drivetrain accuracy.

## Source and storage

Ortiz, M.; Ziyou L.; Zhu, N.; Renaudin, V., *Datasets and Supporting Materials for
the IPIN 2024 Competition Track 4*, Zenodo 2024,
[DOI 10.5281/zenodo.14501047](https://doi.org/10.5281/zenodo.14501047).
The record API declares open access and CC BY 4.0. Preserve this attribution;
do not silently bundle the data or label it as RNE-generated measurements.

All acquired files reside in `E:\RoboSim-external-data\mobility-ipin-14501047`.
The outer archive is `2024_IPIN_Competition_Track04.zip`, 493,132,939 bytes.
Its size and published MD5 `a56df8e682866f3e6b7cc9d424769962` matched after download.
SHA-256: `dadb0e388c00432cf27562f15e27b7c672343b2bb8772da08e91a6dc1a0032a2`.
An initial transfer timed out at 489,572,388 bytes; the remaining range was resumed
only after the process exited. The complete archive was then checked, not accepted
on the basis of a successful partial transfer.

Only the nested `IPIN2024_T4_ULISS_AllanVariance.zip` and technical annex were
extracted. Nested archive: 403,694,339 bytes, SHA-256
`efc4ca484aa8f1c0136a6dd10a16f2b66ce81372906f5fe0875afc7bcde093d2`.
CSV inputs were streamed directly from that ZIP; no gigabyte-scale CSV extraction
or package installation was required. ZIP metadata alone was not treated as proof
that its data parsed correctly.

| Source entry | Uncompressed bytes | SHA-256 of exact uncompressed bytes |
| --- | ---: | --- |
| acceleration_Allan.csv | 551407735 | `1357f596e2089f0ad3be543c337a00624802af248fbb2bd7c12b709773cd53cc` |
| rotation_Allan.csv | 532273603 | `abc06f17b266bf441f4b516ad3790c49b2aebbfe0fe506a39654c27e785d5512` |
| temperature_Allan.csv | 1430055 | `3276c2797d84b646f192d041495569edb526cdbd35a5ef76f474f693c0587b8c` |

## Observed format and timing

A read-only C# streaming audit checked every acceleration and rotation row for
four columns, finite numeric values, decimal timestamp precision no finer than
six places, and timestamp differences. Bounds were 20 million rows, 1,024 line
characters, and 10,000 distinct time deltas. This was an exploratory source audit,
not a checked-in Rust reader, CI test, or physical calibration certificate.

| Entry | Rows | First timestamp | Last timestamp | Delta range, in millionths of source time | Distinct deltas |
| --- | ---: | ---: | ---: | ---: | ---: |
| acceleration | 10663829 | 259553.004897 | 312518.997670 | 4960-4980 | 21 |
| rotation | 10426395 | 259553.002818 | 312518.999172 | 5079-5082 | 4 |

Both streams passed with no nonfinite measurements, repeated timestamps or
backward timestamps. They are not uniformly spaced, and do not share the same
sample times. Absence of repeated/backward timestamps does not prove absence of
physical acquisition faults or establish the meaning of the recorded clock.

The headerless CSVs contain time and source X/Y/Z values. The trial-format annex
defines GPS time-of-week in seconds, acceleration in m/s^2, gyro rate in rad/s and
temperature in Celsius. Its example trial records include an extra sensor label
that the static CSVs lack. Applying those units to the static export is consistent
with its naming and values, but its export code and capture-versus-receipt semantics
remain unverified. Do not manufacture an absolute GPS week, arrival latency,
fault status, saturation status, coordinate rotation or gravity correction.

Annex `IPIN2024_Track4_CallForCompetition_v1.4.pdf`, SHA-256
`81babd8457c70c7957cdc17b0adb86d2e62dc68fa64a08510850f16ae0961b82`,
was visually inspected on pages 2-4. Page 2 names MTi-7; page 3's output tables name
MTi-1. Preserve this discrepancy instead of assigning a device-specific profile.

All 52,965 temperature rows were also checked for finite values. First value:
41.0781; last: 36.4727; minimum: 36.0586; maximum: 52.5625. Under the annex's unit
convention this is a 16.5039 degree Celsius range. Temperature sensor location and
its relationship to IMU die temperature are unknown. Static recording does not
mean constant-temperature noise; no temperature/bias causal fit is claimed.

## Analysis decision and remaining implementation

The existing uniform-series Allan API accepts at most one million records and
requires exact capture periods. Neither full static stream satisfies that contract.
Do not truncate to fit the cap, snap timestamps, combine independently computed
chunk variances (which loses cross-boundary pairs), or synthesize nominal
`ImuFeedback` status to make the physical data pass a simulation-only contract.

[Maddipatla et al., FAVAR and D-FAVAR (2021)](https://doi.org/10.1016/j.ifacol.2021.11.148)
provides a candidate irregular statistic: differences of sample means in adjacent
half-open time windows, weighted by the product of their sample counts. Equations
4-7 and Algorithm 2 were visually inspected on printed pages 27-28. The fast
algorithm truncates its input length; that behavior is not selected for RNE.
Local paper SHA-256:
`938000ce9ed80e11bd2bc388dbe042a4a00ca815284330189f8111b1f7e7e8a7`.

`rne_sensor::allan::timed::weighted_time_deviation` now implements one explicit
duration and endpoint grid over a fallible ordered stream. It retains at most
one million samples, accepts at most 20 million records and one million endpoints,
and fails rather than silently reducing those requests. Compensated rolling sums
use a constant offset for numerical conditioning. Full-stream validation includes
records after the last evaluation endpoint. The source must cover both ends of
the evaluation domain, including a record at or beyond the final endpoint; this
coverage witness is not itself included in the half-open final window. Never add
a synthetic witness to real measurements. First Rust tests bridge the uniform
formula and check the hand-computed irregular fixture and trailing failures;
all 90 sensor tests and sensor all-target Clippy passed. Additional tests compare
an independent direct-window oracle across four durations, irregular sample counts,
a long empty interval followed by recovery, and offsets 0, 9.81 and 1e9. Validation
also covers zero/overflowing grids, incomplete source coverage, zero total weight,
trailing NaN, arithmetic overflow, and the retained/total sample limits. The
20-million-record limit is checked even after evaluation is complete. Full
regression and connection to the physical source remain pending.

The evaluation contract must declare the endpoint grid, window durations,
coverage and empty-window accounting. A zero total weight must fail; missing data
must not become fabricated measurements. Weighted sample means are not
time-weighted continuous-signal integrals. The statistic is not itself a fitted
noise model or a confidence interval.

An independent JavaScript direct-window prototype matched the uniform formula
in 1,600 synthetic cases. An irregular hand fixture with times 0,1,3,4,8,9,
values twice those times, duration 2 and endpoints 4 through 10 yielded variance
10.4, weight 5, three valid pairs and four empty-window pairs. Separate rolling
sum prototypes checked 1.2 million ramp samples and offset/noise fixtures. These
are design experiments only; none is a Rust test or physical-data validation.

`recorded_ipin::audit_ipin_imu` now provides a streaming Rust format audit, with
a one-GiB byte cap, 20-million-row cap and 512-byte line cap. It accepts the
headerless three-axis format only, preserving exact-byte SHA-256 and integer
microsecond source coordinates. It reports interval and per-axis extrema, not
physical calibration or sensor identity. The `ipin_source_check` example accepts
a CSV path or `-` for decompressed stdin, so ZIP entries need not be extracted.
Two Rust tests and all-target benchmark-crate Clippy passed. The compiled CLI
then read both entire ZIP entry streams through stdin without CSV extraction.
Both runs exited successfully; exact hashes, byte counts, row counts, first/last
times and minimum/maximum intervals matched the independent audit above.
Acceleration source-axis extrema were [-0.287936,-0.150897,9.66878] through
[-0.0307083,0.159058,9.99879]; rotation extrema were
[-0.0111828,-0.0190631,-0.00939941] through [0.01465,0.0147979,0.0168089].
These are measurements in source coordinates, not inferred turn-on bias or
calibrated limits. Full regression for this reader remains pending: the successful
`6d7e7d1` IMU CI predates this addition.

Remaining gates: validated source reader and replayable source audit;
strict source identity and unknown-status handling; irregular and long-series
implementation with independent oracle tests; fixed thermal/temporal holdouts;
physical-profile validation without tuning on held-out results; complete regression
for those future changes. Keep the existing uniform API strict and ROS-free.

## Source-to-statistic connection

`analyze_ipin_axis` now shares the same bounded parser as the format audit and
returns the full source audit, selected axis, explicit grid and statistic together.
All three axes and trailing records are validated even if only one axis is analyzed.
Conversion from source microseconds to nanosecond ticks is exact within the bounded
GPS-week coordinate. It does not establish capture timing. The
`ipin_time_deviation` example accepts a file or decompressed stdin and emits JSON
with explicit false calibration/capture-qualification flags.

First physical-source diagnostic protocol, declared before execution: each source
axis separately; 1-second adjacent windows; first endpoint 259556 seconds;
1-second endpoint spacing; 52963 endpoints, ending at 312518 seconds. Both source
streams cover this common interior domain. Validate all source rows and exact
hashes, without time snapping, filtering, thermal correction or profile fitting.
This is one diagnostic duration, not a full Allan curve or noise calibration.
The checked-in `tests/mobility_benchmark/ipin_diagnostic.ps1` ran this protocol
twice on all six source axes. All twelve processes succeeded; each second JSON
output matched its first output byte-for-byte. Each source SHA-256 matched the
audited values above. All axes had 52,963 valid pairs and zero empty-window pairs.
Total sample-count weights were 2,146,858,723 for acceleration and 2,052,325,709
for rotation; these are not independent degrees of freedom.

| Source | X deviation | Y deviation | Z deviation |
| --- | ---: | ---: | ---: |
| acceleration, source units | 0.0018654833471657623 | 0.0017290158780127072 | 0.001822889747619953 |
| rotation, source units | 0.0001493700091269584 | 0.0002086545716780081 | 0.00014812964128606542 |

Evidence: `E:\RoboSim-external-data\mobility-ipin-14501047\time-window-diagnostic-v1.json`,
SHA-256 `4988e5e8d98845d9a14504b73ae3b95717898d4d544a36978b23dcf455a709d0`.
It binds executable and source-code hashes, explicit grids, source audits and
repeatability. This was an uncommitted development build, not evidence for the
earlier frozen CI commit. No thermal correction, profile fitting, confidence
interval, or physical noise-density interpretation follows from this one-duration
result. Complete regression for the source reader and timed statistic remains pending.

To reproduce without extracting the CSVs, build `ipin_time_deviation` in the
external Cargo target, then run from the repository root (PowerShell 7):

```powershell
tests/mobility_benchmark/ipin_diagnostic.ps1 -Archive E:\RoboSim-external-data\mobility-ipin-14501047\IPIN2024_T4_ULISS_AllanVariance.zip -Executable E:\RNE-build\m3c-sensor\debug\examples\ipin_time_deviation.exe -OutputPath E:\RoboSim-external-data\mobility-ipin-14501047\time-window-diagnostic-new.json
```

Choose a new output filename; the script refuses overwriting evidence. The script
contains the fixed grid and expected input hashes, validates both complete passes,
and binds the executable and source files used. A changed build may produce a
different evidence-file hash even when its numerical outputs agree.
