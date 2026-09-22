# Full-contact measured-effort screening

Producer `eed9b4152b12326b52831dba175fff1ea8f1d43e`, +0.02 rad landing-knee
candidate, 500 µs, direct joint effort, full contacts and structural masks.

The run completed 15 seconds, peak joint speed 1.031872x, minimum final-second
upright 0.9999897, minimum base height 0.780605 m, maximum base speed
0.002044 m/s, source standing error 0.004656, continuous positive foot support,
and no nonadjacent negative solver gaps or nonfoot ground contacts. All 23
joints have 32,000 finite realized-effort samples including preparation.

**Rejected:** reported peak effort reaches 120.00001526 Nm on knees with a
120 Nm ceiling (also small positive excess on some other saturated joints).
The f32 force accumulation/projection can exceed the commanded scalar by a few
ULPs. No tolerance is added to the measured torque gate. The next candidate
commands 0.001 Nm below each rating, leaving the physical rating and audit
unchanged. The 125/62.5 µs runs were explicitly cancelled after this coarse
failure; partial logs are not completed validation.

Use `python3 scripts/g1_native_audit.py step-500us.json.gz` to reproduce the
failed `measured_effort_limits` gate. No final native success GIF is claimed.
