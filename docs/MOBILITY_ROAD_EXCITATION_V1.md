# Rigid-road excitation benchmark v1

Status: implemented additive M3-C road/contact subgate

This fixture drives the same explicit four-station Ackermann plant through Rapier and
MuJoCo over one backend-neutral metric road profile. It adds grade, short-wave
roughness, a 40 mm curb, a 40 mm drop, solved wheel contact, suspension response, and
debounced wheel lift/recontact evidence to the existing suspension benchmark. Both
backends receive the same `mobility_ackermann_road_excitation_v1` TaskSpec, seed,
1 ms simulation clock, road solids, force laws, sampling, and verdict code.

## Road contract

`RigidRoadProfileSpec` is an ordered list of finite `RigidRoadPatchSpec` values. Every
patch declares the exact driving-surface center, length, half-width, thickness, grade,
and friction scale in SI units. A pure geometry function reconstructs the cuboid below
that surface, while a deterministic query returns the closest finite patch's surface
point, tangent, normal, and friction. A gap returns no sample. Adjacent surface ends
need not meet: their elevation discontinuity is the curb or drop, represented by real
solid faces rather than a scripted force.

This small contract follows established road-model separation without pretending to be
an importer. [ISO 8608:2016](https://www.iso.org/standard/71202.html) standardizes the
reporting of measured vertical road profiles. The
[ASAM OpenCRG specification](https://asam-ev.github.io/OpenCRG/asamopencrg/latest/specification/06_opencrg_data_format/6_0_Open_crg_data_format.html)
represents road elevation and friction on a curved regular grid. Project Chrono's
[official rigid-terrain API](https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/terrain/RigidTerrain.h)
likewise separates box, mesh, and height-map patches and exposes height, normal, and
friction queries. RNE v1 deliberately implements only the compact finite-patch subset
needed for deterministic backend conformance; it is not ISO roughness classification,
OpenCRG ingestion, or scanned-road validation.

## Maneuver and evidence

After a 1.5 s gravity settle, the four independent motors drive across:

- 6 m of level road;
- a +0.08 rad grade and level transition;
- eight 250 mm alternating +0.06/-0.06 rad roughness patches;
- a 40 mm raised platform with an explicit vertical curb face;
- a 40 mm drop and level runout.

The trace retains 20 Hz chassis height, vertical velocity/acceleration, all four
suspension coordinates, conditioned wheel loads, backend contact flags, and canonical
road-patch indices. A contact-state change must persist for 5 ms before it counts as a
wheel lift or recontact event. Contacts whose road-to-wheel normal has
`abs(normal.x) > 0.20` are classified as curb-face impacts and excluded from the
explicit tire traction law; the rigid-body backend still resolves the collision.

Peak constraint load is retained and bounded inside each backend trace, but it is not
used as a cross-solver parity metric because contact stabilization can concentrate the
same impact into different numbers of fixed steps. Cross-backend curb parity therefore
uses integrated normal impulse:

```text
J_curb = sum(F_normal * dt)
```

This measures the momentum transfer. The same comparison also bounds forward travel,
suspension velocity, RMS vertical acceleration, and lift/recontact event-count gaps.
The trace and comparison carry FNV-1a content digests and reject mutation.

Generate the external-SSD evidence with:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = 'E:\RNE-build\tmp'
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend road-excitation-compare `
  --output E:\RNE-build\m3c-sensor\road-excitation-comparison-v1.json
```

The verified comparison artifact is stored outside the repository at
`E:\RNE-build\m3c-sensor\road-excitation-comparison-v1.json` with digest
`fnv1a64:ac7696c88f20ac7f`. Both individual traces and the comparison pass; the
measured table is recorded below.

| metric | Rapier | MuJoCo | absolute gap |
| --- | ---: | ---: | ---: |
| forward displacement | 12.593020 m | 12.774353 m | 0.181333 m |
| curb normal impulse | 1103.715 N s | 1143.984 N s | 40.269 N s |
| peak curb-face load (non-parity evidence) | 21015.45 N | 6556.18 N | — |
| maximum suspension velocity | 0.793838 m/s | 0.550252 m/s | 0.243585 m/s |
| RMS vertical acceleration | 13.52598 m/s² | 4.21907 m/s² | 9.30691 m/s² |
| wheel lift events | 6 | 8 | 2 |
| wheel recontact events | 2 | 4 | 2 |

## Explicit limits

The road is an authored chain of rigid cuboids, not measured road data. The roughness
sequence is a deterministic excitation and has no claimed ISO 8608 class. Sphere wheel
contact and unidentified tire/suspension parameters remain model limitations. The high
Rapier single-step vertical-acceleration peak is retained openly rather than filtered
away. Parameter identification against real logs, measured-profile import, randomized
road/material distributions, and recorded/shadow/HIL validation remain later M3-C
through M5 gates.
