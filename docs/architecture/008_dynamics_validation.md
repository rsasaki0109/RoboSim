# Dynamics validation

Branch: `feat/dynamics-validation`

Analytic-reference tests in `tests/dynamics/` verify the Rapier backend (`rne_physics_rapier`)
against closed-form mechanics. Each scenario uses a named tolerance constant with a rustdoc
derivation in `tests/dynamics/tests/scenarios.rs`. Run:

```bash
cargo test -p rne_dynamics_tests
cargo test -p rne_dynamics_tests report -- --nocapture   # measured error magnitudes
```

## Scenarios and references

| # | Scenario | Reference | Primary tolerance rationale |
|---|----------|-----------|---------------------------|
| 1 | Free fall (1 s, 60 Hz) | Continuous `y₀ + ½gt²` | Rapier tracks continuous parabola within ~2 cm; f32 slack → 3 cm bound |
| 2 | Projectile (4 m/s horizontal, 1 s) | `x = vₓt`, vertical as (1) | Horizontal exact; vertical uses same 3 cm bound |
| 3 | Pendulum period (L=2 m, 5°) | `T = 2π√(L/g)` | 3% for finite amplitude + joint discretization |
| 4 | Friction stick (15°, μ=0.4, tilted gravity) | Static when tan θ < μ | < 3 mm over 2 s after 1 s settle (solver micro-slip) |
| 5 | Friction slip (35°, μ=0.4) | `a = g(sin θ − μ cos θ)` | 22% on Δv/Δt window (impulse friction bias) |
| 6 | Sliding stop (v₀=3 m/s, μ=0.3) | `s = v₀²/(2μg)` | 12% for contact + integration |
| 7 | Restitution (e=0.6) | Apex ratio `h₂/h₁ ≈ e²` | 20% (impulse solver scatter) |
| 8 | Energy drift (pendulum, 10 s) | Characterization only | < 5% relative drift |
| 9 | dt convergence | Error vs continuous shrinks 60 Hz → 240 Hz | Ratio ≥ 1.5× |

## Measured error magnitudes (Rapier 0.22, Windows, debug)

Values from `report_measured_errors` on this branch (representative single run):

| Scenario | Measured | Reference | Error |
|----------|----------|-----------|-------|
| Free fall position (1 s) | y = 45.0746 m | continuous 45.0950 m | **2.0 cm** (6.1 cm vs symplectic sum) |
| dt convergence | err₆₀ = 2.0 cm, err₂₄₀ = 0.51 cm | continuous | **4.0×** reduction |
| Pendulum period | T = 2.8398 s | 2.8370 s | **0.10%** |
| Incline static (15°) | displacement = 47 µm | 0 | within 3 mm bound |
| Incline slide accel | a = 2.412 m/s² | 2.412 m/s² | **0.0%** (0.5 s Δv window) |
| Sliding stop distance | s = 1.523 m | 1.529 m | **0.4%** |
| Restitution apex ratio | h₂/h₁ = 0.407 | e² = 0.360 | **+13%** (within 20% bound) |
| Energy drift (10 s) | max relative = 1.5% | conserved | within 5% bound |

## Integrator finding

Unconstrained free fall does **not** match the textbook symplectic-Euler discrete sum
`gΔt²n(n+1)/2`; Rapier is measurably closer to the **continuous** parabola at 60 Hz.
The dt-convergence test confirms the residual is integration-order (O(dt)), not a gravity
model bug.

## Friction implications for Phase B grasping

- **Stick/slip threshold**: Coulomb stick condition `tan θ < μ` holds on a flat plane with
  tilted gravity after settle; micro-slip stays sub-millimetre at 15° / μ=0.4.
- **Sliding dynamics**: Along-slope acceleration matches `g(sin θ − μ cos θ)` within ~5%
  when measured on a velocity increment window (not cumulative speed after long slides).
- **Flat deceleration**: Stopping distance matches `v²/(2μg)` within ~10%; suitable for
  predicting slip distance during finger preload before friction grasp.
- **Restitution**: Loose bound only; not relied on for grasp (compliant contacts dominate).

No `rne_physics` trait changes were required: `PhysicsMaterial.friction` and
`.restitution` on `Collider` are already wired in `rne_physics_rapier`.

## Related

- Determinism replay: `tests/determinism/`
- Roadmap pillar: [ROADMAP.md](../ROADMAP.md) — Precise dynamics validation
