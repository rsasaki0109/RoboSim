# Traffic Models

Standard, deterministic microscopic traffic models provided as pure functions.
The runtime's longitudinal integrator keeps its original kinematic rule as the
default so recorded traffic replays stay reproducible; these models are opt-in
building blocks for a runtime or adapter that wants standard dynamics.

## Car-following

`rne_traffic::car_following`:

- `IdmParams` + `idm_acceleration` implement the Intelligent Driver Model. The
  acceleration is `a [1 - (v/v0)^delta - (s*/s)^2]` with the desired gap
  `s* = s0 + max(0, vT + v dv / (2 sqrt(a b)))` and `dv = v - v_lead`. A negative
  result brakes.
- `KraussParams` + `krauss_safe_speed` / `krauss_new_speed` implement the Krauss
  safe-velocity model: `v_safe = -b tau + sqrt((b tau)^2 + v_lead^2 + 2 b gap)`,
  and the next speed is the acceleration-limited, desired-limited, safe-limited
  minimum.

All parameters and observables carry SI units in their names, and `is_valid`
rejects degenerate configurations.

## Lane change

`rne_traffic::lane_change` implements MOBIL:

- `mobil_incentive` scores a candidate change as the changing vehicle's
  acceleration gain plus a politeness-weighted change for the new and old
  followers.
- `mobil_safe` requires the new follower to accept no more than
  `safe_braking_m_s2` deceleration.
- `mobil_should_change` combines the safety criterion with a minimum incentive.
- `mobil_idm_decision` composes the two models: it computes IDM accelerations for
  the subject and for the old/new followers before and after a candidate change,
  then applies the MOBIL criterion. `MobilNeighbor` describes a leader or
  follower on a lane.

Accelerations are supplied by the caller (for example from
`idm_acceleration`), so decisions are a pure function of the surrounding
traffic and are bit-for-bit reproducible.

## Runtime integration

`KinematicTrafficConfig::car_following` selects the longitudinal model for
runtime-owned actors. It defaults to `CarFollowingModel::Kinematic`, so recorded
replays are unchanged. `CarFollowingModel::Idm(IdmParams)` switches the runtime
to IDM: a same-route leader supplies its speed, a cross-route leader is treated
as stationary, and the IDM acceleration is still clamped by the red-signal and
junction-reservation control speed.

## Limits

- Autonomous MOBIL lane selection is not yet wired into the runtime. The
  OpenSCENARIO executor's lane change remains scripted (a route snap), and
  `mobil_idm_decision` is the decision API for a future lane-selection loop that
  knows each lane's leader/follower neighbours.
- Krauss is implemented without the optional random dawdle, so it is
  deterministic; add a seeded dawdle only behind an explicit seed.
- EIDM (the extended IDM with jerk limiting) is not implemented.
- IDM uses the same-route leader speed only; cross-route leaders are treated as
  stationary obstacles.
