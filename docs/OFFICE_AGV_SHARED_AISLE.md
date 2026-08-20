# Office AGV shared aisle

This slice extends [OFFICE_AGV_DELIVERY.md](OFFICE_AGV_DELIVERY.md) with a
**kinematic oncoming AGV** on the analytic corridor. It is not `rne_traffic`
co-simulation and not a warehouse twin: the opposing actor is a planar
footprint so yield and collision judges stay headless.

## What is in v1

| Check | Judge |
|---|---|
| All dock-to-desk geometric checks from delivery v1 | reused course / desk stop box |
| Stop while the shared segment is occupied by the oncoming AGV | `yielded_for_shared_aisle` |
| No footprint overlap with the oncoming AGV | `no_other_agv_contact` |
| Delivery success requires the yield plus dock and desk stops | `delivery_complete` |

Mission order: pickup dock → yield at the shared segment → desk delivery box.
The oncoming AGV enters the shared segment from the desk side, then reverses
back out so the scripted success path does not require a head-on pass.

## Run

```bash
cargo test -p rne_ai office_agv_shared_aisle
cargo run --locked -p office_agv_shared_aisle --example 85_office_agv_shared_aisle -- --smoke
```

`--smoke` requires a clean scripted success and an ignore-yield failure on
`no_other_agv_contact`.

## Assets

- Reuses `assets/scenes/office_agv_delivery.rne.scene.toml`
- `assets/tasks/office_agv_shared_aisle.task.json` (`rne.office.agv_shared_aisle.v1`)

## Not in this slice

- `rne_traffic` ECS actors, signals, or route catalogs (flagship remains the
  traffic-runtime reference)
- Desk cargo unload — see [OFFICE_AGV_DESK_PLACE.md](OFFICE_AGV_DESK_PLACE.md)
- Multi-AGV fleets or full office floor plans
