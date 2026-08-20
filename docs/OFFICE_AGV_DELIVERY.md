# Office AGV delivery

This slice scores a short **analytic office aisle**: stop on the pickup dock,
then stop in the delivery box in front of the desk, without leaving the
corridor. It is not a warehouse twin, Nav2 port, or photoreal interior.

## What is in v1

| Check | Judge |
|---|---|
| Remain inside the corridor half-width | `no_corridor_exit` |
| Stop on the pickup dock long enough to count a visit | `dock_pickup_complete` |
| After pickup, stop in the 1.2 m box before the desk face | `delivery_complete` via `evaluate_office_desk_delivery_stop` |
| Entering/passing the desk region without pickup fails | `no_desk_without_pickup` |
| Passing the desk face without a valid delivery stop fails | `no_desk_overshoot` |

## Run

```bash
cargo test -p rne_ai office_agv_delivery
cargo run --locked -p office_agv_delivery --example 84_office_agv_delivery -- --smoke
```

`--smoke` requires a clean scripted success and a skip-dock failure on
`no_desk_without_pickup`.

## Assets

- `assets/scenes/office_agv_delivery.rne.scene.toml`
- `assets/robots/office_agv_delivery.rne.robot.toml`
- `assets/tasks/office_agv_delivery.task.json` (`rne.office.agv_delivery.v1`)

## Not in this slice

- Shared-aisle traffic / other AGVs (D-2)
- Shelf or desk manipulation / G1 carry (D-3)
- Full office floor plans, elevators, or multi-floor routing
- Perception of packages (pickup/delivery remain geometric)
