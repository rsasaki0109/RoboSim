# Office AGV desk place

This slice extends [OFFICE_AGV_SHARED_AISLE.md](OFFICE_AGV_SHARED_AISLE.md)
with a **kinematic cargo unload** into a desk place box after the delivery
stop. It is not friction grasp, G1 Dex3 carry, or a warehouse twin — place is a
planar region judge so the mission stays headless.

## What is in v1

| Check | Judge |
|---|---|
| Shared-aisle yield + dock + desk stop | reused aisle / delivery geometry |
| Load cargo at the dock | `cargo_loaded` after dock pickup |
| Unload into the desk place box after the desk stop | `desk_place_complete` via `evaluate_office_desk_place` |
| Dropping cargo before the desk stop fails | `no_early_drop` |
| Mission success requires yield, dock, desk stop, and place | `mission_complete` |

Mission order: dock load → yield → desk stop → place unload.

## Run

```bash
cargo test -p rne_ai office_agv_desk_place
cargo run --locked -p office_agv_desk_place --example 86_office_agv_desk_place -- --smoke
```

`--smoke` requires a clean scripted success and a skip-place failure on
`desk_place_complete`.

## Assets

- Reuses `assets/scenes/office_agv_delivery.rne.scene.toml`
- `assets/tasks/office_agv_desk_place.task.json` (`rne.office.agv_desk_place.v1`)

## Not in this slice

- Friction grasp / finger contact place (mobile-lift track)
- G1 Dex3 carry handoff
- Perception of packages
