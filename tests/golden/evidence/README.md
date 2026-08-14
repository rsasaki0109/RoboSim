# Evidence JSON goldens

These fixtures freeze the schema-v1 JSON shape of RNE's trust and evidence
contracts. Tests deserialize and reserialize each fixture with the owning Rust
type. A field addition, removal, rename, ordering change, enum-shape change, or
unknown-field policy change therefore requires an explicit schema decision.

The fixture values are illustrative and are not benchmark baselines. Runtime
evidence is generated under `artifacts/` by `cargo run --locked -p xtask --
evidence` and contains the actual source revision, hashes, and run metadata.
