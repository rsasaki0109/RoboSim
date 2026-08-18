# Controller plugin SDK

`rne_plugin_sdk` is the dependency-free Rust authoring surface for RNE
controller plugins. It contains only version constants, capability bits,
`#[repr(C)]` frames, and callback signatures. It has no dependency on the RNE
host, ECS, renderer, physics backend, ROS 2, or a vendor SDK, and no Rust object
or allocator ownership crosses the shared-library boundary.

SDK schema v1 describes controller ABI v3 while retaining the ABI-v2 base
frames used by compatibility fixtures. Host-facing loading, lifecycle state,
manifests, discovery, and conformance remain in `rne_plugin`; plugin authors do
not need those implementation APIs.

## Start offline

The installed CLI vendors the exact SDK module into every new scaffold:

```bash
rne-asset plugin new my_controller --dir plugins
cargo build --offline --manifest-path plugins/my_controller/Cargo.toml
rne-asset plugin check \
  --library plugins/my_controller/target/debug/libmy_controller.so \
  --manifest plugins/my_controller/rne-plugin.json \
  --output plugins/my_controller/conformance.json
```

Use `my_controller.dll` on Windows or `libmy_controller.dylib` on macOS. The
generated crate has no registry dependencies. Its `src/rne_plugin_sdk.rs` must
remain byte-for-byte compatible with the SDK version declared by the release;
regenerating the scaffold is the simplest upgrade path.

Authors who manage dependencies directly may instead use the exact matching
`rne_plugin_sdk` crate version and import the same ABI items. The release bundle
also carries the canonical module at `sdk/rust/rne_plugin_sdk.rs` for auditing
and non-Cargo build systems. C and C++ authors use the dependency-free canonical
header at `sdk/c/rne_plugin_sdk.h`; it declares all ABI-v2/v3 structures,
callback typedefs, capability bits, calling convention, and required exports.

The installed compatibility corpus retains
`controller-c-abi-layout-v3.json` for the two tier-1 64-bit targets. It freezes
pointer width, structure size/alignment/field offsets, capability values, ten
required symbol names, their first ABI version, and normalized C signatures.
Both the Rust layout tests and installed corpus must pass before release.

## ABI ownership rules

| Value | Owner | Validity |
|---|---|---|
| Observation arrays and their strings | Host | Duration of the callback |
| Output array storage | Host | Duration of the callback |
| Command string pointers | Plugin | Until the host copies the callback result |
| Opaque controller handle | Plugin | From successful create through destroy |
| Error buffer | Host | Writable only up to the declared capacity |

Every exported callback is `unsafe` because the shared-library boundary cannot
prove pointer validity. A plugin must validate configuration, produce no more
commands than the supplied capacity, return deterministic results after an
identical seeded reset, accept exactly one shutdown, and destroy only handles
that it created. `rne-asset plugin check` exercises those observable contracts.

## Compatibility

- `RNE_PLUGIN_SDK_VERSION` versions the Rust authoring surface.
- `RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION` versions the layout fixture.
- `RNE_PLUGIN_ABI_VERSION` selects the exported C ABI.
- Capability bits are additive within one ABI version; unknown bits fail
  closed in the host.
- Existing `rne_plugin::cabi` type paths are re-exports of the SDK definitions,
  so extracting the SDK does not break host applications.
- ABI v2 remains a frozen, independently authored fixture and does not depend
  on the new SDK crate.

See [COMPATIBILITY.md](COMPATIBILITY.md) for the support window and
[OSS_PARITY.md](OSS_PARITY.md) for the complete authoring workflow.

For eventual independent 1.0 certification, retain the exact shared library,
`rne-plugin.json`, conformance report, external repository URL, and lowercase
40-character tested commit. Readiness manifest v4 rehashes both files and
requires their names, the library size, and the negotiated controller identity
to match the report; a report alone is not certification evidence.
