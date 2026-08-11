# Controller Plugin ABI Compatibility

RNE's controller plugin boundary is a versioned C ABI. ABI v2 is both the
current authoring ABI and the oldest supported controller ABI. The compatibility
gate builds an independently defined v2 plugin, loads it with the latest
`rne_plugin` runtime, and executes a controller step.

## Frozen ABI v2 contract

ABI v2 consists of these exported symbols:

- `rne_plugin_abi_version`
- `rne_plugin_name`
- `rne_controller_create`
- `rne_controller_destroy`
- `rne_controller_step`

Joint observations and commands cross the boundary through the v2
`RneJointPosition` and `RneJointVelocity` C layouts. Rust types, allocators, and
RNE crate dependencies do not cross the shared-library boundary.

The compatibility consumer lives in
[`crates/rne_plugin_abi_v2_fixture`](../crates/rne_plugin_abi_v2_fixture). It
owns a copy of the v2 layouts and exports, has no dependency on `rne_plugin` or
another RNE crate, and includes its own `rne-plugin.json` manifest. It is not an
authoring template. Its ABI number, layouts, symbols, and behavior are frozen.

## Support policy

| Runtime | Controller ABI | Status |
|---|---:|---|
| Latest | 2 | Required and tested |

- A breaking symbol-signature or C-layout change requires a new ABI number.
- Adding a newer authoring ABI must not rewrite the v2 fixture. The runtime must
  retain a v2 loading/adapter path so this fixture continues to pass unchanged.
- A source-only maintenance change to keep the fixture buildable is allowed
  only when its exported symbols, C layouts, ABI number, and observable commands
  remain identical.
- Retiring v2 requires an explicit compatibility-policy revision, migration
  guidance, and a release note; it must not happen as an incidental refactor.

`RNE_PLUGIN_ABI_VERSION` identifies the current authoring ABI. While v2 is the
only supported version, the loader can compare directly against that constant.
Before introducing v3, the loader must dispatch v2 libraries to preserved v2
types and adapt their commands to the current robot-native interface.

Run the focused compatibility gate with:

```bash
cargo test -p rne_plugin --test abi_compatibility
```
