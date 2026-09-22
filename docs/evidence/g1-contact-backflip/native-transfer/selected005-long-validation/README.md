# Selected +0.005 rad candidate: long validation

The candidate selected by the unchanged five-second diagnostic loss in
`../fine-knee-refinement/` is evaluated for 15 seconds with the same immutable
producer and full-contact model. The 500 µs rollout passes every recorded
audit gate, including measured physical effort and joint-speed limits.
The 125 and 62.5 µs trials are still running; this is not qualification.
No final success GIF has been generated.

Producer source: `0c84bfdec8bdbdd3a4d9724e5bf13469fc895243`.
Model: `target/research/native-convex-full-model/scene.rne.scene.toml`.

The `model-*` files preserve the declared input scene, robot configuration,
URDF and preparation audit. `model-provenance.json` verifies all 30 source
URDF/mesh files against the immutable producer commit and the generated URDF
against its preparation hash. This snapshot was taken after the coarse run
and during the finer runs, not as a pre-run attestation. The model preparation
audit predates the candidate armature override; see the rollout for the applied
0.01 kg m² armature. Absolute mesh paths require regeneration on another checkout.

The renderer also verified all 1,600 coarse-recording frames with zero physics
ticks (`500-render-verify.log`); the render manifest checks the raw rollout and
visual URDF hashes. This preflight produced no replacement GIF.
