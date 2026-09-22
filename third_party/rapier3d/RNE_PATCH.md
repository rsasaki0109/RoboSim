# RNE Rapier armature and free-root COM patch

Based on the crates.io rapier3d 0.22.0 package (Apache-2.0, LICENSE retained).
The implementation change is confined to
`src/dynamics/joint/multibody_joint/multibody.rs`. `Cargo.toml` adds an empty
workspace so upstream tests run independently in nested research worktrees.

Adds a validated generalized-coordinate armature setter and a zero-initialized
armature vector. Armature follows damping's coordinate remapping on append,
split, growth and root dynamic/fixed transitions. Its diagonal is added to
both mass matrices before permutation/factorization. It introduces no force,
damping, link mass or external wrench. Zero armature skips the new arithmetic.
RNE exposes only a revolute multibody component, not Rapier types in core APIs.
Its backend feature `experimental-armature` gates calls to the patched setter;
default backend builds remain compatible with the unmodified upstream crate.

Source provenance is recorded in `RNE_UPSTREAM_SHA256.json`. Backend tests
exercise analytic acceleration, torque-limited implicit motor impulses,
invalid inputs and retained spatial mass. Changes to upstream should preserve
this contract or replace the patch with equivalent upstream support.

Run patch tests with `cargo test --manifest-path third_party/rapier3d/Cargo.toml --release --lib rne_armature_tests`.

The same module also fixes free-root translation coordinates for bodies with
nonzero local centers of mass. Rapier's root generalized linear velocity is a
COM velocity, so its integrated translation must locate that COM. The free
joint now places its second frame at the local COM and preserves body pose
when rebasing coordinates, reading a body pose, or changing root type.
This numerical correction applies to repository builds regardless of the
armature feature. Unmodified upstream remains API-compatible but does not
contain the correction. Zero COM retains the previous coordinate convention.
See `docs/adr/031-multibody-free-root-com.md` in the engine repository.
