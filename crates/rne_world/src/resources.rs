//! World-level ECS resources.

use bevy_ecs::prelude::Resource;
use rne_core::{mix64, DeterministicRng};
use serde::{Deserialize, Serialize};

const STREAM_DOMAIN_V1: u64 = 0x316D_7274_7345_4E52;

/// World random stream derivation algorithm version.
pub const WORLD_RANDOM_STREAM_VERSION: u32 = 1;

/// Stable identifier for a deterministic world random stream.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RandomStreamId(u64);

impl RandomStreamId {
    /// Creates a stream identifier from a stable numeric value.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric stream identifier.
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Serializable snapshot of world-level random state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorldRandomSnapshot {
    /// Root seed used to derive deterministic streams.
    pub seed: u64,
    /// Current internal state of the main world stream.
    pub main_rng_state: u64,
}

/// Seeded random source for world-level deterministic simulation behavior.
#[derive(Clone, Debug, Resource)]
pub struct WorldRandom {
    seed: u64,
    main: DeterministicRng,
}

impl WorldRandom {
    /// Creates a world random resource from the scene seed.
    pub fn new(seed: u64) -> Self {
        Self {
            seed,
            main: DeterministicRng::new(seed),
        }
    }

    /// Restores a world random resource from a snapshot.
    pub fn from_snapshot(snapshot: WorldRandomSnapshot) -> Self {
        Self {
            seed: snapshot.seed,
            main: DeterministicRng::from_state(snapshot.main_rng_state),
        }
    }

    /// Returns the root seed used by this resource.
    pub const fn seed(&self) -> u64 {
        self.seed
    }

    /// Returns a snapshot of the root seed and main stream state.
    pub fn snapshot(&self) -> WorldRandomSnapshot {
        WorldRandomSnapshot {
            seed: self.seed,
            main_rng_state: self.main.state(),
        }
    }

    /// Restores the root seed and main stream state from a snapshot.
    pub fn restore(&mut self, snapshot: WorldRandomSnapshot) {
        *self = Self::from_snapshot(snapshot);
    }

    /// Replaces the root seed and resets the main stream.
    pub fn reset(&mut self, seed: u64) {
        *self = Self::new(seed);
    }

    /// Returns the main world stream.
    ///
    /// Prefer derived streams for ongoing systems such as sensors, domain
    /// randomization, and agents so call-order changes do not shift unrelated
    /// random samples.
    pub fn main_stream_mut(&mut self) -> &mut DeterministicRng {
        &mut self.main
    }

    /// Returns the next value from the main world stream.
    pub fn next_u64(&mut self) -> u64 {
        self.main.next_u64()
    }

    /// Samples a floating-point value in `[min, max)` from the main world stream.
    ///
    /// # Panics
    ///
    /// Panics under the same conditions as [`DeterministicRng::uniform_f64`].
    pub fn uniform_f64(&mut self, min: f64, max: f64) -> f64 {
        self.main.uniform_f64(min, max)
    }

    /// Samples an index in `[0, len)` from the main world stream.
    ///
    /// # Panics
    ///
    /// Panics under the same conditions as [`DeterministicRng::uniform_usize`].
    pub fn uniform_usize(&mut self, len: usize) -> usize {
        self.main.uniform_usize(len)
    }

    /// Returns the deterministic seed for a derived world random stream.
    pub fn stream_seed(&self, stream_id: RandomStreamId) -> u64 {
        stream_seed(self.seed, stream_id.value())
    }

    /// Creates a derived deterministic stream from the root seed and stream id.
    ///
    /// Calling this repeatedly with the same stream id returns a new RNG at the
    /// same initial state each time and does not consume the main stream.
    pub fn stream(&self, stream_id: RandomStreamId) -> DeterministicRng {
        DeterministicRng::new(self.stream_seed(stream_id))
    }
}

impl Default for WorldRandom {
    fn default() -> Self {
        Self::new(0)
    }
}

fn stream_seed(root_seed: u64, stream_id: u64) -> u64 {
    mix64(root_seed ^ mix64(stream_id ^ STREAM_DOMAIN_V1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn main_stream_is_repeatable() {
        let mut first = WorldRandom::new(42);
        let mut second = WorldRandom::new(42);

        for _ in 0..8 {
            assert_eq!(first.next_u64(), second.next_u64());
        }
    }

    #[test]
    fn stream_ids_are_stable_and_distinct() {
        let world_random = WorldRandom::new(42);
        let mut stream_a = world_random.stream(RandomStreamId::new(7));
        let mut stream_b = world_random.stream(RandomStreamId::new(7));
        let mut stream_c = world_random.stream(RandomStreamId::new(8));

        assert_eq!(stream_a.next_u64(), stream_b.next_u64());
        assert_ne!(stream_a.next_u64(), stream_c.next_u64());
    }

    #[test]
    fn stream_creation_order_does_not_matter() {
        let world_random = WorldRandom::new(42);

        let mut first_a = world_random.stream(RandomStreamId::new(1));
        let mut first_b = world_random.stream(RandomStreamId::new(2));
        let mut second_b = world_random.stream(RandomStreamId::new(2));
        let mut second_a = world_random.stream(RandomStreamId::new(1));

        assert_eq!(first_a.next_u64(), second_a.next_u64());
        assert_eq!(first_b.next_u64(), second_b.next_u64());
    }

    #[test]
    fn stream_derivation_golden_vectors() {
        assert_eq!(WORLD_RANDOM_STREAM_VERSION, 1);

        let cases = [
            (0, 0, 0xC500_8C00_75DB_2FCF, 0xEA7D_F1C6_C5D1_0FD0),
            (0, 1, 0xEE03_B46A_D6BF_1A54, 0x572D_B9D1_CFF9_27C4),
            (42, 7, 0x290B_1D6B_23F1_BE8B, 0x79B9_857A_8F9D_756F),
            (42, 8, 0x001F_920A_430C_B471, 0xE4F4_5B38_5B8C_C1BE),
            (
                u64::MAX,
                u64::MAX,
                0xCCE8_760A_5759_53A7,
                0x771D_ED48_30AA_FED3,
            ),
        ];

        for (root_seed, stream_id, expected_seed, expected_first) in cases {
            let world_random = WorldRandom::new(root_seed);
            assert_eq!(
                world_random.stream_seed(RandomStreamId::new(stream_id)),
                expected_seed
            );

            let mut stream = world_random.stream(RandomStreamId::new(stream_id));
            assert_eq!(stream.next_u64(), expected_first);
        }
    }

    #[test]
    fn stream_creation_does_not_consume_main_stream() {
        let mut with_stream = WorldRandom::new(42);
        let mut without_stream = WorldRandom::new(42);

        let _ = with_stream.stream(RandomStreamId::new(7));

        assert_eq!(with_stream.next_u64(), without_stream.next_u64());
    }

    #[test]
    fn reset_resets_main_stream() {
        let mut world_random = WorldRandom::new(1);
        let first = world_random.next_u64();
        world_random.next_u64();
        world_random.reset(1);

        assert_eq!(world_random.next_u64(), first);
    }

    #[test]
    fn snapshot_restores_main_stream_position() {
        let mut world_random = WorldRandom::new(11);
        world_random.next_u64();
        world_random.next_u64();
        let snapshot = world_random.snapshot();
        let expected = world_random.next_u64();

        let mut restored = WorldRandom::from_snapshot(snapshot);

        assert_eq!(restored.seed(), 11);
        assert_eq!(restored.next_u64(), expected);
    }

    #[test]
    fn restore_replaces_seed_and_main_stream_position() {
        let mut original = WorldRandom::new(3);
        original.next_u64();
        let snapshot = original.snapshot();
        let expected = original.next_u64();

        let mut restored = WorldRandom::new(99);
        restored.next_u64();
        restored.restore(snapshot);

        assert_eq!(restored.seed(), 3);
        assert_eq!(restored.next_u64(), expected);
    }
}
