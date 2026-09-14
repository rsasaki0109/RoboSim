//! Deterministic seeded generator shared by sampling planners and adapters.
//!
//! No global random state is used, so a scene and request reproduce exactly.

/// Small deterministic xorshift generator with an explicit caller seed.
#[derive(Clone, Debug)]
pub(crate) struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    pub(crate) fn new(seed: u64) -> Self {
        let state = seed ^ 0x9E37_79B9_7F4A_7C15;
        Self {
            state: if state == 0 {
                0x1234_5678_9ABC_DEF0
            } else {
                state
            },
        }
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.state = value;
        value
    }

    pub(crate) fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}
