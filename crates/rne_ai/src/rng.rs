//! Deterministic pseudo-random numbers for reproducible domain randomization.

/// SplitMix64-based RNG for reproducible domain randomization.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    /// Creates an RNG seeded from the given value.
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Returns the next 64-bit value and advances the stream.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Samples a floating-point value in `[min, max)`.
    pub fn uniform_f64(&mut self, min: f64, max: f64) -> f64 {
        let unit = (self.next_u64() as f64) / (u64::MAX as f64);
        min + unit * (max - min)
    }

    /// Samples an index in `[0, len)`.
    pub fn uniform_usize(&mut self, len: usize) -> usize {
        assert!(len > 0, "uniform_usize requires len > 0");
        (self.next_u64() as usize) % len
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_is_repeatable() {
        let mut first = DeterministicRng::new(42);
        let mut second = DeterministicRng::new(42);
        for _ in 0..8 {
            assert_eq!(first.next_u64(), second.next_u64());
        }
    }
}
