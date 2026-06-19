//! Deterministic pseudo-random numbers for simulation logic.

const GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;
const F64_UNIT: f64 = 1.0 / ((1_u64 << 53) as f64);
const KEYED_RANDOM_DOMAIN_V1: u64 = 0x3164_7965_6B45_4E52;

/// Deterministic RNG algorithm version used in replay metadata.
pub const DETERMINISTIC_RNG_VERSION: u32 = 1;

/// Stateless keyed random algorithm version used in replay metadata.
pub const KEYED_RANDOM_VERSION: u32 = 1;

/// SplitMix64-based RNG for reproducible simulation behavior.
///
/// This generator is deterministic across supported platforms when the same
/// methods are called in the same order. It is not cryptographically secure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    /// Creates an RNG seeded from the given value.
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Restores an RNG from a previously captured internal state.
    ///
    /// This is intended for deterministic snapshots and replay checkpoints.
    pub const fn from_state(state: u64) -> Self {
        Self { state }
    }

    /// Returns the current internal state.
    pub const fn state(&self) -> u64 {
        self.state
    }

    /// Returns the next 64-bit value and advances the stream.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(GAMMA);
        mix64(self.state)
    }

    /// Samples a floating-point value in `[min, max)`.
    ///
    /// # Panics
    ///
    /// Panics when `min` or `max` is not finite, when `min >= max`, or when
    /// `max - min` is not finite.
    pub fn uniform_f64(&mut self, min: f64, max: f64) -> f64 {
        assert!(min.is_finite(), "min must be finite");
        assert!(max.is_finite(), "max must be finite");
        assert!(min < max, "min must be less than max");

        let width = max - min;
        assert!(width.is_finite(), "range width must be finite");

        let unit = ((self.next_u64() >> 11) as f64) * F64_UNIT;
        let value = width.mul_add(unit, min);
        if value < max {
            value
        } else {
            next_down_f64(max)
        }
    }

    /// Samples an index in `[0, len)`.
    ///
    /// # Panics
    ///
    /// Panics when `len` is zero.
    pub fn uniform_usize(&mut self, len: usize) -> usize {
        assert!(len != 0, "len must be non-zero");

        let bound = u64::try_from(len).expect("usize wider than u64 is unsupported");
        let threshold = bound.wrapping_neg() % bound;

        loop {
            let value = self.next_u64();
            if value >= threshold {
                return (value % bound) as usize;
            }
        }
    }
}

/// Stateless keyed random source for order-independent samples.
///
/// This generator maps `(root_seed, domain, stable_id, sample_index, channel)`
/// to deterministic values without storing or advancing stream state. It is
/// useful for sensor noise and other samples that must not shift when unrelated
/// entities or systems are added. It is not cryptographically secure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct KeyedRandom {
    root_seed: u64,
    domain: u64,
}

impl KeyedRandom {
    /// Creates a keyed random source from a root seed and domain tag.
    pub const fn new(root_seed: u64, domain: u64) -> Self {
        Self { root_seed, domain }
    }

    /// Returns the root seed.
    pub const fn root_seed(&self) -> u64 {
        self.root_seed
    }

    /// Returns the domain tag.
    pub const fn domain(&self) -> u64 {
        self.domain
    }

    /// Returns a deterministic 64-bit value for a stable sample coordinate.
    pub fn sample_u64(&self, stable_id: u64, sample_index: u64, channel: u64) -> u64 {
        let mut value = mix64(self.root_seed ^ KEYED_RANDOM_DOMAIN_V1);
        value = mix64(value ^ self.domain);
        value = mix64(value ^ stable_id);
        value = mix64(value ^ sample_index);
        mix64(value ^ channel)
    }

    /// Samples a floating-point value in `[0, 1)`.
    pub fn sample_unit_f64(&self, stable_id: u64, sample_index: u64, channel: u64) -> f64 {
        ((self.sample_u64(stable_id, sample_index, channel) >> 11) as f64) * F64_UNIT
    }

    /// Samples a floating-point value in `[-1, 1)`.
    pub fn sample_signed_f64(&self, stable_id: u64, sample_index: u64, channel: u64) -> f64 {
        self.sample_unit_f64(stable_id, sample_index, channel) * 2.0 - 1.0
    }

    /// Samples a floating-point value in `[min, max)`.
    ///
    /// # Panics
    ///
    /// Panics when `min` or `max` is not finite, when `min >= max`, or when
    /// `max - min` is not finite.
    pub fn sample_f64(
        &self,
        stable_id: u64,
        sample_index: u64,
        channel: u64,
        min: f64,
        max: f64,
    ) -> f64 {
        assert!(min.is_finite(), "min must be finite");
        assert!(max.is_finite(), "max must be finite");
        assert!(min < max, "min must be less than max");

        let width = max - min;
        assert!(width.is_finite(), "range width must be finite");

        let value = width.mul_add(self.sample_unit_f64(stable_id, sample_index, channel), min);
        if value < max {
            value
        } else {
            next_down_f64(max)
        }
    }
}

/// Applies the SplitMix64 finalizer to a 64-bit value.
pub fn mix64(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^ (value >> 31)
}

fn next_down_f64(value: f64) -> f64 {
    debug_assert!(value.is_finite());
    if value == 0.0 {
        return -f64::from_bits(1);
    }

    let bits = value.to_bits();
    if value > 0.0 {
        f64::from_bits(bits - 1)
    } else {
        f64::from_bits(bits + 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seed_zero_golden_vector() {
        let mut rng = DeterministicRng::new(0);
        assert_eq!(rng.next_u64(), 0xE220_A839_7B1D_CDAF);
        assert_eq!(rng.next_u64(), 0x6E78_9E6A_A1B9_65F4);
        assert_eq!(rng.next_u64(), 0x06C4_5D18_8009_454F);
    }

    #[test]
    fn rng_is_repeatable() {
        let mut first = DeterministicRng::new(42);
        let mut second = DeterministicRng::new(42);
        for _ in 0..8 {
            assert_eq!(first.next_u64(), second.next_u64());
        }
    }

    #[test]
    fn state_restores_stream_position() {
        let mut rng = DeterministicRng::new(42);
        rng.next_u64();
        rng.next_u64();
        let state = rng.state();
        let expected = rng.next_u64();

        let mut restored = DeterministicRng::from_state(state);

        assert_eq!(restored.next_u64(), expected);
    }

    #[test]
    fn uniform_f64_stays_in_half_open_range() {
        let mut rng = DeterministicRng::new(7);
        for _ in 0..1024 {
            let sample = rng.uniform_f64(-2.0, 3.0);
            assert!((-2.0..3.0).contains(&sample));
        }
    }

    #[test]
    fn uniform_usize_stays_in_range() {
        let mut rng = DeterministicRng::new(9);
        for _ in 0..1024 {
            assert!(rng.uniform_usize(11) < 11);
        }
    }

    #[test]
    #[should_panic(expected = "len must be non-zero")]
    fn uniform_usize_rejects_zero_len() {
        let mut rng = DeterministicRng::new(9);
        rng.uniform_usize(0);
    }

    #[test]
    #[should_panic(expected = "min must be less than max")]
    fn uniform_f64_rejects_empty_range() {
        let mut rng = DeterministicRng::new(9);
        rng.uniform_f64(1.0, 1.0);
    }

    #[test]
    fn keyed_random_is_order_independent() {
        let random = KeyedRandom::new(42, 7);

        let first = random.sample_u64(11, 3, 0);
        let unrelated = random.sample_u64(99, 200, 4);
        let second = random.sample_u64(11, 3, 0);

        assert_eq!(first, second);
        assert_ne!(first, unrelated);
    }

    #[test]
    fn keyed_random_separates_channels() {
        let random = KeyedRandom::new(42, 7);

        assert_ne!(random.sample_u64(11, 3, 0), random.sample_u64(11, 3, 1));
    }

    #[test]
    fn keyed_random_unit_values_stay_half_open() {
        let random = KeyedRandom::new(42, 7);

        for channel in 0..1024 {
            let sample = random.sample_unit_f64(11, 3, channel);
            assert!((0.0..1.0).contains(&sample));
        }
    }
}
