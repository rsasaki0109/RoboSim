//! Deterministic pseudo-random numbers for reproducible domain randomization.

#[doc(inline)]
pub use rne_core::DeterministicRng;

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
