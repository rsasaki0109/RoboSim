#[test]
fn legacy_rng_path_is_core_rng_type() {
    fn accepts_core_rng(_: rne_core::DeterministicRng) {}

    accepts_core_rng(rne_ai::rng::DeterministicRng::new(1));

    let _: rne_ai::rng::DeterministicRng = rne_core::DeterministicRng::new(2);
    let _: rne_ai::DeterministicRng = rne_ai::rng::DeterministicRng::new(3);
}

#[test]
fn legacy_rng_path_preserves_sequence_and_methods() {
    let mut rng = rne_ai::rng::DeterministicRng::new(0);
    assert_eq!(rng.next_u64(), 0xE220_A839_7B1D_CDAF);
    assert_eq!(rng.next_u64(), 0x6E78_9E6A_A1B9_65F4);
    assert_eq!(rng.next_u64(), 0x06C4_5D18_8009_454F);

    let mut advanced = rne_ai::rng::DeterministicRng::new(42);
    advanced.next_u64();
    advanced.next_u64();
    let state = advanced.state();
    let expected = advanced.next_u64();

    let mut restored = rne_ai::rng::DeterministicRng::from_state(state);
    assert_eq!(restored.next_u64(), expected);
    assert!(restored.uniform_usize(17) < 17);
    assert!((0.0..1.0).contains(&restored.uniform_f64(0.0, 1.0)));
}
