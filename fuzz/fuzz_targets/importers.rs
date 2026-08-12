#![no_main]

use libfuzzer_sys::fuzz_target;
use rne_fuzz_smoke::{exercise_boundary, FuzzBoundary};

fuzz_target!(|data: &[u8]| {
    let Some((&selector, payload)) = data.split_first() else {
        return;
    };
    let boundary = FuzzBoundary::IMPORTERS[usize::from(selector) % FuzzBoundary::IMPORTERS.len()];
    let _ = exercise_boundary(boundary, payload);
});
