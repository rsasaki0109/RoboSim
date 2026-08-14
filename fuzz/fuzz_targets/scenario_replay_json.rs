#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() > rne_openscenario::MAX_REPLAY_JSON_BYTES {
        return;
    }
    if let Ok(text) = std::str::from_utf8(data) {
        let _ = rne_openscenario::ScenarioReplayArtifact::from_json(text);
    }
});
