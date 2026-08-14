#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() > rne_urdf_import::MAX_IMPORT_BYTES {
        return;
    }
    if let Ok(text) = std::str::from_utf8(data) {
        let _ = rne_urdf_import::parse_urdf(text);
    }
});
