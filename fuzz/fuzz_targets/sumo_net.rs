#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() <= rne_sumo::MAX_IMPORT_BYTES {
        let _ = rne_sumo::parse_sumo_net_xml(data);
    }
});
