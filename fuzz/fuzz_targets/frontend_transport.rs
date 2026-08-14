#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = rne_data::transport::TransportFrame::decode(data, 64 * 1024);
});
