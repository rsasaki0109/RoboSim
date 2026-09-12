//! Print the frozen PMDC final protocol without reading final data.

use anyhow::Result;
use rne_mobility_benchmark::recorded_pmdc::final_protocol::pmdc_final_evaluation_protocol;

fn main() -> Result<()> {
    let protocol = pmdc_final_evaluation_protocol();
    println!(
        "{{\n  \"protocol_sha256\": \"{}\",\n  \"protocol\": {}\n}}",
        protocol.sha256()?,
        serde_json::to_string_pretty(&protocol)?
    );
    Ok(())
}
