//! Canonical traffic asset parsing and file I/O.

use crate::{TrafficAsset, TrafficAssetError};
use std::fs;
use std::io::Read;
use std::path::Path;

/// Maximum accepted native traffic JSON input size.
pub const TRAFFIC_ASSET_MAX_INPUT_BYTES: usize = 128 * 1024 * 1024;

/// Parses, canonicalizes, and validates a `.rne.traffic.json` document.
pub fn parse_traffic_asset(bytes: &[u8]) -> Result<TrafficAsset, TrafficAssetError> {
    ensure_input_len(bytes.len(), Path::new("<memory>"))?;
    let parsed: TrafficAsset = serde_json::from_slice(bytes)?;
    let canonical = parsed.canonicalized();
    canonical.validate()?;
    Ok(canonical)
}

/// Loads, canonicalizes, and validates a `.rne.traffic.json` file.
pub fn load_traffic_asset(path: &Path) -> Result<TrafficAsset, TrafficAssetError> {
    let mut bytes = Vec::new();
    fs::File::open(path)
        .and_then(|file| {
            file.take((TRAFFIC_ASSET_MAX_INPUT_BYTES as u64) + 1)
                .read_to_end(&mut bytes)
        })
        .map_err(|error| TrafficAssetError::io(path, error))?;
    ensure_input_len(bytes.len(), path)?;
    parse_traffic_asset(&bytes)
}

fn ensure_input_len(actual: usize, path: &Path) -> Result<(), TrafficAssetError> {
    if actual > TRAFFIC_ASSET_MAX_INPUT_BYTES {
        return Err(TrafficAssetError::Io {
            path: path.display().to_string(),
            message: format!(
                "input is {actual} bytes, limit is {TRAFFIC_ASSET_MAX_INPUT_BYTES} bytes"
            ),
        });
    }
    Ok(())
}

/// Serializes a validated asset as canonical pretty JSON with one trailing newline.
pub fn canonical_traffic_asset_bytes(asset: &TrafficAsset) -> Result<Vec<u8>, TrafficAssetError> {
    let canonical = asset.canonicalized();
    canonical.validate()?;
    let mut bytes = serde_json::to_vec_pretty(&canonical)?;
    bytes.push(b'\n');
    Ok(bytes)
}

/// Writes a validated asset as canonical `.rne.traffic.json`.
pub fn save_traffic_asset(path: &Path, asset: &TrafficAsset) -> Result<(), TrafficAssetError> {
    let bytes = canonical_traffic_asset_bytes(asset)?;
    fs::write(path, bytes).map_err(|error| TrafficAssetError::io(path, error))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_declared_input_size_before_json_allocation() {
        assert!(
            ensure_input_len(TRAFFIC_ASSET_MAX_INPUT_BYTES + 1, Path::new("fixture.json")).is_err()
        );
    }
}
