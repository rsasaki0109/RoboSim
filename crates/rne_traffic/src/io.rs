//! Canonical traffic asset parsing and file I/O.

use crate::{TrafficAsset, TrafficAssetError};
use std::fs;
use std::path::Path;

/// Parses, canonicalizes, and validates a `.rne.traffic.json` document.
pub fn parse_traffic_asset(bytes: &[u8]) -> Result<TrafficAsset, TrafficAssetError> {
    let parsed: TrafficAsset = serde_json::from_slice(bytes)?;
    let canonical = parsed.canonicalized();
    canonical.validate()?;
    Ok(canonical)
}

/// Loads, canonicalizes, and validates a `.rne.traffic.json` file.
pub fn load_traffic_asset(path: &Path) -> Result<TrafficAsset, TrafficAssetError> {
    let bytes = fs::read(path).map_err(|error| TrafficAssetError::io(path, error))?;
    parse_traffic_asset(&bytes)
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
