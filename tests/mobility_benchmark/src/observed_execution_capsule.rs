//! Exact-byte artifact intake for execution-failure evidence.
//! This layer does not execute history or promote producer replay claims.

use anyhow::{ensure, Context, Result};
use rne_log::ArtifactRef;
use sha2::{Digest, Sha256};
use std::{io::Read, path::Path};

/// Upper bound for any one referenced execution artifact (32 MiB).
pub const MAX_EXECUTION_ARTIFACT_BYTES: usize = 32 * 1024 * 1024;

/// The initial contract/attempt format uses the writer's compact Value encoding.
/// Comparing exact serialization rejects duplicate keys and alternate numeric
/// representations before any backend factory is invoked.
pub(crate) fn decode_canonical_execution_json(bytes: &[u8]) -> Result<serde_json::Value> {
    ensure!(
        bytes.len() <= 1024 * 1024,
        "execution JSON exceeds byte limit"
    );
    let value: serde_json::Value = serde_json::from_slice(bytes)?;
    ensure!(
        serde_json::to_vec(&value)? == bytes,
        "noncanonical execution JSON"
    );
    Ok(value)
}

/// Reads a declared artifact with caller-selected schema identity and size bound.
///
/// The caller must supply trusted expected identifiers, then decode/validate the
/// returned payload's own schema. Hash equality proves byte integrity only, not
/// physical execution, authentication, or a valid replay claim. Resolved paths must
/// remain inside the supplied directory. Concurrent hostile filesystem mutation
/// is outside this local evidence-reader contract; use a stable evidence directory.
pub fn read_execution_artifact(
    directory: &Path,
    reference: &ArtifactRef,
    expected_role: &str,
    expected_kind: &str,
    expected_schema_version: u32,
    maximum_bytes: usize,
) -> Result<Vec<u8>> {
    reference.validate()?;
    ensure!(
        maximum_bytes > 0 && maximum_bytes <= MAX_EXECUTION_ARTIFACT_BYTES,
        "invalid artifact read bound"
    );
    ensure!(
        reference.role == expected_role
            && reference.kind == expected_kind
            && reference.schema_version == expected_schema_version,
        "execution artifact identity mismatch"
    );
    let root = directory
        .canonicalize()
        .context("resolve execution evidence directory")?;
    ensure!(root.is_dir(), "evidence root is not a directory");
    let path = root
        .join(&reference.path)
        .canonicalize()
        .context("resolve execution artifact")?;
    ensure!(
        path.starts_with(&root) && path != root,
        "artifact escapes evidence directory"
    );
    let file = std::fs::File::open(path).context("open execution artifact")?;
    ensure!(file.metadata()?.is_file(), "artifact is not a regular file");
    let mut bytes = Vec::new();
    file.take(maximum_bytes as u64 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= maximum_bytes,
        "execution artifact exceeds byte limit"
    );
    ensure!(
        format!("{:x}", Sha256::digest(&bytes)) == reference.sha256,
        "execution artifact SHA-256 mismatch"
    );
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn execution_json_rejects_duplicate_keys_and_noncanonical_bytes() {
        assert!(decode_canonical_execution_json(br#"{"a":1,"a":1}"#).is_err());
        assert!(decode_canonical_execution_json(b"{\"a\":1}\n").is_err());
        assert_eq!(
            decode_canonical_execution_json(br#"{"a":1}"#).unwrap(),
            serde_json::json!({"a":1})
        );
    }
    #[test]
    fn execution_artifact_intake_binds_exact_bytes_and_trusted_identity() {
        let directory = tempfile::tempdir().unwrap();
        let bytes = b"{\"schema_version\":1}\n";
        let path = directory.path().join("attempt.json");
        std::fs::write(&path, bytes).unwrap();
        let reference = ArtifactRef::new(
            "execution_attempt",
            "test_attempt",
            1,
            "attempt.json",
            format!("{:x}", Sha256::digest(bytes)),
        )
        .unwrap();
        assert_eq!(
            read_execution_artifact(
                directory.path(),
                &reference,
                "execution_attempt",
                "test_attempt",
                1,
                128
            )
            .unwrap(),
            bytes
        );
        assert!(read_execution_artifact(
            directory.path(),
            &reference,
            "execution_attempt",
            "wrong_kind",
            1,
            128
        )
        .is_err());
        assert!(read_execution_artifact(
            directory.path(),
            &reference,
            "execution_attempt",
            "test_attempt",
            2,
            128
        )
        .is_err());
        assert!(read_execution_artifact(
            directory.path(),
            &reference,
            "execution_attempt",
            "test_attempt",
            1,
            4
        )
        .is_err());
        std::fs::write(&path, b"{\"schema_version\":1}").unwrap();
        assert!(read_execution_artifact(
            directory.path(),
            &reference,
            "execution_attempt",
            "test_attempt",
            1,
            128
        )
        .is_err());
        let mut traversal = reference;
        traversal.path = "../attempt.json".into();
        assert!(read_execution_artifact(
            directory.path(),
            &traversal,
            "execution_attempt",
            "test_attempt",
            1,
            128
        )
        .is_err());
    }
}
