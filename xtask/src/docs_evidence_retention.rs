//! Enforces the `docs/evidence` large-file retention policy (ADR 034).
//!
//! New raw evidence traces above [`MAX_INLINE_EVIDENCE_BYTES`] must be stored
//! outside Git (for example, as a GitHub Release asset) and referenced from a
//! small `*.pointer.json` sidecar instead of being committed inline. Evidence
//! already tracked before the policy took effect is recorded, by content hash,
//! in a historical allowlist registry so it keeps passing untouched.

use anyhow::{Context, Result};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};

const EVIDENCE_ROOT: &str = "docs/evidence";
const DEFAULT_REGISTRY: &str = "release/docs-evidence-retention.toml";
const REGISTRY_SCHEMA_VERSION: u32 = 1;

/// Files under `docs/evidence` at or below this size may be committed inline.
///
/// Larger files must either already be listed in the historical allowlist
/// registry (with a matching digest) or be replaced by a `*.pointer.json`
/// sidecar that names an external, content-addressed retrieval location.
pub(crate) const MAX_INLINE_EVIDENCE_BYTES: u64 = 1024 * 1024;

const POINTER_SUFFIX: &str = ".pointer.json";
const POINTER_KIND: &str = "rne_docs_evidence_pointer";
const POINTER_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Registry {
    schema_version: u32,
    #[serde(default)]
    entry: Vec<RegistryEntry>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct RegistryEntry {
    path: String,
    sha256: String,
    size_bytes: u64,
}

/// The pointer sidecar format a contributor commits in place of a large raw
/// evidence file (see `docs/adr/034-*.md`).
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct PointerFile {
    kind: String,
    schema_version: u32,
    original_filename: String,
    sha256: String,
    size_bytes: u64,
    url: String,
}

/// Validates the committed `docs/evidence` retention registry against the
/// current tree. Wired into `xtask lint-boundaries` (and therefore `ci-lint`
/// / `ci`).
pub(crate) fn validate_committed(root: &Path) -> Result<()> {
    validate_registry(root, &root.join(DEFAULT_REGISTRY))
}

/// Standalone `docs-evidence-check` command.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = crate::workspace_root()?;
    let mut registry = root.join(DEFAULT_REGISTRY);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--registry" => {
                let value = args.next().context("--registry requires a path")?;
                let value = PathBuf::from(value);
                registry = if value.is_absolute() {
                    value
                } else {
                    root.join(value)
                };
            }
            "--help" | "-h" => {
                println!("docs-evidence-check [--registry PATH]");
                return Ok(());
            }
            other => anyhow::bail!("unknown docs-evidence-check argument: {other}"),
        }
    }
    validate_registry(&root, &registry)?;
    println!(
        "docs/evidence retention policy ok: registry={}",
        registry.display()
    );
    Ok(())
}

fn validate_registry(root: &Path, registry_path: &Path) -> Result<()> {
    let evidence_root = root.join(EVIDENCE_ROOT);
    if !evidence_root.is_dir() {
        // docs/evidence is optional (for example, in a shallow export); there
        // is nothing to enforce.
        return Ok(());
    }

    let text = fs::read_to_string(registry_path).with_context(|| {
        format!(
            "read docs/evidence retention registry {}",
            registry_path.display()
        )
    })?;
    let registry: Registry = toml::from_str(&text).with_context(|| {
        format!(
            "parse docs/evidence retention registry {}",
            registry_path.display()
        )
    })?;
    anyhow::ensure!(
        registry.schema_version == REGISTRY_SCHEMA_VERSION,
        "docs/evidence retention registry schema must be {REGISTRY_SCHEMA_VERSION}"
    );

    let mut allowlist: BTreeMap<String, (String, u64)> = BTreeMap::new();
    for entry in &registry.entry {
        validate_registry_path(&entry.path)?;
        anyhow::ensure!(
            entry.path.starts_with("docs/evidence/"),
            "docs/evidence retention registry entry escapes docs/evidence: {}",
            entry.path
        );
        anyhow::ensure!(
            is_canonical_sha256(&entry.sha256),
            "docs/evidence retention registry entry has a malformed sha256: {}",
            entry.path
        );
        anyhow::ensure!(
            entry.size_bytes > MAX_INLINE_EVIDENCE_BYTES,
            "docs/evidence retention registry entry {} is at or below the {MAX_INLINE_EVIDENCE_BYTES}-byte \
             threshold and does not need a historical allowlist entry",
            entry.path
        );
        anyhow::ensure!(
            allowlist
                .insert(entry.path.clone(), (entry.sha256.clone(), entry.size_bytes))
                .is_none(),
            "docs/evidence retention registry has a duplicate entry: {}",
            entry.path
        );
    }

    let mut files = Vec::new();
    walk_files(&evidence_root, &mut files)?;
    files.sort();

    let mut seen_allowlisted: BTreeSet<String> = BTreeSet::new();
    for file in &files {
        let metadata = fs::symlink_metadata(file)
            .with_context(|| format!("inspect docs/evidence file {}", file.display()))?;
        anyhow::ensure!(
            metadata.is_file() && !metadata.file_type().is_symlink(),
            "docs/evidence entry must be a regular non-symlink file: {}",
            file.display()
        );
        let relative = file.strip_prefix(root).with_context(|| {
            format!(
                "docs/evidence file escaped the repository root: {}",
                file.display()
            )
        })?;
        let relative = relative.to_string_lossy().replace('\\', "/");
        let size = metadata.len();

        if relative.ends_with(POINTER_SUFFIX) {
            let bytes =
                fs::read(file).with_context(|| format!("read pointer file {}", file.display()))?;
            let pointer: PointerFile = serde_json::from_slice(&bytes)
                .with_context(|| format!("parse pointer file {}", file.display()))?;
            validate_pointer(&pointer, &relative)?;
            continue;
        }

        if size <= MAX_INLINE_EVIDENCE_BYTES {
            continue;
        }

        let Some((expected_digest, expected_size)) = allowlist.get(&relative) else {
            anyhow::bail!(
                "docs/evidence file {relative} exceeds {MAX_INLINE_EVIDENCE_BYTES} bytes and is not a \
                 historical allowlist entry; store it externally (for example, a GitHub Release asset) \
                 and commit a `{POINTER_SUFFIX}` pointer file instead (see ADR 034)"
            );
        };
        let bytes = fs::read(file)
            .with_context(|| format!("read docs/evidence file {}", file.display()))?;
        let digest = format!("sha256:{:x}", Sha256::digest(&bytes));
        anyhow::ensure!(
            *expected_size == size && *expected_digest == digest,
            "docs/evidence historical file {relative} no longer matches its allowlisted digest; \
             historical evidence must not be modified in place (see ADR 034)"
        );
        seen_allowlisted.insert(relative);
    }

    let allowlist_paths: BTreeSet<String> = allowlist.keys().cloned().collect();
    anyhow::ensure!(
        seen_allowlisted == allowlist_paths,
        "docs/evidence retention registry no longer matches the tree; missing from tree: {:?}",
        allowlist_paths
            .difference(&seen_allowlisted)
            .collect::<Vec<_>>()
    );

    Ok(())
}

fn validate_pointer(pointer: &PointerFile, relative: &str) -> Result<()> {
    anyhow::ensure!(
        pointer.kind == POINTER_KIND,
        "pointer file {relative} has the wrong kind"
    );
    anyhow::ensure!(
        pointer.schema_version == POINTER_SCHEMA_VERSION,
        "pointer file {relative} has an unexpected schema version"
    );
    anyhow::ensure!(
        !pointer.original_filename.trim().is_empty()
            && !pointer.original_filename.contains('/')
            && !pointer.original_filename.contains('\\'),
        "pointer file {relative} has an invalid original_filename"
    );
    anyhow::ensure!(
        is_canonical_sha256(&pointer.sha256),
        "pointer file {relative} has a malformed sha256"
    );
    anyhow::ensure!(
        pointer.size_bytes > MAX_INLINE_EVIDENCE_BYTES,
        "pointer file {relative} points at a file that would fit inline; commit it directly instead"
    );
    anyhow::ensure!(
        pointer.url.starts_with("https://") && pointer.url.len() > "https://".len(),
        "pointer file {relative} must have an https retrieval URL"
    );
    Ok(())
}

fn walk_files(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    for entry in fs::read_dir(dir).with_context(|| format!("read directory {}", dir.display()))? {
        let entry = entry.with_context(|| format!("read directory entry in {}", dir.display()))?;
        let path = entry.path();
        let file_type = entry
            .file_type()
            .with_context(|| format!("inspect directory entry {}", path.display()))?;
        if file_type.is_dir() {
            walk_files(&path, out)?;
        } else if file_type.is_file() {
            out.push(path);
        } else {
            anyhow::bail!(
                "docs/evidence contains a non-regular entry: {}",
                path.display()
            );
        }
    }
    Ok(())
}

fn validate_registry_path(path: &str) -> Result<()> {
    anyhow::ensure!(
        !path.is_empty() && !path.contains('\\') && !path.chars().any(char::is_control),
        "docs/evidence retention registry path must be a clean forward-slash path: {path}"
    );
    let parsed = Path::new(path);
    anyhow::ensure!(
        !parsed.is_absolute()
            && parsed
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "docs/evidence retention registry path escaped the repository: {path}"
    );
    Ok(())
}

fn is_canonical_sha256(value: &str) -> bool {
    let Some(digest) = value.strip_prefix("sha256:") else {
        return false;
    };
    digest.len() == 64
        && digest
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn write(path: &Path, bytes: &[u8]) {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, bytes).unwrap();
    }

    fn write_registry(root: &Path, body: &str) {
        write(&root.join(DEFAULT_REGISTRY), body.as_bytes());
    }

    #[test]
    fn committed_registry_matches_the_real_tree() {
        validate_committed(&crate::workspace_root().unwrap()).unwrap();
    }

    #[test]
    fn small_files_never_need_an_allowlist_entry() {
        let root = tempdir().unwrap();
        write(
            &root.path().join("docs/evidence/small-trace.json"),
            &vec![0u8; 1024],
        );
        write_registry(root.path(), "schema_version = 1\n");
        validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap();
    }

    #[test]
    fn missing_docs_evidence_directory_is_not_an_error() {
        let root = tempdir().unwrap();
        // No docs/evidence directory and no registry file at all.
        validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap();
    }

    #[test]
    fn new_large_file_without_an_allowlist_entry_or_pointer_fails() {
        let root = tempdir().unwrap();
        write(
            &root
                .path()
                .join("docs/evidence/new-lab/evidence/huge-trace.json"),
            &vec![7u8; (MAX_INLINE_EVIDENCE_BYTES + 1) as usize],
        );
        write_registry(root.path(), "schema_version = 1\n");
        let error =
            validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap_err();
        assert!(error.to_string().contains("historical allowlist entry"));
    }

    #[test]
    fn allowlisted_large_file_with_matching_digest_passes() {
        let root = tempdir().unwrap();
        let bytes = vec![9u8; (MAX_INLINE_EVIDENCE_BYTES + 42) as usize];
        write(
            &root.path().join("docs/evidence/old-lab/trace.json"),
            &bytes,
        );
        let digest = format!("sha256:{:x}", Sha256::digest(&bytes));
        write_registry(
            root.path(),
            &format!(
                "schema_version = 1\n\n[[entry]]\npath = \"docs/evidence/old-lab/trace.json\"\nsha256 = \"{digest}\"\nsize_bytes = {}\n",
                bytes.len()
            ),
        );
        validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap();
    }

    #[test]
    fn allowlisted_file_whose_bytes_changed_fails() {
        let root = tempdir().unwrap();
        let bytes = vec![9u8; (MAX_INLINE_EVIDENCE_BYTES + 42) as usize];
        write(
            &root.path().join("docs/evidence/old-lab/trace.json"),
            &bytes,
        );
        // Registry digest does not match the file contents above.
        write_registry(
            root.path(),
            &format!(
                "schema_version = 1\n\n[[entry]]\npath = \"docs/evidence/old-lab/trace.json\"\nsha256 = \"sha256:{}\"\nsize_bytes = {}\n",
                "a".repeat(64),
                bytes.len()
            ),
        );
        let error =
            validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap_err();
        assert!(error.to_string().contains("no longer matches"));
    }

    #[test]
    fn registry_entry_for_a_file_no_longer_on_disk_fails() {
        let root = tempdir().unwrap();
        write(&root.path().join("docs/evidence/.keep.json"), b"{}");
        write_registry(
            root.path(),
            &format!(
                "schema_version = 1\n\n[[entry]]\npath = \"docs/evidence/gone/trace.json\"\nsha256 = \"sha256:{}\"\nsize_bytes = {}\n",
                "b".repeat(64),
                MAX_INLINE_EVIDENCE_BYTES + 1
            ),
        );
        let error =
            validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap_err();
        assert!(error.to_string().contains("no longer matches the tree"));
    }

    #[test]
    fn valid_pointer_file_is_accepted_regardless_of_registry() {
        let root = tempdir().unwrap();
        let pointer = format!(
            "{{\"kind\":\"{POINTER_KIND}\",\"schema_version\":{POINTER_SCHEMA_VERSION},\
             \"original_filename\":\"trace.json\",\"sha256\":\"sha256:{}\",\
             \"size_bytes\":{},\"url\":\"https://example.invalid/trace.json\"}}",
            "c".repeat(64),
            MAX_INLINE_EVIDENCE_BYTES + 1
        );
        write(
            &root
                .path()
                .join("docs/evidence/new-lab/trace.json.pointer.json"),
            pointer.as_bytes(),
        );
        write_registry(root.path(), "schema_version = 1\n");
        validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap();
    }

    #[test]
    fn pointer_file_with_wrong_kind_fails() {
        let root = tempdir().unwrap();
        let pointer = format!(
            "{{\"kind\":\"not_a_pointer\",\"schema_version\":{POINTER_SCHEMA_VERSION},\
             \"original_filename\":\"trace.json\",\"sha256\":\"sha256:{}\",\
             \"size_bytes\":{},\"url\":\"https://example.invalid/trace.json\"}}",
            "d".repeat(64),
            MAX_INLINE_EVIDENCE_BYTES + 1
        );
        write(
            &root
                .path()
                .join("docs/evidence/new-lab/trace.json.pointer.json"),
            pointer.as_bytes(),
        );
        write_registry(root.path(), "schema_version = 1\n");
        let error =
            validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap_err();
        assert!(error.to_string().contains("wrong kind"));
    }

    #[test]
    fn duplicate_registry_entries_fail_closed() {
        let root = tempdir().unwrap();
        let digest = format!("sha256:{}", "e".repeat(64));
        write_registry(
            root.path(),
            &format!(
                "schema_version = 1\n\n[[entry]]\npath = \"docs/evidence/a.json\"\nsha256 = \"{digest}\"\nsize_bytes = {size}\n\n[[entry]]\npath = \"docs/evidence/a.json\"\nsha256 = \"{digest}\"\nsize_bytes = {size}\n",
                size = MAX_INLINE_EVIDENCE_BYTES + 1
            ),
        );
        write(&root.path().join("docs/evidence/.keep.json"), b"{}");
        let error =
            validate_registry(root.path(), &root.path().join(DEFAULT_REGISTRY)).unwrap_err();
        assert!(error.to_string().contains("duplicate entry"));
    }

    #[test]
    fn registry_paths_and_digests_fail_closed() {
        for path in ["", "../docs/evidence/a.json", "/docs/evidence/a.json"] {
            assert!(validate_registry_path(path).is_err());
        }
        assert!(!is_canonical_sha256("not-a-digest"));
        assert!(!is_canonical_sha256(&"a".repeat(64)));
        assert!(is_canonical_sha256(&format!("sha256:{}", "a".repeat(64))));
    }
}
