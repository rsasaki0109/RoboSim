//! Shared fail-closed primitives for independently owned evidence submissions.

use super::RELEASE_VERSION;
use anyhow::{bail, Context};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::ffi::OsStr;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

pub(crate) const MAX_EVIDENCE_BYTES: u64 = 64 * 1024 * 1024;
pub(crate) const MAX_LOG_BYTES: u64 = 16 * 1024 * 1024;
pub(crate) const MAX_RELEASE_ARCHIVE_BYTES: u64 = 2 * 1024 * 1024 * 1024;

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Repository {
    pub(crate) owner: String,
    pub(crate) url: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ReleaseIdentity {
    pub(crate) tag: String,
    pub(crate) target: String,
    pub(crate) archive: Artifact,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Platform {
    pub(crate) operating_system: String,
    pub(crate) architecture: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Artifact {
    pub(crate) url: String,
    pub(crate) file_name: String,
    pub(crate) size_bytes: u64,
    pub(crate) sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Reproduction {
    pub(crate) commands: Vec<String>,
    pub(crate) exit_statuses: Vec<i32>,
    pub(crate) stdout_log_path: String,
    pub(crate) stderr_log_path: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct MemberDigest {
    pub(crate) path: String,
    pub(crate) size_bytes: u64,
    pub(crate) sha256: String,
}

pub(crate) fn validate_platform(platform: &Platform, target: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        platform.architecture == "x86_64"
            && matches!(
                (platform.operating_system.as_str(), target),
                ("windows", "x86_64-pc-windows-msvc") | ("linux", "x86_64-unknown-linux-gnu")
            ),
        "external evidence platform and release target differ"
    );
    Ok(())
}

pub(crate) fn release_archive_name(target: &str) -> anyhow::Result<String> {
    let suffix = match target {
        "x86_64-pc-windows-msvc" => "zip",
        "x86_64-unknown-linux-gnu" => "tar.gz",
        _ => bail!("unsupported external evidence release target {target}"),
    };
    Ok(format!("rne-{RELEASE_VERSION}-{target}.{suffix}"))
}

pub(crate) fn validate_external_repository(
    owner: &str,
    url: &str,
    revision: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        !owner.eq_ignore_ascii_case("rsasaki0109")
            && !owner.is_empty()
            && owner.len() <= 39
            && owner
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            && !owner.starts_with('-')
            && !owner.ends_with('-'),
        "external evidence owner must be an independent canonical GitHub owner"
    );
    let prefix = format!("https://github.com/{owner}/");
    anyhow::ensure!(
        url.starts_with(&prefix)
            && url.len() > prefix.len()
            && url.len() <= 256
            && !url[prefix.len()..].contains('/')
            && !url.contains('?')
            && !url.contains('#')
            && url.is_ascii(),
        "external evidence repository must be one public GitHub repository owned by {owner}"
    );
    anyhow::ensure!(
        revision.len() == 40 && revision.bytes().all(is_lower_hex),
        "external evidence revision must be 40 lowercase hexadecimal characters"
    );
    Ok(())
}

pub(crate) fn validate_artifact_shape(artifact: &Artifact, label: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        artifact.url.starts_with("https://")
            && artifact.url.is_ascii()
            && artifact.url.len() <= 2048
            && !artifact.url.contains('?')
            && !artifact.url.contains('#')
            && !artifact.file_name.is_empty()
            && artifact.file_name.len() <= 255
            && Path::new(&artifact.file_name).file_name() == Some(OsStr::new(&artifact.file_name))
            && artifact.url.ends_with(&format!("/{}", artifact.file_name))
            && artifact.size_bytes > 0
            && artifact.sha256.len() == 64
            && artifact.sha256.bytes().all(is_lower_hex),
        "external evidence {label} identity is invalid"
    );
    Ok(())
}

pub(crate) fn validate_artifact(
    submitted: &Artifact,
    path: &Path,
    label: &str,
    maximum_bytes: u64,
) -> anyhow::Result<MemberDigest> {
    let digest = digest_file(path, label, maximum_bytes)?;
    anyhow::ensure!(
        submitted.file_name == digest.path
            && submitted.size_bytes == digest.size_bytes
            && submitted.sha256 == digest.sha256,
        "external evidence {label} bytes differ from the submission candidate"
    );
    Ok(digest)
}

pub(crate) fn digest_file(
    path: &Path,
    label: &str,
    maximum_bytes: u64,
) -> anyhow::Result<MemberDigest> {
    let bytes = read_regular_file(path, label, maximum_bytes)?;
    let name = path
        .file_name()
        .and_then(OsStr::to_str)
        .with_context(|| format!("external evidence {label} name is not UTF-8"))?;
    Ok(MemberDigest {
        path: name.to_string(),
        size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_bytes(&bytes),
    })
}

pub(crate) fn read_regular_file(
    path: &Path,
    label: &str,
    maximum_bytes: u64,
) -> anyhow::Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect external evidence {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file()
            && !metadata.file_type().is_symlink()
            && metadata.len() > 0
            && metadata.len() <= maximum_bytes,
        "external evidence {label} must be a non-empty regular non-symlink file no larger than {maximum_bytes} bytes"
    );
    fs::read(path).with_context(|| format!("read external evidence {label} {}", path.display()))
}

pub(crate) fn validate_repository_checkout(
    root: &Path,
    expected_url: &str,
    expected_revision: &str,
) -> anyhow::Result<()> {
    let metadata = fs::symlink_metadata(root)
        .with_context(|| format!("inspect external evidence repository {}", root.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
        "external evidence repository must be a real non-symlink directory"
    );
    anyhow::ensure!(
        git_output(root, &["rev-parse", "HEAD"])? == expected_revision,
        "external evidence repository HEAD differs from the submitted revision"
    );
    anyhow::ensure!(
        git_output(root, &["status", "--porcelain", "--untracked-files=all"])?.is_empty(),
        "external evidence repository must be clean including untracked files"
    );
    let origin = git_output(root, &["remote", "get-url", "origin"])?;
    anyhow::ensure!(
        origin.strip_suffix(".git").unwrap_or(&origin)
            == expected_url.strip_suffix(".git").unwrap_or(expected_url),
        "external evidence repository origin differs from the submission candidate"
    );
    Ok(())
}

pub(crate) fn validate_committed_file(
    root: &Path,
    path: &Path,
    label: &str,
) -> anyhow::Result<String> {
    let canonical_root = fs::canonicalize(root)?;
    let canonical_path = fs::canonicalize(path)?;
    let relative = canonical_path
        .strip_prefix(&canonical_root)
        .with_context(|| format!("external {label} is outside its evidence repository"))?;
    let relative_text = relative.to_string_lossy().replace('\\', "/");
    validate_relative_path(&relative_text)?;
    let object = format!("HEAD:{relative_text}");
    let output = Command::new("git")
        .current_dir(&canonical_root)
        .args(["show", "--no-textconv", &object])
        .output()?;
    anyhow::ensure!(
        output.status.success(),
        "external {label} is not committed at the submitted revision"
    );
    anyhow::ensure!(
        output.stdout == fs::read(&canonical_path)?,
        "external {label} working bytes differ from the submitted revision"
    );
    Ok(relative_text)
}

pub(crate) fn resolve_repository_member(
    root: &Path,
    relative: &str,
    label: &str,
) -> anyhow::Result<PathBuf> {
    validate_relative_path(relative)?;
    let canonical_root = fs::canonicalize(root)?;
    let path = root.join(relative);
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("inspect external {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "external {label} must be a regular non-symlink file"
    );
    let canonical = fs::canonicalize(&path)?;
    anyhow::ensure!(
        canonical.starts_with(canonical_root),
        "external {label} escapes its evidence repository"
    );
    Ok(canonical)
}

pub(crate) fn validate_relative_path(path: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !path.is_empty()
            && !path.contains('\\')
            && !path.chars().any(char::is_control)
            && !Path::new(path).is_absolute()
            && Path::new(path)
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "external evidence path must be a canonical relative path"
    );
    Ok(())
}

pub(crate) fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn git_output(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn is_lower_hex(byte: u8) -> bool {
    byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)
}

pub(crate) fn absolute_from(root: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    }
}
