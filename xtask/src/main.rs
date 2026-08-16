//! Workspace automation tasks for Robot Native Engine.

mod accelerator;
mod benchmark;
mod capability_report;
mod dataset;
mod evidence;
mod failure_capsule;
mod lekiwi_evidence;
mod release_artifacts;
mod release_exit;
mod release_readiness;
mod task_scale;

use anyhow::Context;
use image::AnimationDecoder;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::io::BufReader;
use std::process::{Command, ExitCode, Stdio};
use std::{
    env, fs,
    path::{Component, Path, PathBuf},
    time::{Instant, SystemTime, UNIX_EPOCH},
};

const HERO_CONTACT_SHEET_FRAMES: [usize; 9] = [0, 6, 12, 18, 24, 30, 36, 42, 47];
const DEFAULT_BEHAVIOR_SEED_RANGE: &str = "0..10";
pub(crate) const RELEASE_VERSION: &str = "0.1.0";
const RELEASE_MSRV: &str = "1.88.0";
const SUPPLY_CHAIN_POLICY_DATE: &str = "2026-08-12";
const CARGO_DENY_VERSION: &str = "0.20.2";
const CARGO_AUDIT_VERSION: &str = "0.22.2";
const RUST_API_BASELINE_SCHEMA_VERSION: u32 = 1;
const CARGO_SEMVER_CHECKS_VERSION: &str = "0.49.0";
pub(crate) const FLAGSHIP_WORKFLOW_REPORT_KIND: &str = "rne_flagship_workflow_report";
pub(crate) const FLAGSHIP_WORKFLOW_REPORT_SCHEMA_VERSION: u32 = 1;
pub(crate) const FLAGSHIP_CROSS_BACKEND_REPORT_KIND: &str = "rne_flagship_cross_backend_report";
pub(crate) const FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION: u32 = 1;
const PUBLIC_RELEASE_PACKAGES: &[&str] = &[
    "rne_adapter_ros2",
    "rne_ai",
    "rne_assets",
    "rne_core",
    "rne_data",
    "rne_deformable",
    "rne_ecs",
    "rne_hardware_gateway",
    "rne_hardware_lekiwi",
    "rne_log",
    "rne_math",
    "rne_mjcf",
    "rne_openscenario",
    "rne_physics",
    "rne_physics_conformance",
    "rne_physics_analytic",
    "rne_physics_rapier",
    "rne_plateau",
    "rne_plugin_sdk",
    "rne_plugin",
    "rne_py",
    "rne_render",
    "rne_render_wgpu",
    "rne_robot",
    "rne_sdf",
    "rne_sensor",
    "rne_sumo",
    "rne_traci",
    "rne_traffic",
    "rne_urdf_import",
    "rne_world",
];

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("xtask error: {error:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> anyhow::Result<()> {
    let mut args = env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "ci".to_string());

    match command.as_str() {
        "ci" => ci(),
        "ci-lint" => ci_lint(),
        "ci-test" => match args.next() {
            Some(partition) => ci_test_partition(Some(&partition)),
            None => ci_test(),
        },
        "ci-smoke" => {
            let partition = args.next();
            anyhow::ensure!(
                args.next().is_none(),
                "ci-smoke accepts at most one partition: manipulator, locomotion, assets, or media"
            );
            ci_smoke(partition.as_deref())
        }
        "ci-headless" => ci_headless(),
        "ci-rl" => ci_rl(),
        "ci-ros2" => ci_ros2(),
        "ci-ros2-bridge" => ci_ros2_bridge(),
        "physics-conformance" => physics_conformance(&mut args),
        "scenario-scale" => scenario_scale(&mut args),
        "parity" => parity(&mut args),
        "house-gif-demo" => house_gif_demo(),
        "hero-media-check" => hero_media_check(),
        "hero-contact-sheet" => hero_contact_sheet(),
        "behavior-ci" => behavior_ci(&mut args),
        "behavior-replay" => behavior_replay(&mut args),
        "flagship" => flagship(&mut args),
        "release-check" => release_check(&mut args),
        "release-bundle" => release_artifacts::release_bundle(&mut args),
        "release-install-smoke" => release_artifacts::release_install_smoke(&mut args),
        "release-exit" => release_exit::release_exit(&mut args),
        "release-readiness" => release_readiness::release_readiness(&mut args),
        "capability-report" => capability_report::capability_report(&mut args),
        "benchmark" => benchmark::benchmark(&mut args),
        "task-scale" => task_scale::task_scale(&mut args),
        "accelerator-check" => accelerator::accelerator_check(&mut args),
        "accelerator-conformance" => accelerator::accelerator_conformance(&mut args),
        "accelerator-scale" => accelerator::accelerator_scale(&mut args),
        "dataset-check" => dataset::dataset_check(&mut args),
        "dataset-evaluate-depth" => dataset::dataset_evaluate_depth(&mut args),
        "evidence" => evidence::evidence(&mut args),
        "failure-capsule" => failure_capsule::run(&mut args),
        "lekiwi-evidence" => lekiwi_evidence::run(&mut args),
        "supply-chain" => supply_chain(&mut args),
        "fuzz-smoke" => fuzz_smoke(&mut args),
        "asset" => asset_command(&mut args),
        "lint-boundaries" => lint_boundaries(),
        other => anyhow::bail!("unknown xtask command: {other}"),
    }
}

/// Validates the frozen 1.0 RC metadata and assembles every publishable crate.
fn release_check(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut allow_dirty = false;
    for argument in args {
        match argument.as_str() {
            "--allow-dirty" => allow_dirty = true,
            other => anyhow::bail!("unknown release-check argument: {other}"),
        }
    }

    let root = workspace_root()?;
    let metadata = cargo_metadata(&root)?;
    validate_release_metadata(&metadata)?;
    validate_public_docs(&metadata)?;
    let rust_api_baseline: RustApiBaselineRegistry = toml::from_str(&fs::read_to_string(
        root.join("release/rust-api-baseline.toml"),
    )?)?;
    validate_rust_api_baseline(&root, &metadata, &rust_api_baseline)?;

    let blocker_text = fs::read_to_string(root.join("release/blockers.toml"))?;
    let blocker_registry = blocker_text.parse::<toml::Value>()?;
    validate_blocker_registry(&blocker_registry)?;
    let contract_text = fs::read_to_string(root.join("release/contracts.toml"))?;
    let contract_registry = contract_text.parse::<toml::Value>()?;
    validate_contract_registry(&contract_registry)?;
    let compatibility = rne_compatibility_suite::run_compatibility(
        &root,
        &root.join("release/compatibility-fixtures.toml"),
    )?;
    anyhow::ensure!(
        compatibility.passed,
        "one or more release compatibility fixtures failed"
    );
    rne_compatibility_suite::verify_historical_source_history(&root)?;
    release_exit::validate_exit_matrix(&root)?;
    release_readiness::validate_committed_manifest(&root)?;
    release_readiness::enforce_release_promotion(&root)?;

    run_cargo_at(
        &root,
        &["doc", "--locked", "--workspace", "--no-deps"],
        &[("RUSTDOCFLAGS", "-D warnings")],
    )?;

    let mut package_args = vec![
        "package".to_string(),
        "--locked".to_string(),
        "--no-verify".to_string(),
    ];
    if allow_dirty {
        package_args.push("--allow-dirty".to_string());
    }
    for package in PUBLIC_RELEASE_PACKAGES {
        package_args.push("-p".to_string());
        package_args.push((*package).to_string());
    }
    run_cargo_owned_at(&root, &package_args, &[])?;

    println!(
        "release metadata ok: version={RELEASE_VERSION} msrv={RELEASE_MSRV} public_packages={} rust_api_baseline={}",
        PUBLIC_RELEASE_PACKAGES.len(),
        rust_api_baseline.baseline_revision
    );
    Ok(())
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RustApiBaselineRegistry {
    schema_version: u32,
    release_version: String,
    baseline_revision: String,
    baseline_tree: String,
    cargo_semver_checks_version: String,
    package: Vec<RustApiBaselinePackage>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RustApiBaselinePackage {
    name: String,
    manifest_path: String,
}

fn validate_rust_api_baseline(
    root: &Path,
    metadata: &serde_json::Value,
    registry: &RustApiBaselineRegistry,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        registry.schema_version == RUST_API_BASELINE_SCHEMA_VERSION,
        "Rust API baseline schema must be {RUST_API_BASELINE_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        registry.release_version == RELEASE_VERSION,
        "Rust API baseline release must be {RELEASE_VERSION}"
    );
    anyhow::ensure!(
        registry.cargo_semver_checks_version == CARGO_SEMVER_CHECKS_VERSION,
        "Rust API baseline must pin cargo-semver-checks {CARGO_SEMVER_CHECKS_VERSION}"
    );
    for (field, value) in [
        ("baseline_revision", registry.baseline_revision.as_str()),
        ("baseline_tree", registry.baseline_tree.as_str()),
    ] {
        anyhow::ensure!(
            is_lower_git_object_id(value),
            "Rust API baseline {field} must be a 40-character lowercase Git object ID"
        );
    }
    anyhow::ensure!(
        registry.package.len() == PUBLIC_RELEASE_PACKAGES.len(),
        "Rust API baseline package count changed: expected {}, got {}",
        PUBLIC_RELEASE_PACKAGES.len(),
        registry.package.len()
    );

    let packages = metadata["packages"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted packages"))?;
    let mut names = BTreeSet::new();
    let mut manifest_paths = BTreeSet::new();
    for (entry, expected_name) in registry.package.iter().zip(PUBLIC_RELEASE_PACKAGES) {
        anyhow::ensure!(
            entry.name == *expected_name,
            "Rust API baseline package order changed: expected {expected_name}, got {}",
            entry.name
        );
        anyhow::ensure!(
            names.insert(entry.name.as_str()),
            "Rust API baseline duplicates package {}",
            entry.name
        );
        anyhow::ensure!(
            manifest_paths.insert(entry.manifest_path.as_str()),
            "Rust API baseline duplicates manifest {}",
            entry.manifest_path
        );
        validate_baseline_manifest_path(&entry.manifest_path)?;

        let package = packages
            .iter()
            .find(|package| package["name"].as_str() == Some(entry.name.as_str()))
            .with_context(|| format!("cargo metadata omitted baseline package {}", entry.name))?;
        let current_manifest = package["manifest_path"]
            .as_str()
            .with_context(|| format!("package {} omitted manifest_path", entry.name))?;
        let current_relative = Path::new(current_manifest)
            .strip_prefix(root)
            .with_context(|| format!("package {} manifest escaped workspace", entry.name))?
            .to_string_lossy()
            .replace('\\', "/");
        anyhow::ensure!(
            current_relative == entry.manifest_path,
            "Rust API baseline manifest moved for {}: expected {}, got {}",
            entry.name,
            entry.manifest_path,
            current_relative
        );
    }

    let commit_object = format!("{}^{{commit}}", registry.baseline_revision);
    ensure_git_success(
        root,
        &["cat-file", "-e", &commit_object],
        "Rust API baseline commit is unavailable",
    )?;
    let actual_tree = git_stdout(
        root,
        &["show", "-s", "--format=%T", &registry.baseline_revision],
    )?;
    anyhow::ensure!(
        actual_tree == registry.baseline_tree,
        "Rust API baseline tree changed: expected {}, got {}",
        registry.baseline_tree,
        actual_tree
    );
    ensure_git_success(
        root,
        &[
            "merge-base",
            "--is-ancestor",
            &registry.baseline_revision,
            "HEAD",
        ],
        "Rust API baseline must remain an ancestor of HEAD",
    )?;
    for entry in &registry.package {
        let object = format!("{}:{}", registry.baseline_revision, entry.manifest_path);
        ensure_git_success(
            root,
            &["cat-file", "-e", &object],
            &format!("Rust API baseline omitted {}", entry.manifest_path),
        )?;
    }
    Ok(())
}

fn validate_baseline_manifest_path(path: &str) -> anyhow::Result<()> {
    anyhow::ensure!(!path.is_empty(), "Rust API baseline manifest path is empty");
    anyhow::ensure!(
        !path.contains('\\'),
        "Rust API baseline manifest path must use forward slashes: {path}"
    );
    let parsed = Path::new(path);
    anyhow::ensure!(
        !parsed.is_absolute()
            && parsed
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "Rust API baseline manifest path is not canonical: {path}"
    );
    anyhow::ensure!(
        path.ends_with("/Cargo.toml"),
        "Rust API baseline manifest path must end in /Cargo.toml: {path}"
    );
    Ok(())
}

fn is_lower_git_object_id(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn ensure_git_success(root: &Path, args: &[&str], context: &str) -> anyhow::Result<()> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "{context}: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(())
}

fn git_stdout(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(String::from_utf8(output.stdout)?.trim().to_string())
}

#[derive(Debug, Deserialize)]
struct SupplyChainExceptionRegistry {
    schema_version: u32,
    release_version: String,
    policy_date: String,
    #[serde(default)]
    advisory: Vec<AdvisoryException>,
    #[serde(default)]
    duplicate: Vec<DuplicateException>,
}

#[derive(Debug, Deserialize)]
struct AdvisoryException {
    id: String,
    package: String,
    version: String,
    category: String,
    reachability: String,
    owner: String,
    rationale: String,
    mitigation: String,
    expires: String,
}

#[derive(Debug, Deserialize)]
struct DuplicateException {
    package: String,
    version: String,
    reachability: String,
    owner: String,
    rationale: String,
    expires: String,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct CargoSbom {
    schema_version: u32,
    release_version: &'static str,
    generated_from: &'static str,
    cargo_lock_sha256: String,
    accepted_advisories: Vec<SbomAcceptedAdvisory>,
    packages: Vec<SbomPackage>,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct SbomAcceptedAdvisory {
    id: String,
    package: String,
    version: String,
    expires: String,
}

#[derive(Debug, Serialize, PartialEq, Eq)]
struct SbomPackage {
    bom_ref: String,
    name: String,
    version: String,
    source: Option<String>,
    checksum: Option<String>,
    license: Option<String>,
    workspace: bool,
    features: Vec<String>,
    dependencies: Vec<String>,
}

/// Checks the pinned dependency policy and emits deterministic supply-chain evidence.
fn supply_chain(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut output_dir = PathBuf::from("artifacts/supply-chain");
    let mut check_tools = true;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output-dir" => {
                output_dir = PathBuf::from(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--output-dir requires a path"))?,
                );
            }
            "--generate-only" => check_tools = false,
            other => anyhow::bail!("unknown supply-chain argument: {other}"),
        }
    }

    let root = workspace_root()?;
    let registry_text = fs::read_to_string(root.join("release/supply-chain-exceptions.toml"))?;
    let registry: SupplyChainExceptionRegistry = toml::from_str(&registry_text)?;
    let lock_bytes = fs::read(root.join("Cargo.lock"))?;
    let lock: toml::Value = toml::from_str(std::str::from_utf8(&lock_bytes)?)?;
    validate_supply_chain_registry(&registry, &lock, current_unix_days()?)?;

    let deny_text = fs::read_to_string(root.join("deny.toml"))?;
    let deny: toml::Value = toml::from_str(&deny_text)?;
    validate_deny_exceptions(&registry, &deny)?;

    if check_tools {
        verify_tool_version("cargo-deny", CARGO_DENY_VERSION)?;
        verify_tool_version("cargo-audit", CARGO_AUDIT_VERSION)?;
        run_supply_tool(&root, "cargo-deny", &["check"])?;

        let mut audit_args = vec!["audit", "--deny", "warnings"];
        for exception in &registry.advisory {
            audit_args.push("--ignore");
            audit_args.push(&exception.id);
        }
        run_supply_tool(&root, "cargo-audit", &audit_args)?;
    }

    let metadata = cargo_metadata_full(&root)?;
    let lock_digest = sha256_hex(&lock_bytes);
    let sbom = build_cargo_sbom(&metadata, &registry, lock_digest.clone())?;
    let output_dir = if output_dir.is_absolute() {
        output_dir
    } else {
        root.join(output_dir)
    };
    fs::create_dir_all(&output_dir)?;
    let mut sbom_json = serde_json::to_vec_pretty(&sbom)?;
    sbom_json.push(b'\n');
    fs::write(output_dir.join("sbom.cargo.json"), sbom_json)?;
    fs::write(
        output_dir.join("cargo-lock.sha256"),
        format!("{lock_digest}  Cargo.lock\n"),
    )?;

    println!(
        "supply-chain evidence ok: packages={} advisories={} output={}",
        sbom.packages.len(),
        sbom.accepted_advisories.len(),
        output_dir.display()
    );
    Ok(())
}

/// Runs deterministic parser campaigns and emits panic-free fuzz evidence.
fn fuzz_smoke(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut output_dir = PathBuf::from("artifacts/fuzz-smoke");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output-dir" => {
                output_dir = PathBuf::from(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--output-dir requires a path"))?,
                );
            }
            other => anyhow::bail!("unknown fuzz-smoke argument: {other}"),
        }
    }

    let report = rne_fuzz_smoke::run_fuzz_smoke_campaign();
    report.validate().map_err(anyhow::Error::msg)?;
    let root = workspace_root()?;
    let output_dir = if output_dir.is_absolute() {
        output_dir
    } else {
        root.join(output_dir)
    };
    fs::create_dir_all(&output_dir)?;
    let report_path = output_dir.join("report.json");
    let mut json = serde_json::to_vec_pretty(&report)?;
    json.push(b'\n');
    fs::write(&report_path, json)?;
    println!(
        "fuzz-smoke evidence ok: boundaries={} cases={} digest={} output={}",
        report.boundaries.len(),
        report.total_cases,
        report.campaign_digest_sha256,
        report_path.display()
    );
    Ok(())
}

fn cargo_metadata_full(root: &Path) -> anyhow::Result<serde_json::Value> {
    let output = Command::new("cargo")
        .current_dir(root)
        .args([
            "metadata",
            "--locked",
            "--format-version",
            "1",
            "--all-features",
        ])
        .output()?;
    anyhow::ensure!(
        output.status.success(),
        "cargo metadata for the supply-chain graph failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    Ok(serde_json::from_slice(&output.stdout)?)
}

fn validate_supply_chain_registry(
    registry: &SupplyChainExceptionRegistry,
    lock: &toml::Value,
    today_days: u64,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        registry.schema_version == 1,
        "supply-chain exception schema_version must be 1"
    );
    anyhow::ensure!(
        registry.release_version == RELEASE_VERSION,
        "supply-chain exception release_version must be {RELEASE_VERSION}"
    );
    anyhow::ensure!(
        registry.policy_date == SUPPLY_CHAIN_POLICY_DATE,
        "supply-chain policy_date must be {SUPPLY_CHAIN_POLICY_DATE}"
    );
    let policy_days = parse_utc_date_days(&registry.policy_date)?;
    anyhow::ensure!(
        today_days >= policy_days,
        "current date predates the supply-chain policy"
    );

    let packages = lock
        .get("package")
        .and_then(toml::Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("Cargo.lock omitted package entries"))?;
    let lock_contains = |name: &str, version: &str| {
        packages.iter().any(|package| {
            package.get("name").and_then(toml::Value::as_str) == Some(name)
                && package.get("version").and_then(toml::Value::as_str) == Some(version)
        })
    };

    let mut advisory_ids = BTreeSet::new();
    for exception in &registry.advisory {
        anyhow::ensure!(
            exception.id.starts_with("RUSTSEC-"),
            "invalid advisory id {}",
            exception.id
        );
        anyhow::ensure!(
            advisory_ids.insert(exception.id.as_str()),
            "duplicate advisory exception {}",
            exception.id
        );
        anyhow::ensure!(
            matches!(
                exception.category.as_str(),
                "unmaintained" | "unsound" | "vulnerability"
            ),
            "unsupported advisory category {}",
            exception.category
        );
        validate_exception_text(
            &exception.package,
            &exception.version,
            &exception.reachability,
            &exception.owner,
            &exception.rationale,
            &exception.expires,
            today_days,
        )?;
        anyhow::ensure!(
            !exception.mitigation.trim().is_empty(),
            "advisory {} must document mitigation",
            exception.id
        );
        anyhow::ensure!(
            lock_contains(&exception.package, &exception.version),
            "advisory exception {} does not match Cargo.lock",
            exception.id
        );
    }

    let mut duplicate_packages = BTreeSet::new();
    for exception in &registry.duplicate {
        validate_exception_text(
            &exception.package,
            &exception.version,
            &exception.reachability,
            &exception.owner,
            &exception.rationale,
            &exception.expires,
            today_days,
        )?;
        anyhow::ensure!(
            duplicate_packages.insert((exception.package.as_str(), exception.version.as_str())),
            "duplicate dependency exception {}@{}",
            exception.package,
            exception.version
        );
        anyhow::ensure!(
            lock_contains(&exception.package, &exception.version),
            "duplicate exception {}@{} does not match Cargo.lock",
            exception.package,
            exception.version
        );
    }
    Ok(())
}

fn validate_exception_text(
    package: &str,
    version: &str,
    reachability: &str,
    owner: &str,
    rationale: &str,
    expires: &str,
    today_days: u64,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        !package.trim().is_empty(),
        "exception package must not be empty"
    );
    anyhow::ensure!(
        !version.trim().is_empty(),
        "exception version must not be empty"
    );
    anyhow::ensure!(
        !reachability.trim().is_empty(),
        "{package}@{version} must document reachability"
    );
    anyhow::ensure!(
        !owner.trim().is_empty(),
        "{package}@{version} must document an owner"
    );
    anyhow::ensure!(
        !rationale.trim().is_empty(),
        "{package}@{version} must document a rationale"
    );
    let expiry_days = parse_utc_date_days(expires)?;
    anyhow::ensure!(
        expiry_days >= today_days,
        "{package}@{version} exception expired on {expires}"
    );
    Ok(())
}

fn validate_deny_exceptions(
    registry: &SupplyChainExceptionRegistry,
    deny: &toml::Value,
) -> anyhow::Result<()> {
    let configured_advisories = deny
        .get("advisories")
        .and_then(|value| value.get("ignore"))
        .and_then(toml::Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("deny.toml advisories.ignore must be an array"))?
        .iter()
        .map(|value| {
            value
                .as_str()
                .ok_or_else(|| anyhow::anyhow!("advisory ignores must be RustSec ID strings"))
        })
        .collect::<anyhow::Result<BTreeSet<_>>>()?;
    let documented_advisories = registry
        .advisory
        .iter()
        .map(|exception| exception.id.as_str())
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        configured_advisories == documented_advisories,
        "deny.toml advisory ignores differ from the exception registry"
    );

    let configured_duplicates = deny
        .get("bans")
        .and_then(|value| value.get("skip"))
        .and_then(toml::Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("deny.toml bans.skip must be an array"))?
        .iter()
        .map(|value| {
            let package_spec = value
                .get("crate")
                .and_then(toml::Value::as_str)
                .ok_or_else(|| anyhow::anyhow!("bans.skip entries must declare crate"))?;
            let (package, version) = package_spec
                .split_once('@')
                .ok_or_else(|| anyhow::anyhow!("bans.skip must use package@version"))?;
            Ok((package, version))
        })
        .collect::<anyhow::Result<BTreeSet<_>>>()?;
    let documented_duplicates = registry
        .duplicate
        .iter()
        .map(|exception| (exception.package.as_str(), exception.version.as_str()))
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        configured_duplicates == documented_duplicates,
        "deny.toml duplicate skips differ from the exception registry"
    );
    anyhow::ensure!(
        deny.get("bans")
            .and_then(|value| value.get("multiple-versions"))
            .and_then(toml::Value::as_str)
            == Some("deny"),
        "deny.toml must deny unaccepted duplicate versions"
    );
    anyhow::ensure!(
        deny.get("bans")
            .and_then(|value| value.get("wildcards"))
            .and_then(toml::Value::as_str)
            == Some("deny"),
        "deny.toml must deny wildcard dependency requirements"
    );
    Ok(())
}

fn verify_tool_version(program: &str, expected: &str) -> anyhow::Result<()> {
    let output = Command::new(program).arg("--version").output()?;
    anyhow::ensure!(output.status.success(), "{program} --version failed");
    let version = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    anyhow::ensure!(
        version.split_whitespace().any(|token| token == expected),
        "{program} must be version {expected}, got {}",
        version.trim()
    );
    Ok(())
}

fn run_supply_tool(root: &Path, program: &str, args: &[&str]) -> anyhow::Result<()> {
    println!("$ {program} {}", args.join(" "));
    let status = Command::new(program)
        .current_dir(root)
        .args(args)
        .status()?;
    anyhow::ensure!(status.success(), "{program} failed with status {status}");
    Ok(())
}

fn build_cargo_sbom(
    metadata: &serde_json::Value,
    registry: &SupplyChainExceptionRegistry,
    lock_digest: String,
) -> anyhow::Result<CargoSbom> {
    let packages = metadata["packages"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted packages"))?;
    let member_ids = metadata["workspace_members"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted workspace_members"))?
        .iter()
        .filter_map(serde_json::Value::as_str)
        .collect::<BTreeSet<_>>();
    let nodes = metadata["resolve"]["nodes"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted resolve nodes"))?;
    let nodes_by_id = nodes
        .iter()
        .filter_map(|node| node["id"].as_str().map(|id| (id, node)))
        .collect::<BTreeMap<_, _>>();

    let mut refs_by_id = BTreeMap::new();
    for package in packages {
        let id = json_string(package, "id")?;
        let name = json_string(package, "name")?;
        let version = json_string(package, "version")?;
        let source = package["source"].as_str();
        let workspace = member_ids.contains(id);
        let bom_ref = if workspace {
            format!("workspace:{name}@{version}")
        } else if let Some(source) = source {
            format!("{source}:{name}@{version}")
        } else {
            format!("path:{name}@{version}")
        };
        refs_by_id.insert(id, bom_ref);
    }

    let mut sbom_packages = Vec::with_capacity(packages.len());
    for package in packages {
        let id = json_string(package, "id")?;
        let node = nodes_by_id
            .get(id)
            .ok_or_else(|| anyhow::anyhow!("resolve graph omitted package {id}"))?;
        let mut dependencies = node["dependencies"]
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("resolve node {id} omitted dependencies"))?
            .iter()
            .map(|dependency| {
                let dependency_id = dependency
                    .as_str()
                    .ok_or_else(|| anyhow::anyhow!("dependency ID must be a string"))?;
                refs_by_id
                    .get(dependency_id)
                    .cloned()
                    .ok_or_else(|| anyhow::anyhow!("unknown dependency ID {dependency_id}"))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        dependencies.sort();
        dependencies.dedup();
        let mut features = node["features"]
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("resolve node {id} omitted features"))?
            .iter()
            .map(|feature| {
                feature
                    .as_str()
                    .map(str::to_owned)
                    .ok_or_else(|| anyhow::anyhow!("feature must be a string"))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        features.sort();
        features.dedup();

        sbom_packages.push(SbomPackage {
            bom_ref: refs_by_id[id].clone(),
            name: json_string(package, "name")?.to_owned(),
            version: json_string(package, "version")?.to_owned(),
            source: package["source"].as_str().map(str::to_owned),
            checksum: package["checksum"].as_str().map(str::to_owned),
            license: package["license"].as_str().map(str::to_owned),
            workspace: member_ids.contains(id),
            features,
            dependencies,
        });
    }
    sbom_packages.sort_by(|left, right| left.bom_ref.cmp(&right.bom_ref));

    let mut accepted_advisories = registry
        .advisory
        .iter()
        .map(|exception| SbomAcceptedAdvisory {
            id: exception.id.clone(),
            package: exception.package.clone(),
            version: exception.version.clone(),
            expires: exception.expires.clone(),
        })
        .collect::<Vec<_>>();
    accepted_advisories.sort_by(|left, right| left.id.cmp(&right.id));

    Ok(CargoSbom {
        schema_version: 1,
        release_version: RELEASE_VERSION,
        generated_from: "Cargo.lock and cargo metadata --locked --all-features",
        cargo_lock_sha256: format!("sha256:{lock_digest}"),
        accepted_advisories,
        packages: sbom_packages,
    })
}

fn json_string<'a>(value: &'a serde_json::Value, key: &str) -> anyhow::Result<&'a str> {
    value[key]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("JSON field {key} must be a string"))
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn current_unix_days() -> anyhow::Result<u64> {
    Ok(SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs() / 86_400)
}

fn parse_utc_date_days(date: &str) -> anyhow::Result<u64> {
    let parts = date
        .split('-')
        .map(str::parse::<u32>)
        .collect::<Result<Vec<_>, _>>()?;
    anyhow::ensure!(parts.len() == 3, "date must use YYYY-MM-DD: {date}");
    let (year, month, day) = (parts[0], parts[1], parts[2]);
    anyhow::ensure!(year >= 1970, "date must not predate Unix epoch: {date}");
    anyhow::ensure!((1..=12).contains(&month), "invalid month in {date}");
    let month_lengths = [
        31,
        if year % 4 == 0 && (year % 100 != 0 || year % 400 == 0) {
            29
        } else {
            28
        },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    anyhow::ensure!(
        day >= 1 && day <= month_lengths[(month - 1) as usize],
        "invalid day in {date}"
    );
    let mut days = 0_u64;
    for candidate_year in 1970..year {
        days += if candidate_year % 4 == 0
            && (candidate_year % 100 != 0 || candidate_year % 400 == 0)
        {
            366
        } else {
            365
        };
    }
    days += month_lengths
        .iter()
        .take((month - 1) as usize)
        .map(|days| u64::from(*days))
        .sum::<u64>();
    days += u64::from(day - 1);
    Ok(days)
}

fn cargo_metadata(root: &Path) -> anyhow::Result<serde_json::Value> {
    let output = Command::new("cargo")
        .current_dir(root)
        .args(["metadata", "--locked", "--format-version", "1", "--no-deps"])
        .output()?;
    anyhow::ensure!(output.status.success(), "cargo metadata failed");
    Ok(serde_json::from_slice(&output.stdout)?)
}

fn validate_release_metadata(metadata: &serde_json::Value) -> anyhow::Result<()> {
    let packages = metadata["packages"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted packages"))?;
    let member_ids = metadata["workspace_members"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted workspace_members"))?
        .iter()
        .filter_map(serde_json::Value::as_str)
        .collect::<BTreeSet<_>>();
    let members = packages
        .iter()
        .filter(|package| {
            package["id"]
                .as_str()
                .is_some_and(|id| member_ids.contains(id))
        })
        .map(|package| {
            let name = package["name"]
                .as_str()
                .ok_or_else(|| anyhow::anyhow!("workspace package omitted name"))?;
            Ok((name, package))
        })
        .collect::<anyhow::Result<BTreeMap<_, _>>>()?;

    for (name, package) in &members {
        anyhow::ensure!(
            package["rust_version"].as_str() == Some(RELEASE_MSRV),
            "workspace package {name} must declare rust-version {RELEASE_MSRV}"
        );
    }

    let actual_public = members
        .iter()
        .filter(|(_, package)| {
            package["publish"].is_null()
                || package["publish"]
                    .as_array()
                    .is_some_and(|registries| !registries.is_empty())
        })
        .map(|(name, _)| *name)
        .collect::<BTreeSet<_>>();
    let expected_public = PUBLIC_RELEASE_PACKAGES
        .iter()
        .copied()
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        actual_public == expected_public,
        "publishable package set differs: expected={expected_public:?} actual={actual_public:?}"
    );

    for name in PUBLIC_RELEASE_PACKAGES {
        let package = members
            .get(name)
            .ok_or_else(|| anyhow::anyhow!("missing public release package {name}"))?;
        anyhow::ensure!(
            package["version"].as_str() == Some(RELEASE_VERSION),
            "public package {name} must have version {RELEASE_VERSION}"
        );
        let dependencies = package["dependencies"]
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("package {name} omitted dependencies"))?;
        for dependency in dependencies {
            let Some(dependency_name) = dependency["name"].as_str() else {
                continue;
            };
            if expected_public.contains(dependency_name) {
                anyhow::ensure!(
                    dependency["req"].as_str() == Some("=0.1.0"),
                    "{name} -> {dependency_name} must use exact requirement ={RELEASE_VERSION}"
                );
            }
        }
    }
    Ok(())
}

fn validate_public_docs(metadata: &serde_json::Value) -> anyhow::Result<()> {
    let packages = metadata["packages"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("cargo metadata omitted packages"))?;
    for name in PUBLIC_RELEASE_PACKAGES {
        let package = packages
            .iter()
            .find(|package| package["name"].as_str() == Some(name))
            .ok_or_else(|| anyhow::anyhow!("missing public release package {name}"))?;
        let lib_target = package["targets"]
            .as_array()
            .and_then(|targets| {
                targets.iter().find(|target| {
                    target["src_path"]
                        .as_str()
                        .is_some_and(|path| path.replace('\\', "/").ends_with("/src/lib.rs"))
                })
            })
            .ok_or_else(|| anyhow::anyhow!("public package {name} has no library target"))?;
        let source_path = lib_target["src_path"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("library target for {name} omitted src_path"))?;
        let source = fs::read_to_string(source_path)?;
        anyhow::ensure!(
            source
                .lines()
                .any(|line| line.trim() == "#![deny(missing_docs)]"),
            "public package {name} must deny missing_docs"
        );
    }
    Ok(())
}

fn validate_blocker_registry(registry: &toml::Value) -> anyhow::Result<()> {
    anyhow::ensure!(
        registry
            .get("schema_version")
            .and_then(toml::Value::as_integer)
            == Some(1),
        "release blocker registry schema_version must be 1"
    );
    anyhow::ensure!(
        registry
            .get("release_version")
            .and_then(toml::Value::as_str)
            == Some(RELEASE_VERSION),
        "release blocker registry version must be {RELEASE_VERSION}"
    );
    let blockers = registry
        .get("blocker")
        .map(|value| {
            value
                .as_array()
                .ok_or_else(|| anyhow::anyhow!("release blocker entries must be an array"))
        })
        .transpose()?
        .map(Vec::as_slice)
        .unwrap_or_default();
    let mut blocker_ids = BTreeSet::new();
    for blocker in blockers {
        let id = blocker
            .get("id")
            .and_then(toml::Value::as_str)
            .unwrap_or("<missing-id>");
        let severity = blocker
            .get("severity")
            .and_then(toml::Value::as_str)
            .unwrap_or("<missing-severity>");
        let status = blocker
            .get("status")
            .and_then(toml::Value::as_str)
            .unwrap_or("<missing-status>");
        anyhow::ensure!(
            id != "<missing-id>" && !id.trim().is_empty(),
            "release blocker id must be non-empty"
        );
        anyhow::ensure!(
            blocker_ids.insert(id),
            "release blocker id is duplicated: {id}"
        );
        anyhow::ensure!(
            matches!(severity, "P0" | "P1" | "P2" | "P3"),
            "release blocker {id} has unsupported severity {severity:?}"
        );
        anyhow::ensure!(
            matches!(status, "open" | "closed"),
            "release blocker {id} has unsupported status {status:?}"
        );
        anyhow::ensure!(
            !matches!((severity, status), ("P0" | "P1", "open")),
            "release blocker {id} is still open at severity {severity}"
        );
        for field in ["summary", "owner", "evidence"] {
            anyhow::ensure!(
                blocker
                    .get(field)
                    .and_then(toml::Value::as_str)
                    .is_some_and(|value| !value.trim().is_empty()),
                "release blocker {id} must include non-empty {field}"
            );
        }
    }
    Ok(())
}

fn validate_contract_registry(registry: &toml::Value) -> anyhow::Result<()> {
    anyhow::ensure!(
        registry
            .get("schema_version")
            .and_then(toml::Value::as_integer)
            == Some(1),
        "release contract registry schema_version must be 1"
    );
    anyhow::ensure!(
        registry
            .get("release_version")
            .and_then(toml::Value::as_str)
            == Some(RELEASE_VERSION),
        "release contract registry version must be {RELEASE_VERSION}"
    );

    let expected = [
        (
            "controller_abi",
            "minimum",
            u64::from(rne_plugin::RNE_PLUGIN_MIN_ABI_VERSION),
        ),
        (
            "controller_abi",
            "current",
            u64::from(rne_plugin::RNE_PLUGIN_ABI_VERSION),
        ),
        (
            "controller_abi",
            "controller_schema",
            u64::from(rne_plugin::CONTROLLER_SCHEMA_VERSION),
        ),
        (
            "controller_abi",
            "conformance_report",
            u64::from(rne_plugin::CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION),
        ),
        (
            "controller_abi",
            "authoring_sdk",
            u64::from(rne_plugin::RNE_PLUGIN_SDK_VERSION),
        ),
        (
            "controller_abi",
            "c_layout",
            u64::from(rne_plugin::RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION),
        ),
        (
            "frontend_transport",
            "major",
            u64::from(rne_data::transport::TRANSPORT_PROTOCOL_MAJOR),
        ),
        (
            "frontend_transport",
            "minor",
            u64::from(rne_data::transport::TRANSPORT_PROTOCOL_MINOR),
        ),
        (
            "python",
            "api_contract",
            u64::from(release_artifacts::PYTHON_API_CONTRACT_SCHEMA_VERSION),
        ),
        (
            "python",
            "api_report",
            u64::from(release_artifacts::PYTHON_API_REPORT_SCHEMA_VERSION),
        ),
        (
            "rust",
            "public_api_baseline",
            u64::from(RUST_API_BASELINE_SCHEMA_VERSION),
        ),
        (
            "assets",
            "run_manifest",
            u64::from(rne_assets::RUN_MANIFEST_VERSION),
        ),
        (
            "assets",
            "traffic",
            u64::from(rne_traffic::TRAFFIC_ASSET_SCHEMA_VERSION),
        ),
        (
            "assets",
            "scenario_document",
            u64::from(rne_openscenario::SCENARIO_DOCUMENT_VERSION),
        ),
        (
            "replays",
            "generic_artifact",
            u64::from(rne_log::REPLAY_ARTIFACT_VERSION),
        ),
        (
            "replays",
            "command_log",
            u64::from(rne_log::REPLAY_LOG_FORMAT_VERSION),
        ),
        (
            "replays",
            "random_snapshot",
            u64::from(rne_log::REPLAY_RANDOM_SNAPSHOT_VERSION),
        ),
        (
            "replays",
            "scenario",
            u64::from(rne_openscenario::SCENARIO_REPLAY_SCHEMA_VERSION),
        ),
        (
            "replays",
            "behavior",
            u64::from(rne_ai::BEHAVIOR_REPLAY_SCHEMA_VERSION),
        ),
        (
            "replays",
            "behavior_contract",
            u64::from(rne_ai::BEHAVIOR_CONTRACT_SCHEMA_VERSION),
        ),
        (
            "replays",
            "behavior_seed_manifest",
            u64::from(rne_ai::BEHAVIOR_SEED_MANIFEST_SCHEMA_VERSION),
        ),
        (
            "replays",
            "behavior_failure_case",
            u64::from(rne_ai::BEHAVIOR_FAILURE_CASE_SCHEMA_VERSION),
        ),
        (
            "physics",
            "snapshot",
            u64::from(rne_physics::PHYSICS_SNAPSHOT_SCHEMA_VERSION),
        ),
        (
            "physics",
            "backend_manifest",
            u64::from(rne_physics::PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION),
        ),
        (
            "physics",
            "conformance_report",
            u64::from(rne_physics::PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION),
        ),
        (
            "physics",
            "external_conformance_report",
            u64::from(
                rne_physics_conformance::EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_SCHEMA_VERSION,
            ),
        ),
        (
            "physics",
            "tolerance_registry",
            u64::from(rne_physics::PHYSICS_TOLERANCE_REGISTRY_VERSION),
        ),
        (
            "determinism",
            "contract",
            u64::from(rne_core::DETERMINISM_CONTRACT_SCHEMA_VERSION),
        ),
        (
            "tasks",
            "task_spec",
            u64::from(rne_ai::TASK_SPEC_SCHEMA_VERSION),
        ),
        (
            "tasks",
            "batch_checkpoint",
            u64::from(rne_ai::PORTABLE_BATCH_CHECKPOINT_VERSION),
        ),
        (
            "tasks",
            "vectorized_episode_checkpoint",
            u64::from(rne_ai::VECTORIZED_EPISODE_CHECKPOINT_VERSION),
        ),
        (
            "snapshots",
            "mobile_manipulator_minimum",
            u64::from(rne_ai::MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION),
        ),
        (
            "snapshots",
            "mobile_manipulator_current",
            u64::from(rne_ai::MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION),
        ),
        (
            "snapshots",
            "mobile_manipulator_migration",
            u64::from(
                rne_compatibility_suite::HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
            ),
        ),
        (
            "accelerators",
            "manifest",
            u64::from(accelerator::ACCELERATOR_MANIFEST_SCHEMA_VERSION),
        ),
        (
            "accelerators",
            "protocol",
            u64::from(accelerator::ACCELERATOR_PROTOCOL_SCHEMA_VERSION),
        ),
        (
            "accelerators",
            "capability_report",
            u64::from(accelerator::ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION),
        ),
        (
            "accelerators",
            "conformance_report",
            u64::from(accelerator::ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION),
        ),
        (
            "accelerators",
            "runtime_contract",
            u64::from(accelerator::ACCELERATOR_RUNTIME_CONTRACT_SCHEMA_VERSION),
        ),
        (
            "accelerators",
            "scale_report",
            u64::from(accelerator::ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION),
        ),
        (
            "datasets",
            "bundle",
            u64::from(rne_data::DATASET_BUNDLE_SCHEMA_VERSION),
        ),
        (
            "datasets",
            "payload",
            u64::from(rne_data::DATASET_PAYLOAD_SCHEMA_VERSION),
        ),
        (
            "datasets",
            "offline_evaluation",
            u64::from(rne_data::DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "gateway_evidence",
            u64::from(rne_hardware_gateway::HARDWARE_GATEWAY_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "wire_protocol",
            u64::from(rne_hardware_gateway::wire::HARDWARE_WIRE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "wire_trace",
            u64::from(rne_hardware_gateway::wire::HARDWARE_WIRE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "session_evidence",
            u64::from(rne_hardware_gateway::wire::HARDWARE_WIRE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "shadow_comparison",
            u64::from(rne_hardware_gateway::shadow::SHADOW_COMPARISON_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "mock_conformance",
            u64::from(rne_hardware_gateway::mock::MOCK_CONFORMANCE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "adapter_conformance",
            u64::from(
                rne_hardware_gateway::conformance::HARDWARE_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION,
            ),
        ),
        (
            "hardware",
            "reference_profile",
            u64::from(rne_hardware_lekiwi::LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "lekiwi_device_bridge",
            u64::from(rne_hardware_lekiwi::LEKIWI_DEVICE_BRIDGE_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "lekiwi_reference_session",
            u64::from(rne_hardware_lekiwi::session::LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION),
        ),
        (
            "hardware",
            "lekiwi_physical_evidence",
            u64::from(
                rne_hardware_lekiwi::physical_evidence::LEKIWI_PHYSICAL_EVIDENCE_SCHEMA_VERSION,
            ),
        ),
        (
            "hardware",
            "lekiwi_power_isolation_diagnostic",
            u64::from(
                rne_hardware_lekiwi::physical_evidence::LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
            ),
        ),
        (
            "hardware",
            "lekiwi_host_termination_diagnostic",
            u64::from(
                rne_hardware_lekiwi::physical_evidence::LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
            ),
        ),
        (
            "evidence",
            "fuzz_smoke_report",
            u64::from(rne_fuzz_smoke::FUZZ_SMOKE_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "release_report",
            u64::from(release_artifacts::RELEASE_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "install_rehearsal_report",
            u64::from(release_artifacts::INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "archive_install_rehearsal_report",
            u64::from(
                release_artifacts::ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
            ),
        ),
        (
            "evidence",
            "final_exit_report",
            u64::from(release_exit::FINAL_EXIT_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "one_zero_readiness_report",
            u64::from(release_readiness::REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "one_zero_readiness_manifest",
            u64::from(release_readiness::MANIFEST_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "github_attestation_verification",
            u64::from(release_readiness::ATTESTATION_RECEIPT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "compatibility_fixture_report",
            u64::from(
                rne_compatibility_suite::COMPATIBILITY_FIXTURE_REPORT_SCHEMA_VERSION,
            ),
        ),
        (
            "evidence",
            "historical_compatibility_decision",
            u64::from(
                rne_compatibility_suite::HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
            ),
        ),
        (
            "evidence",
            "artifact_attestation_policy",
            u64::from(release_exit::ARTIFACT_ATTESTATION_POLICY_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "capability_report",
            u64::from(capability_report::CAPABILITY_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "benchmark_report",
            u64::from(benchmark::BENCHMARK_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "task_scale_report",
            u64::from(task_scale::TASK_SCALE_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "failure_capsule",
            u64::from(rne_log::FAILURE_CAPSULE_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "evidence_manifest",
            u64::from(evidence::EVIDENCE_MANIFEST_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "flagship_workflow_report",
            u64::from(FLAGSHIP_WORKFLOW_REPORT_SCHEMA_VERSION),
        ),
        (
            "evidence",
            "flagship_cross_backend_report",
            u64::from(FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION),
        ),
    ];
    for (section, key, actual) in expected {
        let declared = registry
            .get(section)
            .and_then(|value| value.get(key))
            .and_then(toml::Value::as_integer)
            .and_then(|value| u64::try_from(value).ok());
        anyhow::ensure!(
            declared == Some(actual),
            "release contract {section}.{key} must be {actual}, got {declared:?}"
        );
    }
    Ok(())
}

fn run_cargo_at(root: &Path, args: &[&str], envs: &[(&str, &str)]) -> anyhow::Result<()> {
    let owned = args
        .iter()
        .map(|value| (*value).to_string())
        .collect::<Vec<_>>();
    run_cargo_owned_at(root, &owned, envs)
}

fn run_cargo_owned_at(root: &Path, args: &[String], envs: &[(&str, &str)]) -> anyhow::Result<()> {
    println!("$ cargo {}", args.join(" "));
    let status = Command::new("cargo")
        .current_dir(root)
        .args(args)
        .envs(envs.iter().copied())
        .status()?;
    anyhow::ensure!(
        status.success(),
        "cargo command failed with status {status}"
    );
    Ok(())
}

/// Runs the committed OSS-parity flagship workflows and writes a machine-readable report.
///
/// This is intentionally a small catalog of representative gates rather than a
/// second implementation of the workspace test suite. Each check invokes the
/// same public command or integration test a contributor would use manually.
fn parity(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let mut json_path = root.join("artifacts/oss-parity/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--json" => {
                json_path = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--json requires a path"))?,
                );
            }
            other => anyhow::bail!("unknown parity argument: {other}"),
        }
    }

    let checks = [
        (
            "physics_backend_conformance",
            "cargo run --locked -q -p rne_physics_conformance_suite --bin rne-physics-conformance -- --output artifacts/physics-conformance/report.json",
        ),
        (
            "scenario_traffic_scale",
            "cargo run --locked --release -q -p rne_scenario_scale --bin rne-scenario-scale -- --output artifacts/scenario-scale/report.json",
        ),
        (
            "robot_control_replay",
            "cargo run --locked -q -p rne_asset_cli -- run assets/runs/mesh_diff_drive.rne.run.toml --replay-out target/runs/oss_parity_mesh_diff_drive.rne-replay",
        ),
        (
            "robot_replay_verify",
            "cargo run --locked -q -p rne_asset_cli -- replay target/runs/oss_parity_mesh_diff_drive.rne-replay",
        ),
        (
            "controller_oldest_abi_compatibility",
            "cargo test --locked -q -p rne_plugin --test load frozen_abi_v2_plugin_loads_and_steps_in_the_current_runtime",
        ),
        (
            "controller_multi_robot_spawn_order",
            "cargo test --locked -q -p rne_asset_cli dual_robot_controller_is_independent_of_ecs_spawn_order",
        ),
        (
            "sensor_payload_replay",
            "cargo run --locked -q -p rne_asset_cli -- run assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml --replay-out target/runs/oss_parity_mesh_diff_drive_lidar_payload.rne-replay",
        ),
        (
            "sensor_transport_protocol_golden",
            "cargo test --locked -q -p rne_data transport::tests::frame_header_has_platform_independent_golden_bytes",
        ),
        (
            "sensor_transport_binary_process_e2e",
            "cargo test --locked -q -p rne_asset_cli --test control binary_frontend",
        ),
        (
            "sensor_transport_reconnect",
            "cargo test --locked -q -p rne_asset_cli frontend_transport::tests::disconnect_does_not_quit_and_same_session_reconnects",
        ),
        (
            "scenario_traffic_run",
            "cargo run --locked -q -p rne_asset_cli -- run assets/runs/scenario_speed.rne.run.toml",
        ),
        (
            "scenario_replay_verify",
            "cargo run --locked -q -p rne_asset_cli -- replay target/runs/scenario_speed.rne-replay",
        ),
        (
            "traffic_external_pose_ownership",
            "cargo test --locked -q -p rne_traffic --test external_pose",
        ),
        (
            "traci_mirror_protocol",
            "cargo test --locked -q -p rne_traci --test co_simulation",
        ),
        (
            "runner_tcp_frontend_protocol",
            "cargo test --locked -q -p rne_asset_cli --test control",
        ),
        (
            "runner_tcp_full_resolution_rgbd_e2e",
            "cargo test --locked -q -p rne_asset_cli --test control control_tcp_full_resolution_camera_and_depth_snapshot",
        ),
        (
            "runner_remote_sensor_snapshot_contract",
            "cargo test --locked -q -p rne_asset_cli live_snapshot_contains_bounded_camera_preview",
        ),
        (
            "native_frontend_compile_contract",
            "cargo check --locked -q -p interactive_viewer --example 14_interactive_viewer",
        ),
        (
            "articulated_render_projection",
            "cargo test --locked -q -p rne_ai render_projection_applies_remote_joint_without_stepping_physics",
        ),
        (
            "frontend_remote_snapshot_contract",
            "cargo test --locked -q -p interactive_viewer --example 14_interactive_viewer remote_status_parses_scenario_traffic_positions",
        ),
        (
            "frontend_remote_sensor_projection_contract",
            "cargo test --locked -q -p interactive_viewer --example 14_interactive_viewer remote_status_decodes_camera_and_lidar_previews",
        ),
        (
            "frontend_binary_sensor_projection_contract",
            "cargo test --locked -q -p interactive_viewer --example 14_interactive_viewer binary_sensor_cache_merges_rgbd_and_lidar_into_status_snapshot",
        ),
    ];

    fs::create_dir_all(root.join("target/runs"))?;
    let mut report_checks = Vec::with_capacity(checks.len());
    let mut all_passed = true;
    for (id, command) in checks {
        let started = Instant::now();
        let (passed, output) = run_step_capture(command)?;
        all_passed &= passed;
        let output_tail = output
            .lines()
            .rev()
            .take(12)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect::<Vec<_>>()
            .join("\n");
        report_checks.push(serde_json::json!({
            "id": id,
            "command": command,
            "status": if passed { "passed" } else { "failed" },
            "duration_ms": started.elapsed().as_millis(),
            "output_tail": output_tail,
        }));
    }

    let report = serde_json::json!({
        "schema_version": 1,
        "status": if all_passed { "passed" } else { "failed" },
        "checks": report_checks,
    });
    if let Some(parent) = json_path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&json_path, serde_json::to_vec_pretty(&report)?)?;
    println!("OSS parity report: {}", json_path.display());
    anyhow::ensure!(all_passed, "one or more OSS parity checks failed");
    Ok(())
}

/// Runs the backend-neutral physics capability catalog and writes its JSON report.
fn physics_conformance(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let mut output = root.join("artifacts/physics-conformance/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--json" => {
                output = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--json requires a path"))?,
                );
            }
            other => anyhow::bail!("unknown physics-conformance argument: {other}"),
        }
    }
    let output = output.to_string_lossy().into_owned();
    run_program(
        Path::new("cargo"),
        &[
            "run",
            "--locked",
            "-q",
            "-p",
            "rne_physics_conformance_suite",
            "--bin",
            "rne-physics-conformance",
            "--",
            "--output",
            &output,
        ],
    )
}

/// Runs the release-mode 100-actor scenario scale gate and writes its JSON report.
fn scenario_scale(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let mut output = root.join("artifacts/scenario-scale/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--json" => {
                output = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--json requires a path"))?,
                );
            }
            other => anyhow::bail!("unknown scenario-scale argument: {other}"),
        }
    }
    let output = output.to_string_lossy().into_owned();
    run_program(
        Path::new("cargo"),
        &[
            "run",
            "--locked",
            "--release",
            "-q",
            "-p",
            "rne_scenario_scale",
            "--bin",
            "rne-scenario-scale",
            "--",
            "--output",
            &output,
        ],
    )
}

fn behavior_ci(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let mut seeds = default_behavior_seeds();
    let mut json_path = root.join("artifacts/behavior-ci/report.json");
    let mut junit_path = root.join("artifacts/behavior-ci/junit.xml");
    let mut artifact_dir = root.join("artifacts/behavior-ci/replays");
    let mut failure_case_path = None;
    let mut seeds_explicit = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--seeds" => {
                let value = args
                    .next()
                    .ok_or_else(|| anyhow::anyhow!("--seeds requires START..END"))?;
                seeds = parse_seed_range(&value)?;
                seeds_explicit = true;
            }
            "--json" => {
                json_path = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--json requires a path"))?,
                );
            }
            "--junit" => {
                junit_path = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--junit requires a path"))?,
                );
            }
            "--artifacts" => {
                artifact_dir = root.join(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("--artifacts requires a path"))?,
                );
            }
            "--case" => {
                failure_case_path = Some(
                    root.join(
                        args.next()
                            .ok_or_else(|| anyhow::anyhow!("--case requires a path"))?,
                    ),
                );
            }
            other => anyhow::bail!("unknown behavior-ci argument: {other}"),
        }
    }

    anyhow::ensure!(
        failure_case_path.is_none() || !seeds_explicit,
        "--case and --seeds cannot be used together"
    );
    let (run, expected_failure) = if let Some(case_path) = failure_case_path {
        let failure_case = rne_ai::BehaviorFailureCase::read_json(&case_path)?;
        let scenario = failure_case.scenario.clone();
        let dimensions = failure_case.dimensions.clone();
        let seed = failure_case.seed;
        let expected_contract = failure_case.expected_contract;
        let run = rne_ai::run_behavior_scenarios_with_replays(scenario.clone(), [seed], |seed| {
            g1_behavior_from_dimensions(&scenario, seed, &dimensions)
        })?;
        (run, Some((seed, expected_contract)))
    } else {
        let run = rne_ai::run_behavior_scenarios_with_replays(
            "unitree_g1_dex3_acquire",
            seeds,
            |seed| {
                rne_ai::UnitreeG1Dex3BehaviorScenario::new(
                    seed,
                    rne_ai::UnitreeG1Dex3BehaviorConfig::default(),
                )
            },
        )?;
        (run, None)
    };
    let mut report = run.report;
    let mut artifact_errors = Vec::new();
    for replay in run.failure_replays {
        let replay_path = artifact_dir.join(replay.file_name());
        if let Err(error) = replay.write_json(&replay_path) {
            let _ = report.set_failure_artifacts(replay.seed, None, None, None);
            artifact_errors.push(format!(
                "could not write seed {} replay: {error}",
                replay.seed
            ));
            continue;
        }
        let replay_reference = report_path(&root, &replay_path);
        let _ =
            report.set_failure_artifacts(replay.seed, Some(replay_reference.clone()), None, None);

        let minimized = rne_ai::minimize_behavior_failure(&replay, |dimensions| {
            let candidate = rne_ai::run_behavior_scenarios_with_replays(
                replay.scenario.clone(),
                [replay.seed],
                |seed| g1_behavior_from_dimensions(&replay.scenario, seed, dimensions),
            )?;
            Ok::<_, rne_ai::BehaviorReplayError>(candidate.failure_replays.into_iter().next())
        });
        let minimized = match minimized {
            Ok(minimized) => minimized,
            Err(error) => {
                artifact_errors.push(format!(
                    "could not minimize seed {} replay: {error}",
                    replay.seed
                ));
                continue;
            }
        };
        if let Err(error) =
            rne_ai::verify_behavior_replay(&minimized.artifact, |seed, dimensions| {
                g1_behavior_from_dimensions(&replay.scenario, seed, dimensions)
            })
        {
            artifact_errors.push(format!(
                "could not verify seed {} minimized replay: {error}",
                replay.seed
            ));
            continue;
        }

        let minimized_path = artifact_dir.join(minimized.artifact.minimized_file_name());
        let case_file_name = minimized
            .artifact
            .minimized_file_name()
            .replace(".rne-replay", ".behavior-case.json");
        let case_path = artifact_dir.join(case_file_name);
        let failure_case = rne_ai::BehaviorFailureCase::from_replay(&minimized.artifact);
        if let Err(error) = minimized.artifact.write_json(&minimized_path) {
            artifact_errors.push(format!(
                "could not write seed {} minimized replay: {error}",
                replay.seed
            ));
            continue;
        }
        if let Err(error) = failure_case.write_json(&case_path) {
            artifact_errors.push(format!(
                "could not write seed {} minimized case: {error}",
                replay.seed
            ));
            continue;
        }
        let _ = report.set_failure_artifacts(
            replay.seed,
            Some(replay_reference),
            Some(report_path(&root, &minimized_path)),
            Some(report_path(&root, &case_path)),
        );
    }
    if let Some(parent) = json_path.parent() {
        fs::create_dir_all(parent)?;
    }
    if let Some(parent) = junit_path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&json_path, report.to_json_pretty()?)?;
    fs::write(&junit_path, report.to_junit_xml())?;

    let passed = report
        .seeds
        .iter()
        .filter(|seed| seed.status == rne_ai::BehaviorSeedStatus::Passed)
        .count();
    println!(
        "Behavior CI: {passed}/{} seeds passed\nJSON: {}\nJUnit: {}\nReplays: {}",
        report.seeds.len(),
        json_path.display(),
        junit_path.display(),
        artifact_dir.display()
    );
    if let Some((seed, contract, violation)) = first_behavior_failure(&report) {
        eprintln!(
            "FAIL seed={seed} step={} contract={contract} state_digest={:#018x} entities={} replay={}",
            violation.step,
            violation.state_digest,
            violation.entities.join(","),
            report
                .seeds
                .iter()
                .find(|report| report.seed == seed)
                .and_then(|report| report.replay_artifact.as_deref())
                .unwrap_or("unavailable")
        );
    }
    if !artifact_errors.is_empty() {
        anyhow::bail!(
            "Behavior CI artifact processing failed:\n{}",
            artifact_errors.join("\n")
        );
    }
    if let Some((expected_seed, expected_contract)) = expected_failure {
        let reproduced = first_behavior_failure(&report).is_some_and(|(seed, contract, _)| {
            seed == expected_seed && contract == expected_contract
        });
        anyhow::ensure!(
            reproduced,
            "expected first failure `{expected_contract}` did not reproduce for seed {expected_seed}"
        );
        println!(
            "Expected Behavior CI failure reproduced: seed={expected_seed} contract={expected_contract}"
        );
        return Ok(());
    }
    anyhow::ensure!(report.passed(), "one or more behavior contracts failed");
    Ok(())
}

fn behavior_replay(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let replay_argument = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("behavior-replay requires a .rne-replay path"))?;
    anyhow::ensure!(
        args.next().is_none(),
        "behavior-replay accepts exactly one .rne-replay path"
    );
    let replay_path = root.join(replay_argument);
    let replay = rne_ai::BehaviorReplayArtifact::read_json(&replay_path)?;
    let verification = rne_ai::verify_behavior_replay(&replay, |seed, dimensions| {
        g1_behavior_from_dimensions(&replay.scenario, seed, dimensions)
    })?;
    println!(
        "Behavior replay reproduced: seed={} step={} contract={} state_digest={:#018x} matched_frames={}\nReplay: {}",
        verification.seed,
        verification.step,
        verification.contract,
        verification.state_digest,
        verification.matched_frames,
        replay_path.display()
    );
    Ok(())
}

/// Reproduces the v0.7 shared-aisle flagship success, minimized failure, and capsule.
fn flagship(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut cross_backend = false;
    for argument in args {
        match argument.as_str() {
            "--cross-backend" => cross_backend = true,
            other => anyhow::bail!("unknown flagship argument: {other}"),
        }
    }
    let root = workspace_root()?;
    let artifacts = root.join("artifacts");
    fs::create_dir_all(&artifacts)?;
    let artifacts_metadata = fs::symlink_metadata(&artifacts)?;
    anyhow::ensure!(
        artifacts_metadata.is_dir() && !artifacts_metadata.file_type().is_symlink(),
        "workspace artifacts path must be a real directory"
    );
    let artifacts = artifacts.canonicalize()?;
    let output = artifacts.join("flagship-validation");
    if output.exists() {
        let metadata = fs::symlink_metadata(&output)?;
        anyhow::ensure!(
            metadata.is_dir() && !metadata.file_type().is_symlink(),
            "refusing to replace non-directory or symlinked flagship output {}",
            output.display()
        );
        let resolved = output.canonicalize()?;
        anyhow::ensure!(
            resolved.parent() == Some(artifacts.as_path()),
            "refusing to remove flagship output outside {}",
            artifacts.display()
        );
        fs::remove_dir_all(&resolved)?;
    }

    let workflow_command = if cross_backend {
        "cargo run --locked -p flagship_validation_workflow --features mujoco --example 74_flagship_validation_workflow -- artifacts/flagship-validation --cross-backend"
    } else {
        "cargo run --locked -p flagship_validation_workflow --example 74_flagship_validation_workflow -- artifacts/flagship-validation"
    };
    run_step(workflow_command)?;
    let replay = output.join("failure-minimized.rne-replay");
    let report = output.join("workflow-report.json");
    let success = output.join("success.behavior-report.json");
    let failure = output.join("failure.behavior-report.json");
    let inspector = output.join("replay-inspector.html");
    let task_spec = output.join("flagship.task.json");
    let cross_backend_report = output.join("cross-backend-report.json");
    let mujoco_success = output.join("mujoco-success.behavior-report.json");
    let capsule = output.join("failure-capsule");
    let mut create_args = vec![
        "create".to_string(),
        "--replay".to_string(),
        replay.display().to_string(),
        "--evidence".to_string(),
        report.display().to_string(),
        "--evidence".to_string(),
        success.display().to_string(),
        "--evidence".to_string(),
        failure.display().to_string(),
        "--evidence".to_string(),
        inspector.display().to_string(),
        "--evidence".to_string(),
        task_spec.display().to_string(),
    ];
    if cross_backend {
        create_args.extend([
            "--evidence".to_string(),
            cross_backend_report.display().to_string(),
            "--evidence".to_string(),
            mujoco_success.display().to_string(),
        ]);
    }
    create_args.extend([
        "--output".to_string(),
        capsule.display().to_string(),
        "--backend".to_string(),
        "rapier-native".to_string(),
        "--backend-version".to_string(),
        "0.22".to_string(),
    ]);
    let mut create_args = create_args.into_iter();
    failure_capsule::run(&mut create_args)?;
    let mut verify_args = vec!["verify".to_string(), capsule.display().to_string()].into_iter();
    failure_capsule::run(&mut verify_args)?;

    let report: serde_json::Value = serde_json::from_slice(&fs::read(&report)?)?;
    anyhow::ensure!(
        report.get("kind").and_then(serde_json::Value::as_str)
            == Some(FLAGSHIP_WORKFLOW_REPORT_KIND)
            && report
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(u64::from(FLAGSHIP_WORKFLOW_REPORT_SCHEMA_VERSION)),
        "flagship report kind/schema mismatch"
    );
    anyhow::ensure!(
        report
            .pointer("/success/status")
            .and_then(serde_json::Value::as_str)
            == Some("passed"),
        "flagship report does not contain a passing success run"
    );
    anyhow::ensure!(
        report
            .pointer("/intentional_failure/expected_contract")
            .and_then(serde_json::Value::as_str)
            == Some("perception_stream_alive")
            && report
                .pointer("/intentional_failure/active_dimensions_before")
                .and_then(serde_json::Value::as_u64)
                == Some(3)
            && report
                .pointer("/intentional_failure/active_dimensions_after")
                .and_then(serde_json::Value::as_u64)
                == Some(1),
        "flagship failure was not minimized from three dimensions to the blackout"
    );
    if cross_backend {
        anyhow::ensure!(
            report
                .get("physics_execution_paths")
                .and_then(serde_json::Value::as_array)
                .is_some_and(|paths| {
                    paths
                        == &[
                            serde_json::Value::String("rapier_native".to_string()),
                            serde_json::Value::String("mujoco_native".to_string()),
                        ]
                })
                && report
                    .get("cross_backend_report")
                    .and_then(serde_json::Value::as_str)
                    == Some("cross-backend-report.json"),
            "flagship workflow report does not register both production physics paths"
        );
        let cross_report: serde_json::Value =
            serde_json::from_slice(&fs::read(&cross_backend_report)?)?;
        anyhow::ensure!(
            cross_report.get("kind").and_then(serde_json::Value::as_str)
                == Some(FLAGSHIP_CROSS_BACKEND_REPORT_KIND)
                && cross_report
                    .get("schema_version")
                    .and_then(serde_json::Value::as_u64)
                    == Some(u64::from(FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION))
                && cross_report
                    .get("status")
                    .and_then(serde_json::Value::as_str)
                    == Some("passed")
                && cross_report
                    .get("task_id")
                    .and_then(serde_json::Value::as_str)
                    == Some("rne.flagship.mobile_lift_shared_aisle.v1"),
            "flagship cross-backend report kind/schema/task/status mismatch"
        );
        let backends = cross_report
            .get("backends")
            .and_then(serde_json::Value::as_array)
            .context("flagship cross-backend report omitted backends")?;
        anyhow::ensure!(
            backends.len() == 2
                && backends.iter().all(|backend| {
                    backend.get("status").and_then(serde_json::Value::as_str) == Some("passed")
                }),
            "flagship cross-backend report did not pass both backends"
        );
        let tolerance_checks = cross_report
            .get("tolerance_checks")
            .and_then(serde_json::Value::as_array)
            .context("flagship cross-backend report omitted tolerance checks")?;
        anyhow::ensure!(
            !tolerance_checks.is_empty()
                && tolerance_checks.iter().all(|check| {
                    check.get("status").and_then(serde_json::Value::as_str) == Some("passed")
                        && check
                            .get("unit")
                            .and_then(serde_json::Value::as_str)
                            .is_some()
                        && check
                            .get("maximum_delta")
                            .and_then(serde_json::Value::as_f64)
                            .is_some_and(|value| value > 0.0)
                }),
            "flagship cross-backend tolerance registry is incomplete or failed"
        );
    }
    let inspector_text = fs::read_to_string(&inspector)?;
    anyhow::ensure!(
        inspector_text.contains("id=\"replay-data\"")
            && inspector_text.contains("minimized failure"),
        "flagship browser inspector is incomplete"
    );
    println!(
        "v0.7 flagship evidence verified (cross_backend={}): {}",
        cross_backend,
        output.display()
    );
    Ok(())
}

fn g1_behavior_from_dimensions(
    scenario: &str,
    seed: u64,
    dimensions: &[rne_ai::BehaviorDimension],
) -> Result<rne_ai::UnitreeG1Dex3BehaviorScenario, String> {
    if !matches!(
        scenario,
        "unitree_g1_dex3_acquire" | "unitree_g1_dex3_invalid_tray"
    ) {
        return Err(format!("unsupported Behavior CI scenario `{scenario}`"));
    }
    rne_ai::UnitreeG1Dex3BehaviorScenario::from_dimensions(
        seed,
        rne_ai::UnitreeG1Dex3BehaviorConfig::default(),
        dimensions,
    )
    .map_err(|error| error.to_string())
}

fn report_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

fn first_behavior_failure(
    report: &rne_ai::BehaviorReport,
) -> Option<(u64, &str, &rne_ai::BehaviorViolation)> {
    report.seeds.iter().find_map(|seed| {
        seed.contracts
            .iter()
            .enumerate()
            .filter_map(|(index, contract)| {
                contract
                    .violation
                    .as_ref()
                    .map(|violation| (index, contract.name.as_str(), violation))
            })
            .min_by_key(|(index, _, violation)| (violation.step, *index))
            .map(|(_, contract, violation)| (seed.seed, contract, violation))
    })
}

fn parse_seed_range(value: &str) -> anyhow::Result<Vec<u64>> {
    let (start, end) = value
        .split_once("..")
        .ok_or_else(|| anyhow::anyhow!("seed range must use START..END"))?;
    let start = start.parse::<u64>()?;
    let end = end.parse::<u64>()?;
    anyhow::ensure!(start < end, "seed range must be non-empty and ascending");
    anyhow::ensure!(
        end - start <= 10_000,
        "seed range may contain at most 10000 seeds"
    );
    Ok((start..end).collect())
}

fn default_behavior_seeds() -> Vec<u64> {
    parse_seed_range(DEFAULT_BEHAVIOR_SEED_RANGE).expect("default behavior seed range is valid")
}

/// The full local gate: every stage in sequence.
///
/// CI runs lint, sharded tests, four smoke partitions, RL, headless, parity, and
/// behavior CI as parallel jobs, so wall-clock time there is the slowest stage
/// instead of the sum. Keep stage contents in the stage functions so the local
/// and CI gates cannot drift apart.
fn ci() -> anyhow::Result<()> {
    ci_lint()?;
    ci_test()?;
    ci_smoke(None)?;
    if std::env::var("RNE_SKIP_RL_SMOKES").is_ok() {
        eprintln!("skipping mobile_manipulator_rl_smokes (RNE_SKIP_RL_SMOKES is set)");
    } else {
        ci_rl()?;
    }
    ci_headless()?;
    parity(&mut std::iter::empty::<String>())?;
    fuzz_smoke(&mut std::iter::empty::<String>())?;
    behavior_ci(&mut std::iter::empty::<String>())
}

/// Formatting, dependency boundaries, and Clippy.
fn ci_lint() -> anyhow::Result<()> {
    run_step("cargo fmt --all -- --check")?;
    lint_boundaries()?;
    run_step("cargo clippy --locked --workspace --all-targets -- -D warnings")
}

/// Workspace tests, optional Pinocchio goldens, and asset validation.
fn ci_test() -> anyhow::Result<()> {
    ci_test_partition(None)
}

/// Workspace tests, optionally as one deterministic nextest partition.
///
/// `partition` is `PART/TOTAL`, forwarded to `cargo nextest run --partition
/// hash:PART/TOTAL` so CI can shard the long-running episode suites across parallel
/// jobs. Without a partition (the local `ci` path) plain `cargo test` runs, so
/// contributors do not need nextest installed. The repository has no doctests, so
/// nextest's lack of doctest support loses no coverage; the asset checks run only on
/// the first partition to avoid duplicating them per shard.
fn ci_test_partition(partition: Option<&str>) -> anyhow::Result<()> {
    match partition {
        None => run_step("cargo test --locked --workspace")?,
        Some(spec) => {
            anyhow::ensure!(
                spec.split('/').count() == 2
                    && spec.split('/').all(|part| part.parse::<u32>().is_ok()),
                "partition must be PART/TOTAL, got {spec}"
            );
            run_step(&format!(
                "cargo nextest run --locked --workspace --partition hash:{spec}"
            ))?;
        }
    }
    if partition.is_none_or(|spec| spec.starts_with("1/")) {
        pinocchio_golden_optional()?;
        validate_repo_assets()?;
    }
    Ok(())
}

/// Example smokes and media checks.
fn ci_smoke(partition: Option<&str>) -> anyhow::Result<()> {
    match parse_smoke_partition(partition)? {
        SmokePartition::All => {
            run_example_smokes()?;
            house_gif_demo()?;
            hero_media_check()
        }
        SmokePartition::Manipulator => run_manipulator_smokes(),
        SmokePartition::Locomotion => run_locomotion_smokes(),
        SmokePartition::Assets => run_asset_smokes(),
        SmokePartition::Media => {
            house_gif_demo()?;
            hero_media_check()
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SmokePartition {
    All,
    Manipulator,
    Locomotion,
    Assets,
    Media,
}

fn parse_smoke_partition(partition: Option<&str>) -> anyhow::Result<SmokePartition> {
    match partition {
        None => Ok(SmokePartition::All),
        Some("manipulator") => Ok(SmokePartition::Manipulator),
        Some("locomotion") => Ok(SmokePartition::Locomotion),
        Some("assets") => Ok(SmokePartition::Assets),
        Some("media") => Ok(SmokePartition::Media),
        Some(other) => anyhow::bail!(
            "unknown ci-smoke partition {other:?}; expected manipulator, locomotion, assets, or media"
        ),
    }
}

/// Runs the explicit CPU-only headless renderer and sensor test gates.
fn ci_headless() -> anyhow::Result<()> {
    accelerator::validate_contract(&workspace_root()?)?;
    run_step("cargo test --locked -p rne_render --lib")?;
    run_step("cargo test --locked -p rne_sensor --lib")?;
    run_step("cargo test --locked -p rne_hardware_gateway")?;
    run_step("cargo test --locked -p rne_hardware_lekiwi")?;
    dataset::dataset_reference_smoke()?;
    flagship(&mut std::iter::empty::<String>())
}

/// Python RL smokes, including the maturin build of `rne_py`.
fn ci_rl() -> anyhow::Result<()> {
    mobile_manipulator_rl_smokes()
}

fn run_example_smokes() -> anyhow::Result<()> {
    run_manipulator_smokes()?;
    run_locomotion_smokes()?;
    run_asset_smokes()
}

fn run_manipulator_smokes() -> anyhow::Result<()> {
    run_step("cargo run --locked -p mobile_manipulator_arm --example 20_mobile_manipulator_arm -- --smoke")?;
    run_step(
        "cargo run --locked -p mobile_manipulator_reach --example 21_mobile_manipulator_reach -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_grasp --example 22_mobile_manipulator_grasp -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_transport --example 23_mobile_manipulator_transport -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_wrist_cam --example 24_mobile_manipulator_wrist_cam -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_episode --example 25_mobile_manipulator_episode -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_place --example 26_mobile_manipulator_place -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_vectorized --example 28_mobile_manipulator_vectorized -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_curriculum --example 29_mobile_manipulator_curriculum -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_lift --example 30_mobile_manipulator_lift -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p mobile_manipulator_lift_pick_place --example 31_mobile_manipulator_lift_pick_place -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p lift_pick_place_hero --example 32_lift_pick_place_hero -- --smoke",
    )?;
    run_step("cargo run --locked -p clutter_pick_place_e2e --example 33_clutter_pick_place_e2e -- --smoke")?;
    run_step(
        "cargo run --locked -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator",
    )?;
    run_step(
        "cargo run --locked -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator-mobile",
    )?;
    run_step(
        "cargo run --locked -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator-lift",
    )
}

fn run_locomotion_smokes() -> anyhow::Result<()> {
    run_step("cargo run --locked -p go2_pure_torque --example 64_go2_pure_torque -- --smoke")?;
    run_step(
        "cargo run --locked -p go2_velocity_terrain --example 65_go2_velocity_terrain -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p locomotion_vectorized --example 66_locomotion_vectorized -- --smoke",
    )?;
    run_step(
        "cargo run --locked -p g1_commanded_locomotion --example 67_g1_commanded_locomotion -- --smoke",
    )?;
    run_step("cargo run --locked -p g1_heading_turn --example 68_g1_heading_turn -- --smoke")?;
    run_step(
        "cargo run --locked -p g1_heading_turn --example 68_g1_heading_turn -- --train --smoke",
    )
}

fn run_asset_smokes() -> anyhow::Result<()> {
    run_step("cargo run --locked -p gltf_humanoid_gpu --example 69_gltf_humanoid_gpu -- --smoke")?;
    run_step(
        "cargo run --locked -p g1_photoreal_capture --example 70_g1_photoreal_capture -- --smoke",
    )?;
    run_step("cargo run --locked -p g1_rgbd_sensor --example 71_g1_rgbd_sensor -- --smoke")?;
    run_step("cargo run --locked -p g1_stride_gif --example 63_g1_stride_gif -- --smoke")
}

fn house_gif_demo() -> anyhow::Result<()> {
    let python = python_command()?;
    run_step(&format!(
        "{python} examples/27_mobile_manipulator_rl/house_gif_demo.py --check"
    ))?;
    Ok(())
}

fn hero_media_check() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let readme_path = root.join("README.md");
    let gif_path = root.join("docs/media/rne-hero.gif");
    let png_path = root.join("docs/media/rne-hero.png");
    let metadata_path = root.join("docs/media/rne-hero.json");
    let dex3_gif_path = root.join("docs/media/unitree-g1-dex3.gif");
    let dex3_png_path = root.join("docs/media/unitree-g1-dex3.png");
    let cloth_gif_path = root.join("docs/media/unitree-g1-cloth.gif");
    let cloth_png_path = root.join("docs/media/unitree-g1-cloth.png");
    let learned_g1_gif_path = root.join("docs/media/unitree-g1-learned-stride.gif");
    let learned_g1_png_path = root.join("docs/media/unitree-g1-learned-stride.png");
    let readme = fs::read_to_string(&readme_path)?;
    anyhow::ensure!(
        readme.contains("srcset=\"docs/media/rne-hero.png\""),
        "README hero reduced-motion poster does not point at docs/media/rne-hero.png"
    );
    anyhow::ensure!(
        readme.contains("<img src=\"docs/media/rne-hero.gif\""),
        "README first hero image does not point at docs/media/rne-hero.gif"
    );
    anyhow::ensure!(
        readme.contains(
            "3D RNE mobile manipulator simulation navigating a house-like room while carrying a task object"
        ),
        "README hero alt text does not describe the 3D mobile manipulator simulation"
    );
    anyhow::ensure!(
        readme.contains("Real capture:")
            && readme.contains("docs/media/rne-hero.json")
            && readme.contains("docs/media/generate-hero.sh"),
        "README hero caption does not link the 3D generator and metadata"
    );

    let gif = fs::read(&gif_path)?;
    anyhow::ensure!(gif.starts_with(b"GIF8"), "README hero GIF header mismatch");
    anyhow::ensure!(gif.ends_with(b";"), "README hero GIF trailer missing");
    anyhow::ensure!(
        gif.len() > 100_000,
        "README hero GIF is unexpectedly small: {} bytes",
        gif.len()
    );
    anyhow::ensure!(png_path.is_file(), "README hero PNG is missing");
    anyhow::ensure!(
        readme.contains("srcset=\"docs/media/unitree-g1-dex3.png\"")
            && readme.contains("<img src=\"docs/media/unitree-g1-dex3.gif\""),
        "README G1 Dex3 media references are missing"
    );
    let dex3_gif = fs::read(&dex3_gif_path)?;
    anyhow::ensure!(
        dex3_gif.starts_with(b"GIF8") && dex3_gif.ends_with(b";") && dex3_gif.len() > 100_000,
        "README G1 Dex3 GIF is missing or malformed"
    );
    anyhow::ensure!(dex3_png_path.is_file(), "README G1 Dex3 PNG is missing");
    anyhow::ensure!(
        readme.contains("srcset=\"docs/media/unitree-g1-cloth.png\"")
            && readme.contains("<img src=\"docs/media/unitree-g1-cloth.gif\""),
        "README G1 cloth media references are missing"
    );
    let cloth_gif = fs::read(&cloth_gif_path)?;
    anyhow::ensure!(
        cloth_gif.starts_with(b"GIF8") && cloth_gif.ends_with(b";") && cloth_gif.len() > 100_000,
        "README G1 cloth GIF is missing or malformed"
    );
    anyhow::ensure!(cloth_png_path.is_file(), "README G1 cloth PNG is missing");
    anyhow::ensure!(
        readme.contains("srcset=\"docs/media/unitree-g1-learned-stride.png\"")
            && readme.contains("<img src=\"docs/media/unitree-g1-learned-stride.gif\""),
        "README learned G1 stride media references are missing"
    );
    let learned_g1_gif = fs::read(&learned_g1_gif_path)?;
    anyhow::ensure!(
        learned_g1_gif.starts_with(b"GIF8")
            && learned_g1_gif.ends_with(b";")
            && learned_g1_gif.len() > 100_000,
        "README learned G1 stride GIF is missing or malformed"
    );
    anyhow::ensure!(
        learned_g1_png_path.is_file(),
        "README learned G1 stride PNG is missing"
    );
    let metadata: serde_json::Value = serde_json::from_str(&fs::read_to_string(&metadata_path)?)?;
    anyhow::ensure!(
        metadata["artifact"].as_str() == Some("rne_3d_mobile_manipulator_pick_place_hero"),
        "README hero metadata does not describe the 3D pick/place hero"
    );
    anyhow::ensure!(
        metadata["schema_version"].as_u64() == Some(2),
        "README hero metadata must use schema_version 2"
    );
    let encode = metadata
        .get("encode")
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing encode block"))?;
    let gif_progression = inspect_gif_frame_progression(
        &gif_path,
        usize::try_from(encode["animation_frames"].as_u64().ok_or_else(|| {
            anyhow::anyhow!("README hero encode block missing animation_frames")
        })?)?,
        usize::try_from(
            encode["hold_frames"]
                .as_u64()
                .ok_or_else(|| anyhow::anyhow!("README hero encode block missing hold_frames"))?,
        )?,
    )?;
    anyhow::ensure!(
        encode["fps"].as_f64() == Some(15.0)
            && encode["animation_frames"].as_u64() == Some(100)
            && encode["hold_frames"].as_u64() == Some(10)
            && encode["max_colors"].as_u64() == Some(192)
            && encode["scale_width"].as_u64() == Some(960),
        "README hero encode block does not match the expected hero pipeline"
    );
    let max_byte_size = encode["max_byte_size"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero encode block missing max_byte_size"))?;
    anyhow::ensure!(
        u64::try_from(gif.len())? <= max_byte_size,
        "README hero GIF exceeds encode.max_byte_size: {} > {max_byte_size}",
        gif.len()
    );
    let metadata_width = metadata["width"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing width"))?;
    let metadata_height = metadata["height"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing height"))?;
    let metadata_frame_count = metadata["frame_count"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing frame_count"))?;
    anyhow::ensure!(
        gif_progression.width == u32::try_from(metadata_width)?,
        "README hero metadata width does not match GIF: metadata={metadata_width}, gif={}",
        gif_progression.width
    );
    anyhow::ensure!(
        gif_progression.height == u32::try_from(metadata_height)?,
        "README hero metadata height does not match GIF: metadata={metadata_height}, gif={}",
        gif_progression.height
    );
    anyhow::ensure!(
        u64::try_from(gif_progression.frame_count)? == metadata_frame_count,
        "README hero metadata frame_count does not match GIF: metadata={metadata_frame_count}, gif={}",
        gif_progression.frame_count
    );
    anyhow::ensure!(
        metadata["source"]["kind"].as_str() == Some("wgpu_simulation")
            && metadata["source"]["generator"].as_str() == Some("examples/32_lift_pick_place_hero")
            && metadata["source"]["scene"].as_str()
                == Some("assets/scenes/mm_mobile_hero.rne.scene.toml")
            && metadata["source"]["policy"].as_str() == Some("MobilePickPlaceHeroPolicy")
            && metadata["source"]["physics"].as_str() == Some("MobileManipulatorSim/Rapier"),
        "README hero metadata source is not wgpu_simulation"
    );
    let overlays = metadata["overlays"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata overlays must be an array"))?;
    anyhow::ensure!(
        overlays
            .iter()
            .any(|overlay| overlay.as_str() == Some("house_context"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("base_path"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("object_path"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("pickup_surface"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("task_object"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("drop_tray"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("drop_zone")),
        "README hero metadata is missing expected 3D overlays"
    );
    let base_travel_m = metadata["simulation"]["base_travel_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing base_travel_m"))?;
    let ee_travel_m = metadata["simulation"]["ee_travel_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing ee_travel_m"))?;
    let final_ee_target_error_m = metadata["simulation"]["final_ee_target_error_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing final_ee_target_error_m"))?;
    let object_transport_m = metadata["simulation"]["object_transport_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing object_transport_m"))?;
    let min_object_transport_m = metadata["simulation"]["min_object_transport_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing min_object_transport_m"))?;
    let final_object_place_error_m = metadata["simulation"]["final_object_place_error_m"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing final_object_place_error_m")
        })?;
    let max_final_object_place_error_m = metadata["simulation"]["max_final_object_place_error_m"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing max_final_object_place_error_m")
        })?;
    let grasped_steps = metadata["simulation"]["grasped_steps"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing grasped_steps"))?;
    let released_after_grasp = metadata["simulation"]["released_after_grasp"]
        .as_bool()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing released_after_grasp"))?;
    let max_final_ee_target_error_m = metadata["simulation"]["max_final_ee_target_error_m"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing max_final_ee_target_error_m")
        })?;
    let min_consecutive_frame_delta_ratio = metadata["simulation"]
        ["min_consecutive_frame_delta_ratio"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing min_consecutive_frame_delta_ratio")
        })?;
    let first_last_frame_delta_ratio = metadata["simulation"]["first_last_frame_delta_ratio"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing first_last_frame_delta_ratio")
        })?;
    let min_consecutive_frame_delta_ratio_threshold = metadata["simulation"]
        ["min_consecutive_frame_delta_ratio_threshold"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!(
                "README hero metadata missing min_consecutive_frame_delta_ratio_threshold"
            )
        })?;
    let min_first_last_frame_delta_ratio_threshold = metadata["simulation"]
        ["min_first_last_frame_delta_ratio_threshold"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!(
                "README hero metadata missing min_first_last_frame_delta_ratio_threshold"
            )
        })?;
    let max_hold_frame_delta_ratio = metadata["simulation"]["max_hold_frame_delta_ratio"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing max_hold_frame_delta_ratio")
        })?;
    let max_hold_frame_delta_ratio_threshold = metadata["simulation"]
        ["max_hold_frame_delta_ratio_threshold"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing max_hold_frame_delta_ratio_threshold")
        })?;
    let max_base_height_error_m = metadata["simulation"]["max_base_height_error_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing max_base_height_error_m"))?;
    let min_base_yaw_only_dot = metadata["simulation"]["min_base_yaw_only_dot"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing min_base_yaw_only_dot"))?;
    let trajectory_digest = metadata["simulation"]["trajectory_digest"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing trajectory_digest"))?;
    anyhow::ensure!(
        base_travel_m > 0.20,
        "README hero simulation base travel is too small: {base_travel_m:.2} m"
    );
    anyhow::ensure!(
        ee_travel_m > 0.15,
        "README hero simulation end-effector travel is too small: {ee_travel_m:.2} m"
    );
    anyhow::ensure!(
        max_final_ee_target_error_m <= 0.05,
        "README hero reach target threshold is too loose: {max_final_ee_target_error_m:.3} m"
    );
    anyhow::ensure!(
        final_ee_target_error_m <= max_final_ee_target_error_m,
        "README hero manipulator does not reach the target: final_ee_target_error={final_ee_target_error_m:.3} m"
    );
    anyhow::ensure!(
        min_object_transport_m >= 0.35,
        "README hero object transport threshold is too loose: {min_object_transport_m:.2} m"
    );
    anyhow::ensure!(
        object_transport_m >= min_object_transport_m,
        "README hero object transport is too small: object_transport={object_transport_m:.2} m"
    );
    anyhow::ensure!(
        max_final_object_place_error_m <= 0.20,
        "README hero object place threshold is too loose: {max_final_object_place_error_m:.3} m"
    );
    anyhow::ensure!(
        final_object_place_error_m <= max_final_object_place_error_m,
        "README hero object is not near the drop zone: final_object_place_error={final_object_place_error_m:.3} m"
    );
    anyhow::ensure!(
        grasped_steps >= 12 && released_after_grasp,
        "README hero object was not carried then released: grasped_steps={grasped_steps}, released_after_grasp={released_after_grasp}"
    );
    anyhow::ensure!(
        min_consecutive_frame_delta_ratio_threshold >= 0.0025,
        "README hero frame-delta threshold is too loose: {min_consecutive_frame_delta_ratio_threshold:.4}"
    );
    anyhow::ensure!(
        min_first_last_frame_delta_ratio_threshold >= 0.08,
        "README hero first/last frame-delta threshold is too loose: {min_first_last_frame_delta_ratio_threshold:.4}"
    );
    anyhow::ensure!(
        min_consecutive_frame_delta_ratio >= min_consecutive_frame_delta_ratio_threshold,
        "README hero GIF has nearly frozen adjacent frames: min_consecutive_frame_delta_ratio={min_consecutive_frame_delta_ratio:.4}"
    );
    anyhow::ensure!(
        first_last_frame_delta_ratio >= min_first_last_frame_delta_ratio_threshold,
        "README hero GIF lacks visible progression: first_last_frame_delta_ratio={first_last_frame_delta_ratio:.4}"
    );
    anyhow::ensure!(
        gif_progression.min_consecutive_frame_delta_ratio
            >= min_consecutive_frame_delta_ratio_threshold,
        "README hero GIF bytes have nearly frozen adjacent frames: min_consecutive_frame_delta_ratio={:.4}",
        gif_progression.min_consecutive_frame_delta_ratio
    );
    anyhow::ensure!(
        gif_progression.first_last_frame_delta_ratio >= min_first_last_frame_delta_ratio_threshold,
        "README hero GIF bytes lack visible progression: first_last_frame_delta_ratio={:.4}",
        gif_progression.first_last_frame_delta_ratio
    );
    anyhow::ensure!(
        max_hold_frame_delta_ratio <= max_hold_frame_delta_ratio_threshold,
        "README hero hold seam is not calm enough: max_hold_frame_delta_ratio={max_hold_frame_delta_ratio:.4}"
    );
    anyhow::ensure!(
        max_base_height_error_m <= 0.01,
        "README hero mobile base leaves the ground plane: max_base_height_error={max_base_height_error_m:.4} m"
    );
    anyhow::ensure!(
        min_base_yaw_only_dot >= 0.999_999,
        "README hero mobile base is not upright: min_base_yaw_only_dot={min_base_yaw_only_dot:.9}"
    );
    anyhow::ensure!(
        trajectory_digest.len() == 18
            && trajectory_digest.starts_with("0x")
            && trajectory_digest[2..]
                .chars()
                .all(|character| character.is_ascii_hexdigit()),
        "README hero trajectory_digest must be a 64-bit hex string"
    );
    // The recorded digest is produced on Windows (docs/media/generate-hero.sh);
    // the hero smoke passes everywhere, but arm/payload contact dynamics are not
    // bit-identical across platforms (outcome-stable, not bitwise-stable: even
    // with the mm_minimal settle physics fixed, contact impulse ordering and libm
    // differences shift trajectories by millimetres), so only compare the
    // bit-exact live digest on the generating platform.
    if cfg!(target_os = "linux") {
        eprintln!(
            "skipping README hero live digest comparison on linux (digest is bit-exact and recorded on Windows)"
        );
    } else {
        let live_trajectory_digest = hero_simulation_smoke_digest()?;
        anyhow::ensure!(
            live_trajectory_digest == trajectory_digest,
            "README hero trajectory digest is stale: metadata={trajectory_digest}, live={live_trajectory_digest}"
        );
    }
    anyhow::ensure!(
        metadata["simulation"]["final_base_m"]
            .as_array()
            .is_some_and(|items| items.len() == 3)
            && metadata["simulation"]["final_ee_m"]
                .as_array()
                .is_some_and(|items| items.len() == 3)
            && metadata["simulation"]["final_object_m"]
                .as_array()
                .is_some_and(|items| items.len() == 3),
        "README hero metadata final simulation positions must be 3D vectors"
    );
    anyhow::ensure!(
        metadata["byte_size"].as_u64() == Some(u64::try_from(gif.len())?),
        "README hero metadata byte_size does not match GIF bytes"
    );
    println!(
        "README 3D hero media ok: gif={} bytes metadata={}",
        gif.len(),
        metadata_path.display()
    );
    Ok(())
}

fn hero_simulation_smoke_digest() -> anyhow::Result<String> {
    let command =
        "cargo run --locked -p lift_pick_place_hero --example 32_lift_pick_place_hero -- --smoke";
    let output = run_step_output(command)?;
    extract_hero_digest(&output)
        .ok_or_else(|| anyhow::anyhow!("hero smoke output did not include trajectory digest"))
}

fn hero_contact_sheet() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let gif_path = root.join("docs/media/rne-hero.gif");
    anyhow::ensure!(
        gif_path.is_file(),
        "README hero GIF is missing at {}",
        gif_path.display()
    );

    let output_dir = root.join("target/hero-debug");
    fs::create_dir_all(&output_dir)?;
    let output_path = output_dir.join("contact.png");

    let filter = hero_contact_sheet_filter();
    let status = Command::new("ffmpeg")
        .arg("-y")
        .arg("-v")
        .arg("error")
        .arg("-i")
        .arg(&gif_path)
        .arg("-vf")
        .arg(filter)
        .arg(&output_path)
        .status()
        .map_err(|error| anyhow::anyhow!("ffmpeg is required for hero-contact-sheet: {error}"))?;

    if !status.success() {
        anyhow::bail!("ffmpeg failed while generating {}", output_path.display());
    }

    println!(
        "wrote README hero contact sheet to {}",
        output_path.display()
    );
    Ok(())
}

fn hero_contact_sheet_filter() -> String {
    let select_frames = HERO_CONTACT_SHEET_FRAMES
        .iter()
        .map(|frame| format!("eq(n,{frame})"))
        .collect::<Vec<_>>()
        .join("+");
    format!("select='{select_frames}',scale=320:-1,tile=3x3")
}

#[derive(Clone, Copy, Debug)]
struct GifFrameProgression {
    width: u32,
    height: u32,
    frame_count: usize,
    min_consecutive_frame_delta_ratio: f64,
    first_last_frame_delta_ratio: f64,
}

fn inspect_gif_frame_progression(
    path: &Path,
    animation_frame_count: usize,
    hold_frame_count: usize,
) -> anyhow::Result<GifFrameProgression> {
    let file = fs::File::open(path)?;
    let decoder = image::codecs::gif::GifDecoder::new(BufReader::new(file))?;
    let frames = decoder.into_frames();

    let mut width = 0;
    let mut height = 0;
    let mut frame_count = 0usize;
    let mut first_frame_rgba8 = Vec::new();
    let mut previous_frame_rgba8 = Vec::new();
    let mut min_consecutive_frame_delta_ratio = 1.0_f64;
    let mut first_last_frame_delta_ratio = 0.0_f64;
    let animation_pairs_end = animation_frame_count.saturating_sub(1);

    for frame in frames {
        let frame = frame?;
        let buffer = frame.into_buffer();
        let (frame_width, frame_height) = buffer.dimensions();
        let rgba8 = buffer.into_raw();

        if frame_count == 0 {
            width = frame_width;
            height = frame_height;
            first_frame_rgba8.clone_from(&rgba8);
        } else {
            anyhow::ensure!(
                frame_width == width && frame_height == height,
                "README hero GIF frame dimensions changed at frame {frame_count}: expected {}x{}, got {}x{}",
                width,
                height,
                frame_width,
                frame_height
            );
            if frame_count < animation_pairs_end + 1 {
                let delta_ratio = frame_delta_ratio(&previous_frame_rgba8, &rgba8)?;
                min_consecutive_frame_delta_ratio =
                    min_consecutive_frame_delta_ratio.min(delta_ratio);
            }
            first_last_frame_delta_ratio = frame_delta_ratio(&first_frame_rgba8, &rgba8)?;
        }

        previous_frame_rgba8 = rgba8;
        frame_count += 1;
    }

    anyhow::ensure!(frame_count > 0, "README hero GIF has no decoded frames");
    anyhow::ensure!(
        animation_frame_count + hold_frame_count == frame_count,
        "README hero GIF frame count mismatch: expected {} animation + {} hold = {}, got {}",
        animation_frame_count,
        hold_frame_count,
        animation_frame_count + hold_frame_count,
        frame_count
    );
    if animation_frame_count <= 1 {
        min_consecutive_frame_delta_ratio = 0.0;
    }

    Ok(GifFrameProgression {
        width,
        height,
        frame_count,
        min_consecutive_frame_delta_ratio,
        first_last_frame_delta_ratio,
    })
}

fn frame_delta_ratio(previous_rgba8: &[u8], current_rgba8: &[u8]) -> anyhow::Result<f64> {
    anyhow::ensure!(
        previous_rgba8.len() == current_rgba8.len(),
        "hero frame buffers must have identical byte lengths"
    );
    anyhow::ensure!(
        previous_rgba8.len().is_multiple_of(4),
        "hero frame buffer length must be RGBA8-aligned"
    );
    let pixel_count = previous_rgba8.len() / 4;
    if pixel_count == 0 {
        return Ok(0.0);
    }
    let changed_pixels = previous_rgba8
        .chunks_exact(4)
        .zip(current_rgba8.chunks_exact(4))
        .filter(|(previous, current)| previous != current)
        .count();
    Ok(changed_pixels as f64 / pixel_count as f64)
}

fn extract_hero_digest(output: &str) -> Option<String> {
    let marker = "digest=";
    let start = output.find(marker)? + marker.len();
    let digest = output[start..]
        .split(|character: char| !character.is_ascii_hexdigit() && character != 'x')
        .next()?;
    if digest.len() == 18
        && digest.starts_with("0x")
        && digest[2..]
            .chars()
            .all(|character| character.is_ascii_hexdigit())
    {
        Some(digest.to_string())
    } else {
        None
    }
}

fn python_command() -> anyhow::Result<&'static str> {
    for candidate in ["python", "python3"] {
        if let Ok(status) = Command::new(candidate)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
        {
            if status.success() {
                return Ok(candidate);
            }
        }
    }
    anyhow::bail!("python or python3 is required for house-gif-demo")
}

fn validate_repo_assets() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let scenes = [
        root.join("assets/scenes/episode_diff_drive.rne.scene.toml"),
        root.join("assets/scenes/mm_mobile.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_grasp.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_transport.rne.scene.toml"),
        root.join("assets/scenes/mm_lift.rne.scene.toml"),
        root.join("assets/scenes/mm_lift_pick.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_clutter.rne.scene.toml"),
        root.join("assets/scenes/mm_mobile_clutter.rne.scene.toml"),
        root.join("assets/scenes/mm_mobile_hero.rne.scene.toml"),
        root.join("assets/scenes/unitree_g1_dex3_pick_place.rne.scene.toml"),
        root.join("assets/scenes/deformable_cable.rne.scene.toml"),
        root.join("assets/scenes/deformable_cloth.rne.scene.toml"),
        root.join("assets/scenes/unitree_g1_cloth_handling.rne.scene.toml"),
        root.join("assets/scenes/unitree_go2_dynamic.rne.scene.toml"),
        root.join("assets/scenes/unitree_go2_terrain.rne.scene.toml"),
    ];
    let robots = [
        root.join("assets/robots/diff_drive.rne.robot.toml"),
        root.join("assets/robots/diff_drive_urdf.rne.robot.toml"),
        root.join("assets/robots/mm_minimal.rne.robot.toml"),
        root.join("assets/robots/mm_mobile.rne.robot.toml"),
        root.join("assets/robots/mm_lift.rne.robot.toml"),
        root.join("assets/robots/unitree_g1_29dof_dex3_fixed.rne.robot.toml"),
    ];

    for scene in scenes {
        rne_assets::validate_asset(&scene).map_err(|error| {
            anyhow::anyhow!("asset validation failed for {}: {error}", scene.display())
        })?;
        let robot_count = rne_assets::smoke_spawn_scene(&scene).map_err(|error| {
            anyhow::anyhow!("asset spawn smoke failed for {}: {error}", scene.display())
        })?;
        println!("validated scene {} (robots={robot_count})", scene.display());
    }

    for robot in robots {
        rne_assets::validate_asset(&robot).map_err(|error| {
            anyhow::anyhow!("asset validation failed for {}: {error}", robot.display())
        })?;
        println!("validated robot {}", robot.display());
    }

    Ok(())
}

fn venv_python(root: &Path) -> PathBuf {
    if cfg!(windows) {
        root.join(".venv/Scripts/python.exe")
    } else {
        root.join(".venv/bin/python")
    }
}

fn mobile_manipulator_rl_smokes() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let host_python = python_command()?;
    let venv_py = venv_python(&root);
    if !venv_py.exists() {
        run_step(&format!("{host_python} -m venv .venv"))?;
    }
    run_program(&venv_py, &["release/test_python_api_compat.py"])?;
    run_program(
        &venv_py,
        &["-m", "pip", "install", "-q", "--upgrade", "pip", "maturin"],
    )?;
    run_program(
        &venv_py,
        &[
            "-m",
            "pip",
            "install",
            "-q",
            "-r",
            "examples/27_mobile_manipulator_rl/requirements-ci.txt",
        ],
    )?;
    run_program(
        &venv_py,
        &[
            "-m",
            "maturin",
            "develop",
            "-m",
            "crates/rne_py/Cargo.toml",
            "--release",
        ],
    )?;
    run_program(
        &venv_py,
        &[
            "release/python_api_compat.py",
            "--fixture",
            "release/python-api-v1.json",
            "--output",
            "artifacts/python-api/report.json",
        ],
    )?;
    for script in [
        "run.py",
        "train_place.py",
        "train_visuomotor.py",
        "train_clutter.py",
        "train_clutter_ppo.py",
        "train_mobile_clutter.py",
        "train_mobile_clutter_ppo.py",
        "train_ppo.py",
    ] {
        let script_path = format!("examples/27_mobile_manipulator_rl/{script}");
        run_program(&venv_py, &[&script_path, "--smoke"])?;
    }
    for script in ["run.py", "train_cem.py", "train_ppo.py"] {
        let script_path = format!("examples/66_locomotion_rl/{script}");
        run_program(&venv_py, &[&script_path, "--smoke"])?;
    }
    Ok(())
}

fn asset_command(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let subcommand = args.next().unwrap_or_else(|| "validate".to_string());
    let path = args.next().map(PathBuf::from).unwrap_or_else(|| {
        workspace_root()
            .expect("workspace root")
            .join("assets/scenes/episode_diff_drive.rne.scene.toml")
    });

    match subcommand.as_str() {
        "validate" => {
            let validated = rne_assets::validate_asset(&path)?;
            match validated {
                rne_assets::ValidatedAsset::Scene(bundle) => {
                    println!(
                        "valid scene: robots={} seed={}",
                        bundle.robots.len(),
                        bundle.scene.world.seed
                    );
                    let robot_count = rne_assets::smoke_spawn_scene(&path)?;
                    println!("spawn ok: robots={robot_count}");
                }
                rne_assets::ValidatedAsset::Robot { asset, .. } => {
                    println!(
                        "valid robot: kind={:?} model={}",
                        asset.kind, asset.model_name
                    );
                }
            }
        }
        "inspect" => {
            println!("{}", rne_assets::inspect_asset(&path)?);
        }
        other => anyhow::bail!("unknown asset subcommand: {other}"),
    }

    Ok(())
}

fn ci_ros2() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let script = root.join("adapters/ros2/rne_ros2_node/smoke_test.sh");
    if !script.is_file() {
        anyhow::bail!("missing ROS 2 smoke script at {}", script.display());
    }
    if !ros_setup_available() {
        println!("ROS 2 setup.bash not found under /opt/ros; skipping ci-ros2");
        return Ok(());
    }
    run_step(&format!("bash {}", script.display()))?;
    Ok(())
}

fn ci_ros2_bridge() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let script = root.join("adapters/ros2/rne_ros2_bridge/smoke_test.sh");
    if !script.is_file() {
        anyhow::bail!("missing ROS 2 bridge smoke script at {}", script.display());
    }
    if !ros_setup_available() {
        println!("ROS 2 setup.bash not found under /opt/ros; skipping ci-ros2-bridge");
        return Ok(());
    }
    run_step(&format!("bash {}", script.display()))?;
    Ok(())
}

fn ros_setup_available() -> bool {
    PathBuf::from("/opt/ros/jazzy/setup.bash").is_file()
        || PathBuf::from("/opt/ros/humble/setup.bash").is_file()
}

fn lint_boundaries() -> anyhow::Result<()> {
    let workspace_root = workspace_root()?;
    let forbidden = ["rcl", "rclrs", "rclcpp", "ros2", "adapters/", "../adapters"];

    for manifest in find_cargo_tomls(&workspace_root.join("crates"))? {
        let content = std::fs::read_to_string(&manifest)?;
        for line in content.lines() {
            let trimmed = line.trim();
            if !trimmed.starts_with('"') && !trimmed.contains(" = ") {
                continue;
            }
            for pattern in forbidden {
                if trimmed.contains(pattern) {
                    anyhow::bail!(
                        "forbidden dependency in core crate {}: {}",
                        manifest.display(),
                        trimmed
                    );
                }
            }
        }
    }

    let traffic_manifest = workspace_root.join("crates/rne_traffic/Cargo.toml");
    let traffic_content = std::fs::read_to_string(&traffic_manifest)?;
    let traffic_forbidden = [
        "rne_ai",
        "rne_physics",
        "rne_plateau",
        "rne_render",
        "rne_robot",
        "rne_sensor",
        "rapier",
        "wgpu",
        "sumo",
        "opendrive",
        "lanelet",
    ];
    for line in traffic_content.lines() {
        let trimmed = line.trim();
        if !trimmed.starts_with('"') && !trimmed.contains(" = ") {
            continue;
        }
        for pattern in traffic_forbidden {
            if trimmed.contains(pattern) {
                anyhow::bail!(
                    "forbidden dependency in traffic domain {}: {}",
                    traffic_manifest.display(),
                    trimmed
                );
            }
        }
    }

    println!("dependency boundary check passed");
    Ok(())
}

/// On Linux CI with `pin` installed, optionally regenerate and diff Pinocchio goldens.
///
/// Set `RNE_SKIP_PINOCCHIO_GOLDEN=1` to skip. Set `RNE_PINOCCHIO_REGEN=1` to enable
/// regeneration (default: skip regen, rely on committed JSON + `cargo test`).
fn pinocchio_golden_optional() -> anyhow::Result<()> {
    if std::env::var("RNE_SKIP_PINOCCHIO_GOLDEN").is_ok() {
        eprintln!("skipping pinocchio golden check (RNE_SKIP_PINOCCHIO_GOLDEN is set)");
        return Ok(());
    }
    if std::env::var("RNE_PINOCCHIO_REGEN").is_err() {
        return Ok(());
    }
    if !cfg!(target_os = "linux") {
        eprintln!("pinocchio golden regen is Linux-only; using committed goldens");
        return Ok(());
    }
    let has_pin = Command::new("python3")
        .args(["-c", "import pinocchio"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false);
    if !has_pin {
        eprintln!("pin not installed; using committed pinocchio goldens");
        return Ok(());
    }
    run_step("python3 scripts/pinocchio_reference.py --write-golden")?;
    run_step("git diff --exit-code -- tests/golden/dynamics/")?;
    Ok(())
}

fn run_step(command: &str) -> anyhow::Result<()> {
    println!("$ {command}");
    let status = if cfg!(windows) {
        Command::new("cmd").args(["/C", command]).status()?
    } else {
        Command::new("sh").arg("-c").arg(command).status()?
    };

    if status.success() {
        Ok(())
    } else {
        anyhow::bail!("command failed with status {status}");
    }
}

fn run_step_output(command: &str) -> anyhow::Result<String> {
    println!("$ {command}");
    let output = if cfg!(windows) {
        Command::new("cmd").args(["/C", command]).output()?
    } else {
        Command::new("sh").arg("-c").arg(command).output()?
    };

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    print!("{stdout}");
    eprint!("{stderr}");

    if output.status.success() {
        Ok(format!("{stdout}\n{stderr}"))
    } else {
        anyhow::bail!("command failed with status {}", output.status);
    }
}

/// Runs a catalog command while retaining its output for a parity report.
fn run_step_capture(command: &str) -> anyhow::Result<(bool, String)> {
    println!("$ {command}");
    let output = if cfg!(windows) {
        Command::new("cmd").args(["/C", command]).output()?
    } else {
        Command::new("sh").arg("-c").arg(command).output()?
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    print!("{stdout}");
    eprint!("{stderr}");
    Ok((output.status.success(), format!("{stdout}\n{stderr}")))
}

fn run_program(program: &Path, args: &[&str]) -> anyhow::Result<()> {
    println!("$ {} {}", program.display(), args.join(" "));
    let status = Command::new(program).args(args).status()?;
    if status.success() {
        Ok(())
    } else {
        anyhow::bail!("command failed with status {status}");
    }
}

fn workspace_root() -> anyhow::Result<PathBuf> {
    let output = Command::new("cargo")
        .args(["metadata", "--locked", "--format-version", "1", "--no-deps"])
        .output()?;

    if !output.status.success() {
        anyhow::bail!("cargo metadata failed");
    }

    let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)?;
    let root = metadata["workspace_root"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("missing workspace_root in cargo metadata"))?;

    Ok(PathBuf::from(root))
}

fn find_cargo_tomls(dir: &std::path::Path) -> anyhow::Result<Vec<PathBuf>> {
    let mut manifests = Vec::new();
    if !dir.exists() {
        return Ok(manifests);
    }

    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            manifests.extend(find_cargo_tomls(&path)?);
        } else if path.file_name().is_some_and(|name| name == "Cargo.toml") {
            manifests.push(path);
        }
    }

    Ok(manifests)
}

#[cfg(test)]
mod tests {
    use super::{
        build_cargo_sbom, default_behavior_seeds, extract_hero_digest, frame_delta_ratio,
        hero_contact_sheet_filter, parse_seed_range, parse_smoke_partition, parse_utc_date_days,
        validate_blocker_registry, validate_contract_registry, validate_rust_api_baseline,
        validate_supply_chain_registry, RustApiBaselineRegistry, SmokePartition,
        SupplyChainExceptionRegistry, SUPPLY_CHAIN_POLICY_DATE,
    };

    #[test]
    fn parses_half_open_behavior_seed_ranges() {
        assert_eq!(parse_seed_range("0..3").expect("range"), vec![0, 1, 2]);
        assert!(parse_seed_range("3..3").is_err());
        assert!(parse_seed_range("3...4").is_err());
    }

    #[test]
    fn default_behavior_ci_covers_ten_seeds() {
        assert_eq!(default_behavior_seeds(), (0_u64..10).collect::<Vec<_>>());
    }

    #[test]
    fn extracts_hero_digest_from_smoke_output() {
        let output = "3D hero simulation smoke ok: digest=0xd85cd8fbdbce1cb9, base_travel=4.51 m";
        assert_eq!(
            extract_hero_digest(output).as_deref(),
            Some("0xd85cd8fbdbce1cb9")
        );
    }

    #[test]
    fn rejects_missing_or_malformed_hero_digest() {
        assert_eq!(extract_hero_digest("3D hero simulation smoke ok"), None);
        assert_eq!(extract_hero_digest("digest=d85cd8fbdbce1cb9"), None);
        assert_eq!(extract_hero_digest("digest=0xd85cd8fbdbce1cb"), None);
    }

    #[test]
    fn builds_hero_contact_sheet_filter() {
        assert_eq!(
            hero_contact_sheet_filter(),
            "select='eq(n,0)+eq(n,6)+eq(n,12)+eq(n,18)+eq(n,24)+eq(n,30)+eq(n,36)+eq(n,42)+eq(n,47)',scale=320:-1,tile=3x3"
        );
    }

    #[test]
    fn computes_frame_delta_ratio_per_pixel() {
        let previous = [0, 0, 0, 255, 10, 10, 10, 255];
        let current = [0, 0, 0, 255, 11, 10, 10, 255];

        assert_eq!(frame_delta_ratio(&previous, &current).unwrap(), 0.5);
        assert_eq!(frame_delta_ratio(&previous, &previous).unwrap(), 0.0);
    }

    #[test]
    fn rejects_frame_delta_length_mismatch() {
        assert!(frame_delta_ratio(&[0, 0, 0, 255], &[0, 0, 0]).is_err());
    }

    #[test]
    fn parses_ci_smoke_partitions() {
        assert_eq!(parse_smoke_partition(None).unwrap(), SmokePartition::All);
        assert_eq!(
            parse_smoke_partition(Some("manipulator")).unwrap(),
            SmokePartition::Manipulator
        );
        assert_eq!(
            parse_smoke_partition(Some("locomotion")).unwrap(),
            SmokePartition::Locomotion
        );
        assert_eq!(
            parse_smoke_partition(Some("assets")).unwrap(),
            SmokePartition::Assets
        );
        assert_eq!(
            parse_smoke_partition(Some("media")).unwrap(),
            SmokePartition::Media
        );
        assert!(parse_smoke_partition(Some("unknown")).is_err());
    }

    #[test]
    fn committed_release_contract_matches_compiled_versions() {
        let registry = include_str!("../../release/contracts.toml")
            .parse::<toml::Value>()
            .expect("release contract TOML");
        validate_contract_registry(&registry).expect("release contract must match constants");
    }

    #[test]
    fn committed_rust_api_baseline_is_immutable_and_complete() {
        let root = super::workspace_root().expect("workspace root");
        let metadata = super::cargo_metadata(&root).expect("cargo metadata");
        let registry: RustApiBaselineRegistry =
            toml::from_str(include_str!("../../release/rust-api-baseline.toml"))
                .expect("Rust API baseline TOML");
        validate_rust_api_baseline(&root, &metadata, &registry)
            .expect("committed Rust API baseline");

        let mut incomplete = registry.clone();
        incomplete.package.pop();
        assert!(validate_rust_api_baseline(&root, &metadata, &incomplete).is_err());

        let mut retargeted = registry;
        retargeted.baseline_tree = "0000000000000000000000000000000000000000".to_string();
        assert!(validate_rust_api_baseline(&root, &metadata, &retargeted).is_err());
    }

    #[test]
    fn open_critical_release_blocker_is_rejected() {
        let registry = r#"
            schema_version = 1
            release_version = "0.1.0"

            [[blocker]]
            id = "RNE-TEST"
            severity = "P1"
            status = "open"
        "#
        .parse::<toml::Value>()
        .expect("blocker TOML");
        assert!(validate_blocker_registry(&registry).is_err());
    }

    #[test]
    fn malformed_release_blocker_cannot_bypass_the_exit_gate() {
        let registry = r#"
            schema_version = 1
            release_version = "0.1.0"

            [[blocker]]
            id = "RNE-TEST"
            severity = "P1"
            status = "Open"
            summary = "case-changing open must not bypass the gate"
            owner = "release-team"
            evidence = "test"
        "#
        .parse::<toml::Value>()
        .expect("blocker TOML");
        assert!(validate_blocker_registry(&registry).is_err());
    }

    #[test]
    fn committed_supply_chain_exceptions_match_the_lockfile() {
        let registry: SupplyChainExceptionRegistry =
            toml::from_str(include_str!("../../release/supply-chain-exceptions.toml"))
                .expect("supply-chain exception TOML");
        let lock = include_str!("../../Cargo.lock")
            .parse::<toml::Value>()
            .expect("Cargo.lock TOML");
        let policy_days = parse_utc_date_days(SUPPLY_CHAIN_POLICY_DATE).expect("policy date");
        validate_supply_chain_registry(&registry, &lock, policy_days)
            .expect("supply-chain exceptions must match the locked graph");
    }

    #[test]
    fn validates_utc_dates_including_leap_days() {
        assert!(parse_utc_date_days("2024-02-29").is_ok());
        assert!(parse_utc_date_days("2023-02-29").is_err());
        assert!(parse_utc_date_days("2026-13-01").is_err());
    }

    #[test]
    fn cargo_sbom_sorts_packages_features_and_dependencies() {
        let registry: SupplyChainExceptionRegistry = toml::from_str(
            r#"
                schema_version = 1
                release_version = "0.1.0"
                policy_date = "2026-08-12"
            "#,
        )
        .expect("registry");
        let metadata = serde_json::json!({
            "workspace_members": ["path+file:///repo#app@1.0.0"],
            "packages": [
                {
                    "id": "path+file:///repo#app@1.0.0",
                    "name": "app",
                    "version": "1.0.0",
                    "source": null,
                    "checksum": null,
                    "license": "MIT"
                },
                {
                    "id": "registry+https://github.com/rust-lang/crates.io-index#dep@2.0.0",
                    "name": "dep",
                    "version": "2.0.0",
                    "source": "registry+https://github.com/rust-lang/crates.io-index",
                    "checksum": "abc",
                    "license": "Apache-2.0"
                }
            ],
            "resolve": {
                "nodes": [
                    {
                        "id": "path+file:///repo#app@1.0.0",
                        "dependencies": [
                            "registry+https://github.com/rust-lang/crates.io-index#dep@2.0.0",
                            "registry+https://github.com/rust-lang/crates.io-index#dep@2.0.0"
                        ],
                        "features": ["z", "a"]
                    },
                    {
                        "id": "registry+https://github.com/rust-lang/crates.io-index#dep@2.0.0",
                        "dependencies": [],
                        "features": []
                    }
                ]
            }
        });

        let sbom = build_cargo_sbom(&metadata, &registry, "00ff".to_string()).expect("SBOM");
        assert_eq!(sbom.packages[0].name, "dep");
        assert_eq!(sbom.packages[1].features, ["a", "z"]);
        assert_eq!(sbom.packages[1].dependencies.len(), 1);
        assert_eq!(sbom.cargo_lock_sha256, "sha256:00ff");
    }
}
