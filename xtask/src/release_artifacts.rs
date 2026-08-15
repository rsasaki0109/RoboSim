//! Cross-platform 1.0 RC bundle assembly and installed-artifact rehearsal.

use super::{
    cargo_metadata, fuzz_smoke, supply_chain, validate_blocker_registry,
    validate_contract_registry, validate_release_metadata, workspace_root, RELEASE_VERSION,
};
use anyhow::{bail, Context};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{OsStr, OsString};
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Output};

/// Machine-readable release provenance report schema.
pub(crate) const RELEASE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Machine-readable installed-bundle rehearsal report schema.
pub(crate) const INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION: u32 = 2;

const RELEASE_BINARY_PACKAGES: [(&str, &str); 5] = [
    ("rne_asset_cli", "rne-asset"),
    ("rne_physics_conformance", "rne-physics-conformance"),
    ("rne_scenario_scale", "rne-scenario-scale"),
    ("rne_hardware_gateway", "rne-hardware-conformance"),
    ("rne_hardware_gateway", "rne-hardware-mock-device"),
];
const RELEASE_PLUGIN_PACKAGE: &str = "rne_plugin_example_velocity_servo";
const SHA256_MANIFEST: &str = "SHA256SUMS";
const RELEASE_REPORT: &str = "release-report.json";
const INSTALL_REPORT: &str = "install-rehearsal-report.json";
const INSTALL_CHECK_IDS: [&str; 7] = [
    "robot_replay",
    "scenario_replay",
    "physics_conformance",
    "scenario_scale_100",
    "hardware_adapter",
    "controller_plugin",
    "python_wheel",
];

const BUNDLE_FILES: [(&str, &str); 18] = [
    ("README.md", "README.md"),
    ("CHANGELOG.md", "CHANGELOG.md"),
    ("LICENSE-MIT", "LICENSE-MIT"),
    ("LICENSE-APACHE", "LICENSE-APACHE"),
    ("docs/COMPATIBILITY.md", "COMPATIBILITY.md"),
    ("docs/RELEASE_INSTALL.md", "INSTALL.md"),
    (
        "crates/rne_plugin_sdk/src/abi.rs",
        "sdk/rust/rne_plugin_sdk.rs",
    ),
    ("release/blockers.toml", "release/blockers.toml"),
    ("release/exit-matrix.toml", "release/exit-matrix.toml"),
    (
        "release/artifact-attestation.toml",
        "release/artifact-attestation.toml",
    ),
    ("release/python_wheel_smoke.py", "python-wheel-smoke.py"),
    (
        "assets/runs/mesh_diff_drive.rne.run.toml",
        "assets/runs/mesh_diff_drive.rne.run.toml",
    ),
    (
        "assets/tasks/diff_drive_goal.task.json",
        "assets/tasks/diff_drive_goal.task.json",
    ),
    (
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
    ),
    (
        "assets/robots/mesh_diff_drive.rne.robot.toml",
        "assets/robots/mesh_diff_drive.rne.robot.toml",
    ),
    (
        "assets/robots/mesh_diff_drive/mesh_diff_drive.urdf",
        "assets/robots/mesh_diff_drive/mesh_diff_drive.urdf",
    ),
    (
        "assets/robots/mesh_diff_drive/meshes/base_link.stl",
        "assets/robots/mesh_diff_drive/meshes/base_link.stl",
    ),
    (
        "assets/runs/scenario_speed.rne.run.toml",
        "assets/runs/scenario_speed.rne.run.toml",
    ),
];
const SCENARIO_FILES: [(&str, &str); 2] = [
    ("assets/scenarios/speed.xosc", "assets/scenarios/speed.xosc"),
    (
        "assets/traffic/corridor.rne.traffic.json",
        "assets/traffic/corridor.rne.traffic.json",
    ),
];

#[derive(Debug)]
struct BundleOptions {
    target: String,
    wheel: PathBuf,
    output_dir: PathBuf,
    expected_tag: Option<String>,
    python: PathBuf,
    allow_dirty: bool,
}

#[derive(Debug)]
struct InstallOptions {
    bundle_dir: PathBuf,
    output_dir: PathBuf,
    python: PathBuf,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct MemberDigest {
    path: String,
    size_bytes: u64,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct AuditVerdicts {
    cargo_deny: String,
    cargo_audit: String,
    source_policy: String,
    license_policy: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct InstallCheck {
    id: String,
    status: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct InstallRehearsalReport {
    schema_version: u32,
    release_version: String,
    target: String,
    status: String,
    checks: Vec<InstallCheck>,
}

impl InstallRehearsalReport {
    fn all_passed(&self) -> bool {
        self.status == "passed"
            && self
                .checks
                .iter()
                .map(|check| check.id.as_str())
                .eq(INSTALL_CHECK_IDS)
            && self.checks.iter().all(|check| check.status == "passed")
    }

    fn verdicts(&self) -> BTreeMap<String, String> {
        self.checks
            .iter()
            .map(|check| (check.id.clone(), check.status.clone()))
            .collect()
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
struct ReleaseReport {
    schema_version: u32,
    release_version: String,
    git_commit: String,
    target: String,
    rustc_version: String,
    cargo_version: String,
    cargo_lock_sha256: String,
    clean_worktree: bool,
    expected_tag: Option<String>,
    tag_matches_commit: bool,
    reproducible: bool,
    audit: AuditVerdicts,
    fuzz_campaign_digest_sha256: String,
    contracts: serde_json::Value,
    flagship_workflows: BTreeMap<String, String>,
    members: Vec<MemberDigest>,
}

/// Builds and stages one native release bundle, including wheel and provenance evidence.
pub(crate) fn release_bundle(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_bundle_options(args)?;
    validate_release_target(&options.target)?;
    ensure_native_target(&root, &options.target)?;

    let wheel = absolute_from(&root, &options.wheel);
    anyhow::ensure!(wheel.is_file(), "wheel does not exist: {}", wheel.display());
    anyhow::ensure!(
        wheel.extension() == Some(OsStr::new("whl")),
        "wheel must use the .whl extension: {}",
        wheel.display()
    );

    let output_root = absolute_from(&root, &options.output_dir);
    fs::create_dir_all(&output_root)
        .with_context(|| format!("create release output {}", output_root.display()))?;
    let bundle_name = bundle_name(&options.target);
    let bundle_dir = output_root.join(&bundle_name);
    reset_generated_child(&output_root, &bundle_dir, &bundle_name)?;

    let clean_worktree = git_worktree_is_clean(&root)?;
    anyhow::ensure!(
        clean_worktree || options.allow_dirty,
        "release bundle requires a clean worktree (use --allow-dirty only for local development)"
    );
    let git_commit = git_output(&root, &["rev-parse", "HEAD"])?;
    let tag_matches_commit =
        validate_expected_tag(&root, options.expected_tag.as_deref(), &git_commit)?;

    let metadata = cargo_metadata(&root)?;
    validate_release_metadata(&metadata)?;
    let blockers: toml::Value =
        toml::from_str(&fs::read_to_string(root.join("release/blockers.toml"))?)?;
    validate_blocker_registry(&blockers)?;
    let contracts: toml::Value =
        toml::from_str(&fs::read_to_string(root.join("release/contracts.toml"))?)?;
    validate_contract_registry(&contracts)?;

    build_native_artifacts(&root, &options.target)?;
    stage_static_files(&root, &bundle_dir)?;
    stage_native_artifacts(&metadata, &bundle_dir, &options.target)?;
    copy_file(
        &wheel,
        &bundle_dir
            .join("wheels")
            .join(wheel.file_name().context("wheel path has no file name")?),
    )?;

    let evidence_dir = release_evidence_dir(&metadata, &options.target)?;
    reset_generated_directory(&evidence_dir)?;
    let mut supply_args = vec![
        "--output-dir".to_string(),
        evidence_dir.to_string_lossy().into_owned(),
    ]
    .into_iter();
    supply_chain(&mut supply_args)?;
    let mut fuzz_args = vec![
        "--output-dir".to_string(),
        evidence_dir.to_string_lossy().into_owned(),
    ]
    .into_iter();
    fuzz_smoke(&mut fuzz_args)?;
    copy_file(
        &evidence_dir.join("sbom.cargo.json"),
        &bundle_dir.join("sbom.cargo.json"),
    )?;
    copy_file(
        &evidence_dir.join("cargo-lock.sha256"),
        &bundle_dir.join("evidence/cargo-lock.sha256"),
    )?;
    copy_file(
        &evidence_dir.join("report.json"),
        &bundle_dir.join("evidence/fuzz-smoke-report.json"),
    )?;

    let rehearsal_dir = output_root.join(format!(".rehearsal-{}", options.target));
    reset_generated_directory(&rehearsal_dir)?;
    let rehearsal = run_install_rehearsal(
        &bundle_dir,
        &rehearsal_dir,
        &options.python,
        &options.target,
        false,
    )?;
    write_pretty_json(&bundle_dir.join(INSTALL_REPORT), &rehearsal)?;
    anyhow::ensure!(
        rehearsal.all_passed(),
        "installed-bundle rehearsal failed; inspect {}",
        bundle_dir.join(INSTALL_REPORT).display()
    );

    let members = collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST])?;
    let fuzz: serde_json::Value =
        serde_json::from_slice(&fs::read(evidence_dir.join("report.json"))?)?;
    let fuzz_digest = fuzz["campaign_digest_sha256"]
        .as_str()
        .context("fuzz report omitted campaign_digest_sha256")?
        .to_string();
    let lock_bytes = fs::read(root.join("Cargo.lock"))?;
    let report = ReleaseReport {
        schema_version: RELEASE_REPORT_SCHEMA_VERSION,
        release_version: RELEASE_VERSION.to_string(),
        git_commit,
        target: options.target.clone(),
        rustc_version: program_version("rustc")?,
        cargo_version: program_version("cargo")?,
        cargo_lock_sha256: sha256_hex(&lock_bytes),
        clean_worktree,
        expected_tag: options.expected_tag.clone(),
        tag_matches_commit,
        reproducible: clean_worktree && options.expected_tag.is_some() && tag_matches_commit,
        audit: AuditVerdicts {
            cargo_deny: "passed".to_string(),
            cargo_audit: "passed".to_string(),
            source_policy: "passed".to_string(),
            license_policy: "passed".to_string(),
        },
        fuzz_campaign_digest_sha256: fuzz_digest,
        contracts: serde_json::to_value(contracts)?,
        flagship_workflows: rehearsal.verdicts(),
        members,
    };
    write_pretty_json(&bundle_dir.join(RELEASE_REPORT), &report)?;
    write_sha256_manifest(&bundle_dir)?;
    verify_sha256_manifest(&bundle_dir)?;

    println!(
        "release bundle ready: target={} reproducible={} path={}",
        options.target,
        report.reproducible,
        bundle_dir.display()
    );
    Ok(())
}

/// Verifies an extracted bundle and reruns every installed-artifact smoke.
pub(crate) fn release_install_smoke(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_install_options(args)?;
    let bundle_dir = absolute_from(&root, &options.bundle_dir);
    let output_dir = absolute_from(&root, &options.output_dir);
    anyhow::ensure!(
        bundle_dir.is_dir(),
        "bundle directory missing: {}",
        bundle_dir.display()
    );
    prepare_empty_directory(&output_dir)?;
    verify_sha256_manifest(&bundle_dir)?;

    let release: ReleaseReport =
        serde_json::from_slice(&fs::read(bundle_dir.join(RELEASE_REPORT))?)?;
    anyhow::ensure!(
        release.schema_version == RELEASE_REPORT_SCHEMA_VERSION
            && release.release_version == RELEASE_VERSION,
        "bundle release report is incompatible"
    );
    let payload_members = collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST])?;
    anyhow::ensure!(
        release.members == payload_members,
        "bundle payload does not match release-report.json"
    );
    validate_release_target(&release.target)?;
    let report = run_install_rehearsal(
        &bundle_dir,
        &output_dir,
        &options.python,
        &release.target,
        true,
    )?;
    write_pretty_json(&output_dir.join(INSTALL_REPORT), &report)?;
    anyhow::ensure!(
        report.all_passed(),
        "installed-bundle rehearsal failed; inspect {}",
        output_dir.join(INSTALL_REPORT).display()
    );
    println!(
        "installed release bundle passed: target={} report={}",
        release.target,
        output_dir.join(INSTALL_REPORT).display()
    );
    Ok(())
}

fn parse_bundle_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<BundleOptions> {
    let mut target = None;
    let mut wheel = None;
    let mut output_dir = PathBuf::from("artifacts/release");
    let mut expected_tag = None;
    let mut python = default_python();
    let mut allow_dirty = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--target" => target = Some(required_arg(args, "--target")?),
            "--wheel" => wheel = Some(PathBuf::from(required_arg(args, "--wheel")?)),
            "--output-dir" => output_dir = PathBuf::from(required_arg(args, "--output-dir")?),
            "--expected-tag" => expected_tag = Some(required_arg(args, "--expected-tag")?),
            "--python" => python = PathBuf::from(required_arg(args, "--python")?),
            "--allow-dirty" => allow_dirty = true,
            other => bail!("unknown release-bundle argument: {other}"),
        }
    }
    Ok(BundleOptions {
        target: target.context("release-bundle requires --target TARGET")?,
        wheel: wheel.context("release-bundle requires --wheel PATH")?,
        output_dir,
        expected_tag,
        python,
        allow_dirty,
    })
}

fn parse_install_options(
    args: &mut impl Iterator<Item = String>,
) -> anyhow::Result<InstallOptions> {
    let mut bundle_dir = None;
    let mut output_dir = PathBuf::from("artifacts/release-install-smoke");
    let mut python = default_python();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--bundle-dir" => {
                bundle_dir = Some(PathBuf::from(required_arg(args, "--bundle-dir")?));
            }
            "--output-dir" => output_dir = PathBuf::from(required_arg(args, "--output-dir")?),
            "--python" => python = PathBuf::from(required_arg(args, "--python")?),
            other => bail!("unknown release-install-smoke argument: {other}"),
        }
    }
    Ok(InstallOptions {
        bundle_dir: bundle_dir.context("release-install-smoke requires --bundle-dir PATH")?,
        output_dir,
        python,
    })
}

fn required_arg(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn default_python() -> PathBuf {
    PathBuf::from(if cfg!(windows) { "python" } else { "python3" })
}

fn validate_release_target(target: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !target.is_empty()
            && target.len() <= 96
            && target
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.')),
        "invalid release target {target:?}"
    );
    anyhow::ensure!(
        target.contains("windows") || target.contains("linux"),
        "M6 release bundles support Linux and Windows targets only"
    );
    Ok(())
}

fn bundle_name(target: &str) -> String {
    format!("rne-{RELEASE_VERSION}-{target}")
}

fn ensure_native_target(root: &Path, target: &str) -> anyhow::Result<()> {
    let output = command_output(root, Path::new("rustc"), &[OsString::from("-vV")], &[])?;
    ensure_success("rustc -vV", &output)?;
    let text = String::from_utf8_lossy(&output.stdout);
    let host = text
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .context("rustc -vV omitted host target")?;
    anyhow::ensure!(
        host == target,
        "release rehearsal must be native so the wheel and bundle match: host={host} target={target}"
    );
    Ok(())
}

fn build_native_artifacts(root: &Path, _target: &str) -> anyhow::Result<()> {
    let mut args = vec![
        OsString::from("build"),
        OsString::from("--locked"),
        OsString::from("--release"),
    ];
    for package in RELEASE_BINARY_PACKAGES
        .iter()
        .map(|(package, _)| *package)
        .collect::<BTreeSet<_>>()
    {
        args.push(OsString::from("-p"));
        args.push(OsString::from(package));
    }
    args.push(OsString::from("-p"));
    args.push(OsString::from(RELEASE_PLUGIN_PACKAGE));
    let output = command_output(root, Path::new("cargo"), &args, &[])?;
    print_output(&output);
    ensure_success("cargo build release artifacts", &output)
}

fn stage_static_files(root: &Path, bundle_dir: &Path) -> anyhow::Result<()> {
    for (source, destination) in BUNDLE_FILES.into_iter().chain(SCENARIO_FILES) {
        copy_file(&root.join(source), &bundle_dir.join(destination))?;
    }
    Ok(())
}

fn stage_native_artifacts(
    metadata: &serde_json::Value,
    bundle_dir: &Path,
    target: &str,
) -> anyhow::Result<()> {
    let target_dir = PathBuf::from(
        metadata["target_directory"]
            .as_str()
            .context("cargo metadata omitted target_directory")?,
    );
    let release_dir = target_dir.join("release");
    for (_, binary) in RELEASE_BINARY_PACKAGES {
        let file = native_binary_name(binary, target);
        copy_file(&release_dir.join(&file), &bundle_dir.join("bin").join(file))?;
    }
    let plugin = native_plugin_name(target);
    copy_file(
        &release_dir.join(&plugin),
        &bundle_dir.join("lib").join(plugin),
    )?;
    let root = workspace_root()?;
    copy_file(
        &root.join("crates/rne_plugin_example_velocity_servo/rne-plugin.json"),
        &bundle_dir.join("lib/rne-plugin.json"),
    )?;
    Ok(())
}

fn native_binary_name(name: &str, target: &str) -> String {
    if target.contains("windows") {
        format!("{name}.exe")
    } else {
        name.to_string()
    }
}

fn native_plugin_name(target: &str) -> String {
    native_cdylib_name(RELEASE_PLUGIN_PACKAGE, target)
}

fn native_cdylib_name(name: &str, target: &str) -> String {
    if target.contains("windows") {
        format!("{name}.dll")
    } else if target.contains("darwin") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

fn release_evidence_dir(metadata: &serde_json::Value, target: &str) -> anyhow::Result<PathBuf> {
    let target_dir = PathBuf::from(
        metadata["target_directory"]
            .as_str()
            .context("cargo metadata omitted target_directory")?,
    );
    Ok(target_dir.join("release-evidence").join(target))
}

fn run_install_rehearsal(
    bundle_dir: &Path,
    output_dir: &Path,
    python: &Path,
    target: &str,
    verify_checksums: bool,
) -> anyhow::Result<InstallRehearsalReport> {
    if verify_checksums {
        verify_sha256_manifest(bundle_dir)?;
    }
    fs::create_dir_all(output_dir)?;
    let bin_dir = bundle_dir.join("bin");
    let asset_cli = bin_dir.join(native_binary_name("rne-asset", target));
    let physics = bin_dir.join(native_binary_name("rne-physics-conformance", target));
    let scale = bin_dir.join(native_binary_name("rne-scenario-scale", target));
    let hardware_conformance = bin_dir.join(native_binary_name("rne-hardware-conformance", target));
    let hardware_mock = bin_dir.join(native_binary_name("rne-hardware-mock-device", target));

    let robot_replay = output_dir.join("robot.rne-replay");
    let robot_run = run_check_command(
        "robot replay generation",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("run"),
            bundle_dir
                .join("assets/runs/mesh_diff_drive.rne.run.toml")
                .into_os_string(),
            OsString::from("--replay-out"),
            robot_replay.clone().into_os_string(),
        ],
        &[],
    );
    let robot_verify = robot_run
        && run_check_command(
            "robot replay verification",
            bundle_dir,
            &asset_cli,
            &[OsString::from("replay"), robot_replay.into_os_string()],
            &[],
        );

    let scenario_replay = output_dir.join("scenario.rne-replay");
    let scenario_run = run_check_command(
        "scenario replay generation",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("run"),
            bundle_dir
                .join("assets/runs/scenario_speed.rne.run.toml")
                .into_os_string(),
            OsString::from("--replay-out"),
            scenario_replay.clone().into_os_string(),
        ],
        &[],
    );
    let scenario_verify = scenario_run
        && run_check_command(
            "scenario replay verification",
            bundle_dir,
            &asset_cli,
            &[OsString::from("replay"), scenario_replay.into_os_string()],
            &[],
        );

    let physics_report = output_dir.join("physics-conformance.json");
    let physics_passed = run_check_command(
        "physics conformance",
        bundle_dir,
        &physics,
        &[
            OsString::from("--output"),
            physics_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &physics_report,
        "all_passed",
        &serde_json::Value::Bool(true),
    );

    let scale_report = output_dir.join("scenario-scale.json");
    let scale_passed = run_check_command(
        "100-actor scenario scale",
        bundle_dir,
        &scale,
        &[
            OsString::from("--output"),
            scale_report.clone().into_os_string(),
        ],
        &[(
            OsString::from("RNE_SCENARIO_SCALE_BENCHMARK_CLASS"),
            OsString::from(format!("release-rehearsal-{target}")),
        )],
    ) && json_field_matches(
        &scale_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );

    let hardware_report = output_dir.join("hardware-adapter-conformance.json");
    let hardware_passed = run_check_command(
        "external hardware adapter conformance",
        bundle_dir,
        &hardware_conformance,
        &[
            OsString::from("--adapter"),
            hardware_mock.clone().into_os_string(),
            OsString::from("--adapter-arg"),
            OsString::from("--device-id"),
            OsString::from("--adapter-arg"),
            OsString::from("rne-release-hardware-mock-v1"),
            OsString::from("--adapter-arg"),
            OsString::from("--expected-task-id"),
            OsString::from("--adapter-arg"),
            OsString::from("rne.diff_drive.goal.v1"),
            OsString::from("--adapter-arg"),
            OsString::from("--observation-width"),
            OsString::from("--adapter-arg"),
            OsString::from("9"),
            OsString::from("--adapter-arg"),
            OsString::from("--action-width"),
            OsString::from("--adapter-arg"),
            OsString::from("2"),
            OsString::from("--task"),
            bundle_dir
                .join("assets/tasks/diff_drive_goal.task.json")
                .into_os_string(),
            OsString::from("--allow-hil"),
            OsString::from("--output"),
            hardware_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &hardware_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );

    let plugin_report = output_dir.join("controller-plugin-conformance.json");
    let reference_plugin_passed = run_check_command(
        "controller plugin conformance",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("check"),
            OsString::from("--library"),
            bundle_dir
                .join("lib")
                .join(native_plugin_name(target))
                .into_os_string(),
            OsString::from("--manifest"),
            bundle_dir.join("lib/rne-plugin.json").into_os_string(),
            OsString::from("--output"),
            plugin_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &plugin_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );
    let scaffold_plugin_passed = run_scaffold_rehearsal(bundle_dir, output_dir, &asset_cli, target);
    let plugin_passed = reference_plugin_passed && scaffold_plugin_passed;

    let wheel_passed = run_python_wheel_smoke(bundle_dir, output_dir, python, target);
    let checks = vec![
        check("robot_replay", robot_verify),
        check("scenario_replay", scenario_verify),
        check("physics_conformance", physics_passed),
        check("scenario_scale_100", scale_passed),
        check("hardware_adapter", hardware_passed),
        check("controller_plugin", plugin_passed),
        check("python_wheel", wheel_passed),
    ];
    let passed = checks.iter().all(|check| check.status == "passed");
    Ok(InstallRehearsalReport {
        schema_version: INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        release_version: RELEASE_VERSION.to_string(),
        target: target.to_string(),
        status: if passed { "passed" } else { "failed" }.to_string(),
        checks,
    })
}

fn run_scaffold_rehearsal(
    bundle_dir: &Path,
    output_dir: &Path,
    asset_cli: &Path,
    target: &str,
) -> bool {
    const NAME: &str = "release_scaffold_controller";
    let parent = output_dir.join("controller-authoring");
    if !run_check_command(
        "scaffold controller plugin",
        bundle_dir,
        asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("new"),
            OsString::from(NAME),
            OsString::from("--dir"),
            parent.clone().into_os_string(),
        ],
        &[],
    ) {
        return false;
    }
    let crate_dir = parent.join(NAME);
    let bundled_sdk = bundle_dir.join("sdk/rust/rne_plugin_sdk.rs");
    let scaffold_sdk = crate_dir.join("src/rne_plugin_sdk.rs");
    match (fs::read(&bundled_sdk), fs::read(&scaffold_sdk)) {
        (Ok(bundled), Ok(scaffolded)) if bundled == scaffolded => {}
        (Ok(_), Ok(_)) => {
            eprintln!("scaffold SDK differs from bundled SDK source");
            return false;
        }
        (Err(error), _) => {
            eprintln!(
                "could not read bundled SDK {}: {error}",
                bundled_sdk.display()
            );
            return false;
        }
        (_, Err(error)) => {
            eprintln!(
                "could not read scaffold SDK {}: {error}",
                scaffold_sdk.display()
            );
            return false;
        }
    }
    let scaffold_target = parent.join("target");
    if !run_check_command(
        "build scaffolded controller offline",
        &crate_dir,
        Path::new("cargo"),
        &[
            OsString::from("build"),
            OsString::from("--offline"),
            OsString::from("--manifest-path"),
            crate_dir.join("Cargo.toml").into_os_string(),
            OsString::from("--target-dir"),
            scaffold_target.clone().into_os_string(),
        ],
        &[(OsString::from("RUSTFLAGS"), OsString::from("-Dwarnings"))],
    ) {
        return false;
    }
    let report = output_dir.join("controller-scaffold-conformance.json");
    run_check_command(
        "scaffolded controller conformance",
        bundle_dir,
        asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("check"),
            OsString::from("--library"),
            scaffold_target
                .join("debug")
                .join(native_cdylib_name(NAME, target))
                .into_os_string(),
            OsString::from("--manifest"),
            crate_dir.join("rne-plugin.json").into_os_string(),
            OsString::from("--output"),
            report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    )
}

fn run_python_wheel_smoke(
    bundle_dir: &Path,
    output_dir: &Path,
    python: &Path,
    target: &str,
) -> bool {
    let wheels = match files_with_extension(&bundle_dir.join("wheels"), "whl") {
        Ok(wheels) if wheels.len() == 1 => wheels,
        Ok(wheels) => {
            eprintln!("expected exactly one wheel, found {}", wheels.len());
            return false;
        }
        Err(error) => {
            eprintln!("could not enumerate bundled wheel: {error:#}");
            return false;
        }
    };
    let venv = output_dir.join("wheel-venv");
    if venv.exists() {
        if let Err(error) = fs::remove_dir_all(&venv) {
            eprintln!("could not reset wheel venv {}: {error}", venv.display());
            return false;
        }
    }
    if !run_check_command(
        "create wheel smoke venv",
        output_dir,
        python,
        &[
            OsString::from("-m"),
            OsString::from("venv"),
            venv.clone().into_os_string(),
        ],
        &[],
    ) {
        return false;
    }
    let installed_python = if target.contains("windows") {
        venv.join("Scripts/python.exe")
    } else {
        venv.join("bin/python")
    };
    if !run_check_command(
        "install bundled ABI3 wheel",
        output_dir,
        &installed_python,
        &[
            OsString::from("-m"),
            OsString::from("pip"),
            OsString::from("install"),
            OsString::from("--disable-pip-version-check"),
            OsString::from("--no-index"),
            OsString::from("--no-deps"),
            OsString::from("--force-reinstall"),
            wheels[0].clone().into_os_string(),
        ],
        &[],
    ) {
        return false;
    }
    run_check_command(
        "execute ABI3 wheel smoke",
        output_dir,
        &installed_python,
        &[bundle_dir.join("python-wheel-smoke.py").into_os_string()],
        &[],
    )
}

fn check(id: &str, passed: bool) -> InstallCheck {
    InstallCheck {
        id: id.to_string(),
        status: if passed { "passed" } else { "failed" }.to_string(),
    }
}

fn json_field_matches(path: &Path, field: &str, expected: &serde_json::Value) -> bool {
    let result = fs::read(path)
        .map_err(anyhow::Error::from)
        .and_then(|bytes| serde_json::from_slice::<serde_json::Value>(&bytes).map_err(Into::into));
    match result {
        Ok(value) if value.get(field) == Some(expected) => true,
        Ok(value) => {
            eprintln!(
                "{} field {field:?} did not match {expected}: {:?}",
                path.display(),
                value.get(field)
            );
            false
        }
        Err(error) => {
            eprintln!("could not validate {}: {error:#}", path.display());
            false
        }
    }
}

fn run_check_command(
    label: &str,
    cwd: &Path,
    program: &Path,
    args: &[OsString],
    envs: &[(OsString, OsString)],
) -> bool {
    println!("$ {} ({label})", program.display());
    match command_output(cwd, program, args, envs) {
        Ok(output) => {
            print_output(&output);
            if !output.status.success() {
                eprintln!("{label} failed with status {}", output.status);
            }
            output.status.success()
        }
        Err(error) => {
            eprintln!("{label} could not start: {error:#}");
            false
        }
    }
}

fn command_output(
    cwd: &Path,
    program: &Path,
    args: &[OsString],
    envs: &[(OsString, OsString)],
) -> anyhow::Result<Output> {
    Command::new(program)
        .current_dir(cwd)
        .args(args)
        .envs(envs.iter().cloned())
        .output()
        .with_context(|| format!("run {}", program.display()))
}

fn print_output(output: &Output) {
    print!("{}", String::from_utf8_lossy(&output.stdout));
    eprint!("{}", String::from_utf8_lossy(&output.stderr));
}

fn ensure_success(label: &str, output: &Output) -> anyhow::Result<()> {
    anyhow::ensure!(
        output.status.success(),
        "{label} failed with status {}",
        output.status
    );
    Ok(())
}

fn program_version(program: &str) -> anyhow::Result<String> {
    let output = Command::new(program).arg("--version").output()?;
    ensure_success(&format!("{program} --version"), &output)?;
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn git_worktree_is_clean(root: &Path) -> anyhow::Result<bool> {
    Ok(git_output(root, &["status", "--porcelain", "--untracked-files=all"])?.is_empty())
}

fn validate_expected_tag(
    root: &Path,
    expected_tag: Option<&str>,
    commit: &str,
) -> anyhow::Result<bool> {
    let Some(tag) = expected_tag else {
        return Ok(false);
    };
    anyhow::ensure!(
        tag == format!("v{RELEASE_VERSION}"),
        "expected release tag must be v{RELEASE_VERSION}, got {tag}"
    );
    let reference = format!("refs/tags/{tag}^{{commit}}");
    let tag_commit = git_output(root, &["rev-parse", &reference])?;
    anyhow::ensure!(
        tag_commit == commit,
        "release tag {tag} points to {tag_commit}, not tested commit {commit}"
    );
    Ok(true)
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

fn absolute_from(root: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    }
}

fn reset_generated_child(parent: &Path, path: &Path, expected_name: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        path.parent() == Some(parent) && path.file_name() == Some(OsStr::new(expected_name)),
        "refusing to reset unexpected generated path {}",
        path.display()
    );
    reset_generated_directory(path)
}

fn reset_generated_directory(path: &Path) -> anyhow::Result<()> {
    if path.exists() {
        fs::remove_dir_all(path)
            .with_context(|| format!("remove generated directory {}", path.display()))?;
    }
    fs::create_dir_all(path)
        .with_context(|| format!("create generated directory {}", path.display()))
}

fn prepare_empty_directory(path: &Path) -> anyhow::Result<()> {
    if path.exists() {
        anyhow::ensure!(
            fs::read_dir(path)?.next().is_none(),
            "release-install-smoke output directory must be empty: {}",
            path.display()
        );
    } else {
        fs::create_dir_all(path)?;
    }
    Ok(())
}

fn copy_file(source: &Path, destination: &Path) -> anyhow::Result<()> {
    anyhow::ensure!(
        source.is_file(),
        "bundle source missing: {}",
        source.display()
    );
    if let Some(parent) = destination.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::copy(source, destination).with_context(|| {
        format!(
            "copy bundle member {} to {}",
            source.display(),
            destination.display()
        )
    })?;
    Ok(())
}

fn files_with_extension(directory: &Path, extension: &str) -> anyhow::Result<Vec<PathBuf>> {
    let mut files = fs::read_dir(directory)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.is_file() && path.extension() == Some(OsStr::new(extension)))
        .collect::<Vec<_>>();
    files.sort();
    Ok(files)
}

fn collect_files(root: &Path) -> anyhow::Result<Vec<PathBuf>> {
    fn visit(directory: &Path, files: &mut Vec<PathBuf>) -> anyhow::Result<()> {
        let mut entries = fs::read_dir(directory)?.collect::<Result<Vec<_>, _>>()?;
        entries.sort_by_key(std::fs::DirEntry::file_name);
        for entry in entries {
            let path = entry.path();
            let file_type = entry.file_type()?;
            anyhow::ensure!(
                !file_type.is_symlink(),
                "bundle contains symbolic link {}",
                path.display()
            );
            if file_type.is_dir() {
                visit(&path, files)?;
            } else if file_type.is_file() {
                files.push(path);
            } else {
                bail!("bundle contains unsupported member {}", path.display());
            }
        }
        Ok(())
    }

    let mut files = Vec::new();
    visit(root, &mut files)?;
    Ok(files)
}

fn collect_member_digests(root: &Path, excluded: &[&str]) -> anyhow::Result<Vec<MemberDigest>> {
    let mut members = Vec::new();
    for path in collect_files(root)? {
        let relative = member_path(root, &path)?;
        if excluded.contains(&relative.as_str()) {
            continue;
        }
        let bytes = fs::read(&path)?;
        members.push(MemberDigest {
            path: relative,
            size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
            sha256: sha256_hex(&bytes),
        });
    }
    members.sort_by(|left, right| left.path.cmp(&right.path));
    Ok(members)
}

fn member_path(root: &Path, path: &Path) -> anyhow::Result<String> {
    let relative = path.strip_prefix(root)?;
    validate_relative_member(relative)?;
    Ok(relative.to_string_lossy().replace('\\', "/"))
}

fn validate_relative_member(path: &Path) -> anyhow::Result<()> {
    anyhow::ensure!(
        !path.as_os_str().is_empty()
            && !path.is_absolute()
            && path
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "invalid bundle member path {}",
        path.display()
    );
    Ok(())
}

fn write_sha256_manifest(root: &Path) -> anyhow::Result<()> {
    let members = collect_member_digests(root, &[SHA256_MANIFEST])?;
    let mut text = String::new();
    for member in members {
        text.push_str(&member.sha256);
        text.push_str("  ");
        text.push_str(&member.path);
        text.push('\n');
    }
    fs::write(root.join(SHA256_MANIFEST), text)?;
    Ok(())
}

fn verify_sha256_manifest(root: &Path) -> anyhow::Result<()> {
    let text = fs::read_to_string(root.join(SHA256_MANIFEST))
        .with_context(|| format!("read {SHA256_MANIFEST} from {}", root.display()))?;
    let mut declared = BTreeMap::new();
    for line in text.lines() {
        let (digest, path) = line
            .split_once("  ")
            .context("SHA256SUMS entries must use `<digest>  <path>`")?;
        anyhow::ensure!(
            digest.len() == 64 && digest.bytes().all(|byte| byte.is_ascii_hexdigit()),
            "invalid SHA-256 digest for {path}"
        );
        let path_buf = PathBuf::from(path);
        validate_relative_member(&path_buf)?;
        anyhow::ensure!(
            declared
                .insert(path.to_string(), digest.to_ascii_lowercase())
                .is_none(),
            "duplicate SHA256SUMS member {path}"
        );
    }
    let actual = collect_member_digests(root, &[SHA256_MANIFEST])?
        .into_iter()
        .map(|member| (member.path, member.sha256))
        .collect::<BTreeMap<_, _>>();
    anyhow::ensure!(
        declared == actual,
        "bundle SHA256SUMS does not match its members"
    );
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn write_pretty_json(path: &Path, value: &impl Serialize) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn release_target_is_bounded_and_cannot_escape() {
        assert!(validate_release_target("x86_64-unknown-linux-gnu").is_ok());
        assert!(validate_release_target("x86_64-pc-windows-msvc").is_ok());
        assert!(validate_release_target("../windows").is_err());
        assert!(validate_release_target("macos").is_err());
    }

    #[test]
    fn native_artifact_names_match_platform_conventions() {
        assert_eq!(
            native_binary_name("rne-asset", "x86_64-pc-windows-msvc"),
            "rne-asset.exe"
        );
        assert_eq!(
            native_plugin_name("x86_64-pc-windows-msvc"),
            "rne_plugin_example_velocity_servo.dll"
        );
        assert_eq!(
            native_plugin_name("x86_64-unknown-linux-gnu"),
            "librne_plugin_example_velocity_servo.so"
        );
        assert_eq!(
            native_cdylib_name("custom_controller", "aarch64-apple-darwin"),
            "libcustom_controller.dylib"
        );
    }

    #[test]
    fn checksum_manifest_rejects_tampering_and_unlisted_files() {
        let directory = tempfile::tempdir().expect("temporary bundle");
        fs::write(directory.path().join("a.txt"), b"stable").expect("member");
        write_sha256_manifest(directory.path()).expect("manifest");
        let valid_manifest = fs::read_to_string(directory.path().join(SHA256_MANIFEST))
            .expect("read valid manifest");
        verify_sha256_manifest(directory.path()).expect("valid manifest");

        fs::write(directory.path().join("a.txt"), b"changed").expect("tamper");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::write(directory.path().join("a.txt"), b"stable").expect("restore");
        fs::write(directory.path().join("extra.txt"), b"extra").expect("extra");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::remove_file(directory.path().join("extra.txt")).expect("remove extra");

        fs::remove_file(directory.path().join("a.txt")).expect("remove member");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::write(directory.path().join("a.txt"), b"stable").expect("restore member");

        fs::write(
            directory.path().join(SHA256_MANIFEST),
            format!("{valid_manifest}{valid_manifest}"),
        )
        .expect("duplicate manifest entry");
        assert!(verify_sha256_manifest(directory.path()).is_err());

        let digest = sha256_hex(b"stable");
        fs::write(
            directory.path().join(SHA256_MANIFEST),
            format!("{digest}  ../a.txt\n"),
        )
        .expect("traversal manifest entry");
        assert!(verify_sha256_manifest(directory.path()).is_err());
    }

    #[test]
    fn install_report_requires_every_frozen_workflow() {
        let report = InstallRehearsalReport {
            schema_version: INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
            release_version: RELEASE_VERSION.to_string(),
            target: "x86_64-unknown-linux-gnu".to_string(),
            status: "passed".to_string(),
            checks: [
                "robot_replay",
                "scenario_replay",
                "physics_conformance",
                "scenario_scale_100",
                "hardware_adapter",
                "controller_plugin",
                "python_wheel",
            ]
            .map(|id| check(id, true))
            .to_vec(),
        };
        assert!(report.all_passed());

        let mut duplicated = report;
        duplicated.checks[6].id = "robot_replay".to_string();
        assert!(!duplicated.all_passed());
    }
}
