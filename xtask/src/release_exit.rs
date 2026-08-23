//! Machine-readable final exit matrix for the 1.0 RC.

use super::{release_readiness, validate_blocker_registry, workspace_root, RELEASE_VERSION};
use anyhow::Context;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

/// Machine-readable final exit report schema.
pub(crate) const FINAL_EXIT_REPORT_SCHEMA_VERSION: u32 = 1;
/// Machine-readable release artifact attestation policy schema.
pub(crate) const ARTIFACT_ATTESTATION_POLICY_SCHEMA_VERSION: u32 = 1;

const EXIT_MATRIX_PATH: &str = "release/exit-matrix.toml";
pub(crate) const EXPECTED_ATTESTATION_PROVIDER: &str = "github_sigstore";
const EXPECTED_ATTESTATION_ACTION: &str = "actions/attest@v4";
pub(crate) const EXPECTED_ATTESTATION_ISSUER: &str = "https://token.actions.githubusercontent.com";
pub(crate) const EXPECTED_ATTESTATION_REPOSITORY: &str = "rsasaki0109/RoboSim";
pub(crate) const EXPECTED_ATTESTATION_WORKFLOW: &str = ".github/workflows/release.yml";
pub(crate) const EXPECTED_ATTESTATION_PREDICATE: &str = "https://slsa.dev/provenance/v1";
const EXPECTED_SCOPES: [&str; 2] = ["ci", "release"];
const EXPECTED_AGGREGATE_CHECKS: [&str; 2] =
    ["CI / workspace", "Release rehearsal / release_candidate"];
const EXPECTED_CI_JOBS: [&str; 14] = [
    "lint",
    "test",
    "smoke",
    "rl",
    "headless",
    "flagship",
    "msrv",
    "release_contract",
    "semver",
    "behavior_ci",
    "evidence",
    "parity",
    "supply_chain",
    "fuzz_smoke",
];
const EXPECTED_RELEASE_JOBS: [&str; 2] = ["linux", "windows"];

#[derive(Debug, Deserialize)]
struct ExitMatrix {
    schema_version: u32,
    release_version: String,
    locked_graph: String,
    blocker_registry: String,
    artifact_attestation_policy: String,
    required_aggregate_checks: Vec<String>,
    scope: Vec<ExitScope>,
    gate: Vec<ExitGate>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ArtifactAttestationPolicy {
    schema_version: u32,
    provider: String,
    action: String,
    issuer: String,
    repository: String,
    workflow: String,
    predicate_type: String,
    subjects: Vec<String>,
    attested_events: Vec<String>,
    pull_request_attestations: bool,
    require_source_ref: bool,
    deny_self_hosted_runners: bool,
    verify_command: String,
}

#[derive(Debug, Deserialize)]
struct ExitScope {
    id: String,
    workflow: String,
    aggregate_job: String,
    required_jobs: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct ExitGate {
    id: String,
    scope: String,
    workflow: String,
    job: String,
    runner: String,
    clean_checkout: bool,
    commands: Vec<String>,
}

#[derive(Debug)]
struct ExitOptions {
    scope: String,
    results: BTreeMap<String, String>,
    output_dir: PathBuf,
    allow_dirty: bool,
}

#[derive(Debug, Serialize)]
struct FinalExitReport {
    schema_version: u32,
    release_version: String,
    scope: String,
    git_commit: String,
    cargo_lock_sha256: String,
    clean_checkout: bool,
    development_dirty_override: bool,
    zero_open_p0_p1_blockers: bool,
    all_required_checks_green: bool,
    release_eligible: bool,
    required_aggregate_checks: Vec<String>,
    jobs: Vec<ExitJobVerdict>,
    gates: Vec<ExitGateEvidence>,
}

#[derive(Debug, Serialize)]
struct ExitJobVerdict {
    job: String,
    status: String,
}

#[derive(Debug, Serialize)]
struct ExitGateEvidence {
    id: String,
    workflow: String,
    job: String,
    runner: String,
    clean_checkout_required: bool,
    commands: Vec<String>,
    status: String,
}

/// Validates the committed exit contract and records one aggregate workflow verdict.
pub(crate) fn release_exit(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_options(args)?;
    release_readiness::enforce_release_promotion(&root)?;
    let matrix = read_and_validate_matrix(&root)?;
    let scope = matrix
        .scope
        .iter()
        .find(|scope| scope.id == options.scope)
        .with_context(|| format!("unknown release-exit scope {:?}", options.scope))?;

    let expected_jobs = scope.required_jobs.iter().cloned().collect::<BTreeSet<_>>();
    let actual_jobs = options.results.keys().cloned().collect::<BTreeSet<_>>();
    anyhow::ensure!(
        actual_jobs == expected_jobs,
        "release-exit results differ for scope {}: expected={expected_jobs:?} actual={actual_jobs:?}",
        scope.id
    );
    for (job, result) in &options.results {
        anyhow::ensure!(
            matches!(
                result.as_str(),
                "success" | "failure" | "cancelled" | "skipped"
            ),
            "unsupported result {result:?} for job {job}"
        );
    }

    let blocker_path = safe_repo_path(&root, &matrix.blocker_registry)?;
    let blockers = fs::read_to_string(&blocker_path)
        .with_context(|| format!("read blocker registry {}", blocker_path.display()))?
        .parse::<toml::Value>()?;
    validate_blocker_registry(&blockers)?;

    let clean_checkout = git_worktree_is_clean(&root)?;
    let all_required_checks_green = options.results.values().all(|value| value == "success");
    let zero_open_p0_p1_blockers = true;
    let release_eligible = clean_checkout && zero_open_p0_p1_blockers && all_required_checks_green;
    let gates = matrix
        .gate
        .iter()
        .filter(|gate| gate.scope == scope.id)
        .map(|gate| ExitGateEvidence {
            id: gate.id.clone(),
            workflow: gate.workflow.clone(),
            job: gate.job.clone(),
            runner: gate.runner.clone(),
            clean_checkout_required: gate.clean_checkout,
            commands: gate.commands.clone(),
            status: options
                .results
                .get(&gate.job)
                .expect("validated gate job must have a result")
                .clone(),
        })
        .collect::<Vec<_>>();
    let lock_bytes = fs::read(safe_repo_path(&root, &matrix.locked_graph)?)?;
    let report = FinalExitReport {
        schema_version: FINAL_EXIT_REPORT_SCHEMA_VERSION,
        release_version: RELEASE_VERSION.to_string(),
        scope: scope.id.clone(),
        git_commit: git_output(&root, &["rev-parse", "HEAD"])?,
        cargo_lock_sha256: format!("{:x}", Sha256::digest(lock_bytes)),
        clean_checkout,
        development_dirty_override: options.allow_dirty && !clean_checkout,
        zero_open_p0_p1_blockers,
        all_required_checks_green,
        release_eligible,
        required_aggregate_checks: matrix.required_aggregate_checks.clone(),
        jobs: scope
            .required_jobs
            .iter()
            .map(|job| ExitJobVerdict {
                job: job.clone(),
                status: options.results[job].clone(),
            })
            .collect(),
        gates,
    };

    let output_dir = absolute_from(&root, &options.output_dir);
    fs::create_dir_all(&output_dir)?;
    let report_path = output_dir.join(format!("{}.json", scope.id));
    fs::write(&report_path, serde_json::to_vec_pretty(&report)?)?;

    anyhow::ensure!(
        all_required_checks_green,
        "scope {} has a non-success required job; inspect {}",
        scope.id,
        report_path.display()
    );
    anyhow::ensure!(
        clean_checkout || options.allow_dirty,
        "release exit evidence requires a clean checkout (use --allow-dirty only for local development)"
    );
    println!(
        "release exit scope passed: scope={} release_eligible={} report={}",
        scope.id,
        release_eligible,
        report_path.display()
    );
    Ok(())
}

/// Checks that the machine-readable matrix and both workflow aggregate gates agree.
pub(crate) fn validate_exit_matrix(root: &Path) -> anyhow::Result<()> {
    read_and_validate_matrix(root).map(|_| ())
}

fn parse_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<ExitOptions> {
    let mut scope = None;
    let mut results = BTreeMap::new();
    let mut output_dir = PathBuf::from("artifacts/release-exit");
    let mut allow_dirty = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scope" => scope = Some(next_value(args, "--scope")?),
            "--result" => {
                let value = next_value(args, "--result")?;
                let (job, result) = value
                    .split_once('=')
                    .with_context(|| format!("--result must use JOB=STATUS, got {value:?}"))?;
                anyhow::ensure!(
                    !job.is_empty() && !result.is_empty(),
                    "empty --result field"
                );
                anyhow::ensure!(
                    results
                        .insert(job.to_string(), result.to_string())
                        .is_none(),
                    "duplicate result for job {job}"
                );
            }
            "--output-dir" => output_dir = PathBuf::from(next_value(args, "--output-dir")?),
            "--allow-dirty" => allow_dirty = true,
            other => anyhow::bail!("unknown release-exit argument: {other}"),
        }
    }
    Ok(ExitOptions {
        scope: scope.context("release-exit requires --scope ci|release")?,
        results,
        output_dir,
        allow_dirty,
    })
}

fn next_value(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn read_and_validate_matrix(root: &Path) -> anyhow::Result<ExitMatrix> {
    let matrix_path = root.join(EXIT_MATRIX_PATH);
    let matrix: ExitMatrix = toml::from_str(
        &fs::read_to_string(&matrix_path)
            .with_context(|| format!("read exit matrix {}", matrix_path.display()))?,
    )?;
    anyhow::ensure!(
        matrix.schema_version == 1,
        "exit matrix schema_version must be 1"
    );
    anyhow::ensure!(
        matrix.release_version == RELEASE_VERSION,
        "exit matrix release_version must be {RELEASE_VERSION}"
    );
    anyhow::ensure!(
        matrix.required_aggregate_checks == EXPECTED_AGGREGATE_CHECKS.map(str::to_string),
        "exit matrix aggregate checks must be the two frozen RC checks"
    );
    anyhow::ensure!(
        safe_repo_path(root, &matrix.locked_graph)?.is_file(),
        "exit matrix locked graph is missing"
    );
    anyhow::ensure!(
        safe_repo_path(root, &matrix.blocker_registry)?.is_file(),
        "exit matrix blocker registry is missing"
    );
    let attestation_policy_path = safe_repo_path(root, &matrix.artifact_attestation_policy)?;
    let attestation_policy: ArtifactAttestationPolicy = toml::from_str(
        &fs::read_to_string(&attestation_policy_path).with_context(|| {
            format!(
                "read artifact attestation policy {}",
                attestation_policy_path.display()
            )
        })?,
    )?;
    validate_attestation_policy(&attestation_policy)?;

    let scope_ids = matrix
        .scope
        .iter()
        .map(|scope| scope.id.as_str())
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        scope_ids == EXPECTED_SCOPES.into_iter().collect(),
        "exit matrix must define exactly ci and release scopes"
    );
    let mut gate_ids = BTreeSet::new();
    for gate in &matrix.gate {
        anyhow::ensure!(
            gate_ids.insert(gate.id.as_str()),
            "duplicate exit gate {}",
            gate.id
        );
        anyhow::ensure!(
            scope_ids.contains(gate.scope.as_str()),
            "unknown scope for gate {}",
            gate.id
        );
        anyhow::ensure!(
            gate.clean_checkout,
            "gate {} must require a clean checkout",
            gate.id
        );
        anyhow::ensure!(!gate.commands.is_empty(), "gate {} has no command", gate.id);
        for command in &gate.commands {
            validate_locked_command(&gate.id, command)?;
        }
    }

    for scope in &matrix.scope {
        let expected_jobs = match scope.id.as_str() {
            "ci" => EXPECTED_CI_JOBS.as_slice(),
            "release" => EXPECTED_RELEASE_JOBS.as_slice(),
            _ => unreachable!("scope set already validated"),
        };
        anyhow::ensure!(
            scope.required_jobs == expected_jobs,
            "scope {} required jobs changed: expected={expected_jobs:?} actual={:?}",
            scope.id,
            scope.required_jobs
        );
        let gate_jobs = matrix
            .gate
            .iter()
            .filter(|gate| gate.scope == scope.id)
            .map(|gate| gate.job.as_str())
            .collect::<BTreeSet<_>>();
        let scope_gate_count = matrix
            .gate
            .iter()
            .filter(|gate| gate.scope == scope.id)
            .count();
        anyhow::ensure!(
            gate_jobs == expected_jobs.iter().copied().collect()
                && scope_gate_count == expected_jobs.len(),
            "scope {} gate jobs differ from its required jobs",
            scope.id
        );
        anyhow::ensure!(
            matrix
                .gate
                .iter()
                .filter(|gate| gate.scope == scope.id)
                .all(|gate| gate.workflow == scope.workflow),
            "scope {} gates must use workflow {}",
            scope.id,
            scope.workflow
        );
        validate_workflow(root, scope, &matrix.gate)?;
    }
    let release_scope = matrix
        .scope
        .iter()
        .find(|scope| scope.id == "release")
        .expect("validated release scope");
    anyhow::ensure!(
        attestation_policy.workflow == release_scope.workflow,
        "artifact attestation policy must protect the release workflow"
    );
    validate_release_attestation_workflow(root, release_scope, &attestation_policy)?;
    Ok(matrix)
}

fn validate_attestation_policy(policy: &ArtifactAttestationPolicy) -> anyhow::Result<()> {
    anyhow::ensure!(
        policy.schema_version == ARTIFACT_ATTESTATION_POLICY_SCHEMA_VERSION,
        "artifact attestation policy schema_version must be {ARTIFACT_ATTESTATION_POLICY_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        policy.provider == EXPECTED_ATTESTATION_PROVIDER,
        "artifact attestation provider must be {EXPECTED_ATTESTATION_PROVIDER}"
    );
    anyhow::ensure!(
        policy.action == EXPECTED_ATTESTATION_ACTION,
        "artifact attestation action must be {EXPECTED_ATTESTATION_ACTION}"
    );
    anyhow::ensure!(
        policy.issuer == EXPECTED_ATTESTATION_ISSUER,
        "artifact attestation issuer must be {EXPECTED_ATTESTATION_ISSUER}"
    );
    anyhow::ensure!(
        policy.repository == EXPECTED_ATTESTATION_REPOSITORY,
        "artifact attestation repository must be {EXPECTED_ATTESTATION_REPOSITORY}"
    );
    anyhow::ensure!(
        policy.predicate_type == EXPECTED_ATTESTATION_PREDICATE,
        "artifact attestation predicate must be {EXPECTED_ATTESTATION_PREDICATE}"
    );
    anyhow::ensure!(
        policy.subjects == ["native_archive", "python_wheel", "archive_install_report"],
        "artifact attestation subjects must be native_archive, python_wheel, and archive_install_report"
    );
    anyhow::ensure!(
        policy.attested_events == ["push_tag", "workflow_dispatch"],
        "artifact attestations must run for tag pushes and manual rehearsals"
    );
    anyhow::ensure!(
        !policy.pull_request_attestations,
        "pull-request jobs must not mint artifact attestations"
    );
    anyhow::ensure!(
        policy.require_source_ref,
        "artifact attestation verification must bind the release tag ref"
    );
    anyhow::ensure!(
        policy.deny_self_hosted_runners,
        "artifact attestation verification must reject self-hosted builders"
    );
    anyhow::ensure!(
        policy.verify_command
            == format!(
                "gh attestation verify ARTIFACT -R {repository} --bundle ATTESTATION_BUNDLE --cert-identity https://github.com/{repository}/{workflow}@refs/tags/TAG --source-ref refs/tags/TAG --source-digest REVISION --signer-digest REVISION --cert-oidc-issuer {issuer} --predicate-type {predicate} --deny-self-hosted-runners --format json",
                repository = EXPECTED_ATTESTATION_REPOSITORY,
                workflow = EXPECTED_ATTESTATION_WORKFLOW,
                issuer = EXPECTED_ATTESTATION_ISSUER,
                predicate = EXPECTED_ATTESTATION_PREDICATE,
            ),
        "artifact attestation verifier drifted"
    );
    Ok(())
}

fn validate_release_attestation_workflow(
    root: &Path,
    scope: &ExitScope,
    policy: &ArtifactAttestationPolicy,
) -> anyhow::Result<()> {
    let workflow_path = safe_repo_path(root, &scope.workflow)?;
    let workflow = fs::read_to_string(&workflow_path)?;
    for (job, archive_glob, retain_bundle) in [
        (
            "linux",
            "artifacts/release/*.tar.gz",
            "cp ${{ steps.attest.outputs.bundle-path }} artifacts/attestations/release-bundle.json",
        ),
        (
            "windows",
            "artifacts/release/*.zip",
            "Copy-Item -LiteralPath ${{ steps.attest.outputs.bundle-path }} -Destination artifacts/attestations/release-bundle.json",
        ),
    ] {
        let block = normalize_workflow(workflow_job_block(&workflow, job)?);
        for permission in [
            "contents: read",
            "id-token: write",
            "attestations: write",
            "artifact-metadata: write",
        ] {
            anyhow::ensure!(
                block.contains(permission),
                "release job {job} omitted attestation permission {permission}"
            );
        }
        anyhow::ensure!(
            block.contains(&format!("uses: {}", policy.action)),
            "release job {job} must use {}",
            policy.action
        );
        anyhow::ensure!(
            block.contains("id: attest") && block.contains(retain_bundle),
            "release job {job} must retain the exact generated attestation bundle"
        );
        anyhow::ensure!(
            block.contains("if: github.event_name != pull_request"),
            "release job {job} must not attest pull-request artifacts"
        );
        anyhow::ensure!(
            block.contains(archive_glob) && block.contains("artifacts/wheels/*.whl"),
            "release job {job} must attest its archive and Python wheel"
        );
        let install_report =
            "artifacts/extracted-evidence/archive-install-rehearsal-report.json";
        anyhow::ensure!(
            block.matches(install_report).count() >= 2,
            "release job {job} must attest and retain its archive-bound install report"
        );
    }

    let publish = normalize_workflow(workflow_job_block(&workflow, "publish")?);
    anyhow::ensure!(
        publish.contains("attestations: read"),
        "release publish job must be allowed to read attestations"
    );
    let verify = "gh attestation verify $asset";
    let verify_index = publish
        .find(verify)
        .context("release publish job must verify every artifact attestation")?;
    let report_verify_index = publish
        .find("gh attestation verify $report")
        .context("release publish job must verify each archive-install report attestation")?;
    for requirement in [
        "-R $GH_REPO",
        "--bundle $bundle",
        "--cert-identity https://github.com/$GH_REPO/.github/workflows/release.yml@$GITHUB_REF",
        "--source-ref $GITHUB_REF",
        "--source-digest $GITHUB_SHA",
        "--signer-digest $GITHUB_SHA",
        "--cert-oidc-issuer https://token.actions.githubusercontent.com",
        "--predicate-type https://slsa.dev/provenance/v1",
        "--deny-self-hosted-runners",
    ] {
        anyhow::ensure!(
            publish.contains(requirement),
            "release publish verifier omitted policy requirement {requirement}"
        );
    }
    let publish_index = publish
        .find("gh release create")
        .context("release publish job omitted gh release create")?;
    anyhow::ensure!(
        verify_index < publish_index && report_verify_index < publish_index,
        "release assets and archive-install reports must be attestation-verified before publication"
    );
    Ok(())
}

fn validate_locked_command(gate: &str, command: &str) -> anyhow::Result<()> {
    let graph_command = command.starts_with("cargo run ")
        || command.starts_with("cargo check ")
        || command.starts_with("cargo test ")
        || command.starts_with("cargo build ")
        || command.starts_with("maturin build ");
    anyhow::ensure!(
        !graph_command || command.split_whitespace().any(|part| part == "--locked"),
        "exit gate {gate} graph command is not locked: {command}"
    );
    Ok(())
}

fn validate_workflow(root: &Path, scope: &ExitScope, gates: &[ExitGate]) -> anyhow::Result<()> {
    let workflow_path = safe_repo_path(root, &scope.workflow)?;
    anyhow::ensure!(
        scope.workflow.starts_with(".github/workflows/"),
        "exit scope {} workflow must live under .github/workflows",
        scope.id
    );
    let workflow = fs::read_to_string(&workflow_path)
        .with_context(|| format!("read workflow {}", workflow_path.display()))?;
    let workflow_header = workflow
        .split_once("jobs:")
        .map_or(workflow.as_str(), |part| part.0);
    anyhow::ensure!(
        workflow_header.contains("  pull_request:"),
        "workflow {} must run for pull requests",
        scope.workflow
    );
    anyhow::ensure!(
        !workflow_header.contains("    paths:") && !workflow_header.contains("    paths-ignore:"),
        "workflow {} must not omit release gates through pull-request path filters",
        scope.workflow
    );
    for gate in gates.iter().filter(|gate| gate.scope == scope.id) {
        let block = workflow_job_block(&workflow, &gate.job)?;
        let normalized = normalize_workflow(block);
        anyhow::ensure!(
            normalized.contains(&normalize_workflow(&format!("runs-on: {}", gate.runner))),
            "gate {} runner drifted from {}",
            gate.id,
            gate.runner
        );
        anyhow::ensure!(
            !gate.clean_checkout || normalized.contains("uses: actions/checkout@v4"),
            "gate {} no longer starts from actions/checkout@v4",
            gate.id
        );
        if gate.id == "release_contract" {
            anyhow::ensure!(
                normalized.contains("fetch-depth: 0"),
                "release contract gate must retain full history for migration provenance"
            );
        }
        for command in &gate.commands {
            anyhow::ensure!(
                normalized.contains(&normalize_workflow(command)),
                "gate {} command drifted from workflow: {}",
                gate.id,
                command
            );
        }
        if gate.id == "semver" {
            validate_semver_packages(block)?;
        }
    }

    let aggregate = normalize_workflow(workflow_job_block(&workflow, &scope.aggregate_job)?);
    let expected_needs = format!("needs: [{}]", scope.required_jobs.join(", "));
    anyhow::ensure!(
        aggregate.contains(&normalize_workflow(&expected_needs)),
        "aggregate job {} must need every {} scope job",
        scope.aggregate_job,
        scope.id
    );
    anyhow::ensure!(
        aggregate.contains("if: always()"),
        "aggregate job {} must run even when a dependency fails",
        scope.aggregate_job
    );
    let aggregate_command = format!(
        "cargo run --locked -p xtask -- release-exit --scope {}",
        scope.id
    );
    anyhow::ensure!(
        aggregate.contains(&aggregate_command),
        "aggregate job {} must emit {} exit evidence",
        scope.aggregate_job,
        scope.id
    );
    if scope.id == "release" {
        let publish = normalize_workflow(workflow_job_block(&workflow, "publish")?);
        anyhow::ensure!(
            publish.contains("needs: [release_candidate]"),
            "release publishing must depend on the aggregate release_candidate job"
        );
    }
    Ok(())
}

fn validate_semver_packages(block: &str) -> anyhow::Result<()> {
    let mut declared = Vec::new();
    for (index, _) in block.match_indices("-p ") {
        let package = block[index + 3..]
            .chars()
            .take_while(|character| character.is_ascii_alphanumeric() || *character == '_')
            .collect::<String>();
        if package.starts_with("rne_") {
            declared.push(package);
        }
    }
    let unique = declared.iter().cloned().collect::<BTreeSet<_>>();
    anyhow::ensure!(
        unique.len() == declared.len(),
        "SemVer matrix contains duplicate public packages"
    );
    let expected = super::PUBLIC_RELEASE_PACKAGES
        .iter()
        .map(|package| (*package).to_string())
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        unique == expected,
        "SemVer matrix public packages changed: missing={:?} extra={:?}",
        expected.difference(&unique).collect::<Vec<_>>(),
        unique.difference(&expected).collect::<Vec<_>>()
    );
    let normalized = normalize_workflow(block);
    for fixed_baseline_check in [
        "release/rust-api-baseline.toml",
        "cargo metadata --locked --no-deps --format-version 1",
        "git diff --quiet \"$registry_guard_ref\" -- release/rust-api-baseline.toml",
        "previous[\"release_version\"]",
        "if [[ \"$previous_release\" == \"$current_release\" ]]",
        "Rust API baseline release version must increase",
        "git cat-file -e \"$baseline^{commit}\"",
        "git merge-base --is-ancestor \"$baseline\" HEAD",
        "git cat-file -e \"$baseline:$frozen_manifest\"",
        "cargo semver-checks --version",
        "packages+=(\"-p\" \"$package\")",
    ] {
        anyhow::ensure!(
            normalized.contains(&normalize_workflow(fixed_baseline_check)),
            "SemVer matrix omitted fixed baseline check: {fixed_baseline_check}"
        );
    }
    for moving_baseline in [
        "baseline=\"origin/${{ github.base_ref }}\"",
        "baseline=\"HEAD^\"",
    ] {
        anyhow::ensure!(
            !normalized.contains(&normalize_workflow(moving_baseline)),
            "SemVer matrix still uses moving baseline: {moving_baseline}"
        );
    }
    Ok(())
}

fn workflow_job_block<'a>(workflow: &'a str, job: &str) -> anyhow::Result<&'a str> {
    let marker = format!("  {job}:");
    let mut start = None;
    let mut offset = 0;
    for line in workflow.split_inclusive('\n') {
        if line.trim_end_matches(['\r', '\n']) == marker {
            start = Some(offset);
            break;
        }
        offset += line.len();
    }
    let start = start.with_context(|| format!("workflow omitted job {job}"))?;
    let header_len = workflow[start..]
        .find('\n')
        .map_or(workflow.len() - start, |index| index + 1);
    let after_header = start + header_len;
    let mut end = workflow.len();
    let mut block_offset = after_header;
    for line in workflow[after_header..].split_inclusive('\n') {
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.starts_with("  ") && !trimmed.starts_with("   ") && trimmed.ends_with(':') {
            end = block_offset;
            break;
        }
        block_offset += line.len();
    }
    Ok(&workflow[start..end])
}

fn normalize_workflow(value: &str) -> String {
    value
        .replace(['"', '\''], "")
        .split_whitespace()
        .filter(|part| *part != "\\" && *part != "`")
        .collect::<Vec<_>>()
        .join(" ")
}

fn safe_repo_path(root: &Path, relative: &str) -> anyhow::Result<PathBuf> {
    let path = Path::new(relative);
    anyhow::ensure!(
        !path.is_absolute(),
        "exit matrix path must be relative: {relative}"
    );
    anyhow::ensure!(
        path.components()
            .all(|component| matches!(component, Component::Normal(_))),
        "exit matrix path is unsafe: {relative}"
    );
    Ok(root.join(path))
}

fn absolute_from(root: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    }
}

fn git_worktree_is_clean(root: &Path) -> anyhow::Result<bool> {
    Ok(git_output(root, &["status", "--porcelain"])?.is_empty())
}

fn git_output(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(String::from_utf8(output.stdout)?.trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::{
        normalize_workflow, parse_options, validate_attestation_policy, validate_exit_matrix,
        validate_locked_command, workflow_job_block, ArtifactAttestationPolicy,
    };

    #[test]
    fn committed_exit_matrix_matches_workflows() {
        let root = super::workspace_root().expect("workspace root");
        validate_exit_matrix(&root).expect("committed exit matrix");
    }

    #[test]
    fn parses_exact_job_results() {
        let mut args = [
            "--scope",
            "release",
            "--result",
            "linux=success",
            "--result",
            "windows=failure",
        ]
        .into_iter()
        .map(str::to_string);
        let options = parse_options(&mut args).expect("options");
        assert_eq!(options.scope, "release");
        assert_eq!(options.results["linux"], "success");
        assert_eq!(options.results["windows"], "failure");
    }

    #[test]
    fn rejects_unlocked_graph_commands() {
        assert!(validate_locked_command("test", "cargo test --workspace").is_err());
        assert!(validate_locked_command("test", "cargo test --locked --workspace").is_ok());
        assert!(validate_locked_command("semver", "cargo semver-checks check-release").is_ok());
    }

    #[test]
    fn attestation_policy_rejects_repository_drift() {
        let root = super::workspace_root().expect("workspace root");
        let text = std::fs::read_to_string(root.join("release/artifact-attestation.toml"))
            .expect("attestation policy");
        let mut policy: ArtifactAttestationPolicy =
            toml::from_str(&text).expect("parse attestation policy");
        validate_attestation_policy(&policy).expect("committed policy");
        policy.repository = "untrusted/fork".to_string();
        assert!(validate_attestation_policy(&policy).is_err());
    }

    #[test]
    fn isolates_workflow_jobs_and_normalizes_shell_continuations() {
        let workflow = r"jobs:
  linux:
    runs-on: ubuntu-latest
    run: cargo test \
      --locked
  windows:
    runs-on: windows-latest
";
        let block = workflow_job_block(workflow, "linux").expect("linux job");
        assert!(normalize_workflow(block).contains("cargo test --locked"));
        assert!(!block.contains("windows-latest"));
    }
}
