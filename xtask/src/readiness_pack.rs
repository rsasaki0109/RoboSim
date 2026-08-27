//! Safe authoring helpers for an external RNE 1.0 readiness evidence pack.

use super::{external_simulator, release_artifacts, release_readiness, workspace_root};
use anyhow::{bail, Context, Result};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::ffi::OsString;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Component, Path, PathBuf};

const MANIFEST_NAME: &str = "one-zero-readiness.toml";
const BASELINE_MANIFEST: &str = "release/one-zero-readiness.toml";
const COMPATIBILITY_SOURCE: &str = "release/evidence/compatibility-report-v1.json";
const COMPATIBILITY_DESTINATION: &str = "evidence/compatibility-report-v1.json";

#[derive(Debug, Eq, PartialEq)]
struct StagedEvidence {
    relative_path: String,
    sha256: String,
    size_bytes: u64,
}

/// Creates or adds a file to an external 1.0-readiness evidence pack.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = workspace_root()?;
    let command = args
        .next()
        .context(
            "readiness-pack requires a command: init, stage, accept-installed-flagship, or accept-external-simulator; use readiness-pack --help",
        )?;
    match command.as_str() {
        "init" => {
            let output = parse_init_options(args, &root)?;
            init_pack(&root, &output)?;
            println!(
                "readiness pack initialized: manifest={}",
                output.join(MANIFEST_NAME).display()
            );
            Ok(())
        }
        "stage" => {
            let options = parse_stage_options(args, &root)?;
            let staged = stage_file(&root, &options.pack, &options.source, &options.path)?;
            println!("{}", inline_reference(&staged));
            println!("size_bytes = {}", staged.size_bytes);
            Ok(())
        }
        "accept-installed-flagship" => {
            let options = parse_accept_installed_flagship_options(args, &root)?;
            accept_installed_flagship(&root, &options)?;
            println!(
                "installed flagship evidence accepted: id={} manifest={}",
                options.id,
                options.pack.join(MANIFEST_NAME).display()
            );
            Ok(())
        }
        "accept-external-simulator" => {
            let options = parse_accept_external_simulator_options(args, &root)?;
            accept_external_simulator(&root, &options)?;
            println!(
                "external simulator evidence accepted: id={} manifest={}",
                options.id,
                options.pack.join(MANIFEST_NAME).display()
            );
            Ok(())
        }
        "--help" | "-h" => {
            print_usage();
            Ok(())
        }
        other => bail!("unknown readiness-pack command: {other}"),
    }
}

#[derive(Debug)]
struct StageOptions {
    pack: PathBuf,
    source: PathBuf,
    path: String,
}

#[derive(Debug)]
struct AcceptInstalledFlagshipOptions {
    pack: PathBuf,
    id: String,
    owner: String,
    repository: String,
    revision: String,
    measured_on: String,
    release_archive: String,
    proof_bundle: String,
    submission_candidate: String,
    stdout_log: String,
    stderr_log: String,
    report: String,
}

#[derive(Debug, Eq, PartialEq)]
struct AcceptedInstalledFlagshipEvidence {
    release_archive: StagedEvidence,
    proof_bundle: StagedEvidence,
    submission_candidate: StagedEvidence,
    stdout_log: StagedEvidence,
    stderr_log: StagedEvidence,
    report: StagedEvidence,
}

#[derive(Debug)]
struct AcceptExternalSimulatorOptions {
    pack: PathBuf,
    id: String,
    owner: String,
    repository: String,
    revision: String,
    adapter: String,
    task_spec: String,
    runtime_manifest: String,
    runtime_artifacts: Vec<String>,
    adapter_arguments: Vec<String>,
    conformance_report: String,
    release_archive: String,
    submission_candidate: String,
    stdout_log: String,
    stderr_log: String,
    submission_report: String,
}

#[derive(Debug, Eq, PartialEq)]
struct AcceptedExternalSimulatorEvidence {
    adapter: StagedEvidence,
    task_spec: StagedEvidence,
    runtime_manifest: StagedEvidence,
    runtime_artifacts: Vec<StagedEvidence>,
    conformance_report: StagedEvidence,
    release_archive: StagedEvidence,
    submission_candidate: StagedEvidence,
    stdout_log: StagedEvidence,
    stderr_log: StagedEvidence,
    submission_report: StagedEvidence,
}

fn print_usage() {
    println!("readiness-pack init --output DIR");
    println!("readiness-pack stage --pack DIR --source FILE --path FORWARD/SLASH/PATH");
    println!(
        "readiness-pack accept-installed-flagship --pack DIR --id ID --owner OWNER \\\n+         --repository URL --revision COMMIT --measured-on YYYY-MM-DD \\\n+         --release-archive PATH --proof-bundle PATH --submission-candidate PATH \\\n+         --stdout-log PATH --stderr-log PATH --report PATH"
    );
    println!(
        "readiness-pack accept-external-simulator --pack DIR --id ID --owner OWNER \\\n+         --repository URL --revision COMMIT --adapter PATH --task-spec PATH \\\n+         --runtime-manifest PATH --runtime-artifact PATH (three times) \\\n+         --adapter-argument VALUE (repeat as required) --conformance-report PATH \\\n+         --release-archive PATH --submission-candidate PATH --stdout-log PATH \\\n+         --stderr-log PATH --submission-report PATH"
    );
}

fn parse_init_options(args: &mut impl Iterator<Item = String>, root: &Path) -> Result<PathBuf> {
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" => {
                set_once(
                    &mut output,
                    absolute_from(root, next_value(args, "--output")?),
                    "--output",
                )?;
            }
            other => bail!("unknown readiness-pack init argument: {other}"),
        }
    }
    output.context("readiness-pack init requires --output DIR")
}

fn parse_stage_options(
    args: &mut impl Iterator<Item = String>,
    root: &Path,
) -> Result<StageOptions> {
    let mut pack = None;
    let mut source = None;
    let mut path = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--pack" => set_once(
                &mut pack,
                absolute_from(root, next_value(args, "--pack")?),
                "--pack",
            )?,
            "--source" => set_once(
                &mut source,
                absolute_from(root, next_value(args, "--source")?),
                "--source",
            )?,
            "--path" => set_once(&mut path, next_value(args, "--path")?, "--path")?,
            other => bail!("unknown readiness-pack stage argument: {other}"),
        }
    }
    Ok(StageOptions {
        pack: pack.context("readiness-pack stage requires --pack DIR")?,
        source: source.context("readiness-pack stage requires --source FILE")?,
        path: path.context("readiness-pack stage requires --path RELATIVE/PATH")?,
    })
}

fn parse_accept_installed_flagship_options(
    args: &mut impl Iterator<Item = String>,
    root: &Path,
) -> Result<AcceptInstalledFlagshipOptions> {
    let mut pack = None;
    let mut id = None;
    let mut owner = None;
    let mut repository = None;
    let mut revision = None;
    let mut measured_on = None;
    let mut release_archive = None;
    let mut proof_bundle = None;
    let mut submission_candidate = None;
    let mut stdout_log = None;
    let mut stderr_log = None;
    let mut report = None;
    while let Some(argument) = args.next() {
        anyhow::ensure!(
            matches!(
                argument.as_str(),
                "--pack"
                    | "--id"
                    | "--owner"
                    | "--repository"
                    | "--revision"
                    | "--measured-on"
                    | "--release-archive"
                    | "--proof-bundle"
                    | "--submission-candidate"
                    | "--stdout-log"
                    | "--stderr-log"
                    | "--report"
            ),
            "unknown readiness-pack accept-installed-flagship argument: {argument}"
        );
        let value = next_value(args, &argument)?;
        match argument.as_str() {
            "--pack" => set_once(&mut pack, absolute_from(root, value), "--pack")?,
            "--id" => set_once(&mut id, value, "--id")?,
            "--owner" => set_once(&mut owner, value, "--owner")?,
            "--repository" => set_once(&mut repository, value, "--repository")?,
            "--revision" => set_once(&mut revision, value, "--revision")?,
            "--measured-on" => set_once(&mut measured_on, value, "--measured-on")?,
            "--release-archive" => set_once(&mut release_archive, value, "--release-archive")?,
            "--proof-bundle" => set_once(&mut proof_bundle, value, "--proof-bundle")?,
            "--submission-candidate" => {
                set_once(&mut submission_candidate, value, "--submission-candidate")?
            }
            "--stdout-log" => set_once(&mut stdout_log, value, "--stdout-log")?,
            "--stderr-log" => set_once(&mut stderr_log, value, "--stderr-log")?,
            "--report" => set_once(&mut report, value, "--report")?,
            _ => unreachable!("accepted argument was matched above"),
        }
    }
    Ok(AcceptInstalledFlagshipOptions {
        pack: pack.context("accept-installed-flagship requires --pack DIR")?,
        id: id.context("accept-installed-flagship requires --id ID")?,
        owner: owner.context("accept-installed-flagship requires --owner OWNER")?,
        repository: repository.context("accept-installed-flagship requires --repository URL")?,
        revision: revision.context("accept-installed-flagship requires --revision COMMIT")?,
        measured_on: measured_on
            .context("accept-installed-flagship requires --measured-on YYYY-MM-DD")?,
        release_archive: release_archive
            .context("accept-installed-flagship requires --release-archive PATH")?,
        proof_bundle: proof_bundle
            .context("accept-installed-flagship requires --proof-bundle PATH")?,
        submission_candidate: submission_candidate
            .context("accept-installed-flagship requires --submission-candidate PATH")?,
        stdout_log: stdout_log.context("accept-installed-flagship requires --stdout-log PATH")?,
        stderr_log: stderr_log.context("accept-installed-flagship requires --stderr-log PATH")?,
        report: report.context("accept-installed-flagship requires --report PATH")?,
    })
}

fn parse_accept_external_simulator_options(
    args: &mut impl Iterator<Item = String>,
    root: &Path,
) -> Result<AcceptExternalSimulatorOptions> {
    let mut pack = None;
    let mut id = None;
    let mut owner = None;
    let mut repository = None;
    let mut revision = None;
    let mut adapter = None;
    let mut task_spec = None;
    let mut runtime_manifest = None;
    let mut runtime_artifacts = Vec::new();
    let mut adapter_arguments = Vec::new();
    let mut conformance_report = None;
    let mut release_archive = None;
    let mut submission_candidate = None;
    let mut stdout_log = None;
    let mut stderr_log = None;
    let mut submission_report = None;
    while let Some(argument) = args.next() {
        anyhow::ensure!(
            matches!(
                argument.as_str(),
                "--pack"
                    | "--id"
                    | "--owner"
                    | "--repository"
                    | "--revision"
                    | "--adapter"
                    | "--task-spec"
                    | "--runtime-manifest"
                    | "--runtime-artifact"
                    | "--adapter-argument"
                    | "--conformance-report"
                    | "--release-archive"
                    | "--submission-candidate"
                    | "--stdout-log"
                    | "--stderr-log"
                    | "--submission-report"
            ),
            "unknown readiness-pack accept-external-simulator argument: {argument}"
        );
        let value = next_value(args, &argument)?;
        match argument.as_str() {
            "--pack" => set_once(&mut pack, absolute_from(root, value), "--pack")?,
            "--id" => set_once(&mut id, value, "--id")?,
            "--owner" => set_once(&mut owner, value, "--owner")?,
            "--repository" => set_once(&mut repository, value, "--repository")?,
            "--revision" => set_once(&mut revision, value, "--revision")?,
            "--adapter" => set_once(&mut adapter, value, "--adapter")?,
            "--task-spec" => set_once(&mut task_spec, value, "--task-spec")?,
            "--runtime-manifest" => set_once(&mut runtime_manifest, value, "--runtime-manifest")?,
            "--runtime-artifact" => runtime_artifacts.push(value),
            "--adapter-argument" => adapter_arguments.push(value),
            "--conformance-report" => {
                set_once(&mut conformance_report, value, "--conformance-report")?
            }
            "--release-archive" => set_once(&mut release_archive, value, "--release-archive")?,
            "--submission-candidate" => {
                set_once(&mut submission_candidate, value, "--submission-candidate")?
            }
            "--stdout-log" => set_once(&mut stdout_log, value, "--stdout-log")?,
            "--stderr-log" => set_once(&mut stderr_log, value, "--stderr-log")?,
            "--submission-report" => {
                set_once(&mut submission_report, value, "--submission-report")?
            }
            _ => unreachable!("accepted argument was matched above"),
        }
    }
    anyhow::ensure!(
        runtime_artifacts.len() == 3,
        "accept-external-simulator requires exactly three ordered --runtime-artifact paths"
    );
    Ok(AcceptExternalSimulatorOptions {
        pack: pack.context("accept-external-simulator requires --pack DIR")?,
        id: id.context("accept-external-simulator requires --id ID")?,
        owner: owner.context("accept-external-simulator requires --owner OWNER")?,
        repository: repository.context("accept-external-simulator requires --repository URL")?,
        revision: revision.context("accept-external-simulator requires --revision COMMIT")?,
        adapter: adapter.context("accept-external-simulator requires --adapter PATH")?,
        task_spec: task_spec.context("accept-external-simulator requires --task-spec PATH")?,
        runtime_manifest: runtime_manifest
            .context("accept-external-simulator requires --runtime-manifest PATH")?,
        runtime_artifacts,
        adapter_arguments,
        conformance_report: conformance_report
            .context("accept-external-simulator requires --conformance-report PATH")?,
        release_archive: release_archive
            .context("accept-external-simulator requires --release-archive PATH")?,
        submission_candidate: submission_candidate
            .context("accept-external-simulator requires --submission-candidate PATH")?,
        stdout_log: stdout_log.context("accept-external-simulator requires --stdout-log PATH")?,
        stderr_log: stderr_log.context("accept-external-simulator requires --stderr-log PATH")?,
        submission_report: submission_report
            .context("accept-external-simulator requires --submission-report PATH")?,
    })
}

fn set_once<T>(slot: &mut Option<T>, value: T, option: &str) -> Result<()> {
    anyhow::ensure!(slot.is_none(), "{option} may be specified only once");
    *slot = Some(value);
    Ok(())
}

fn next_value(args: &mut impl Iterator<Item = String>, option: &str) -> Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn absolute_from(root: &Path, path: String) -> PathBuf {
    let path = PathBuf::from(path);
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn init_pack(root: &Path, output: &Path) -> Result<()> {
    release_readiness::validate_committed_manifest(root)?;
    ensure_missing(output, "readiness pack")?;
    let parent = output
        .parent()
        .context("readiness pack output must have a parent directory")?;
    fs::create_dir_all(parent)
        .with_context(|| format!("create readiness pack parent {}", parent.display()))?;
    let staging = staging_sibling(output, "rne-init")?;
    ensure_missing(&staging, "readiness pack staging path")?;
    fs::create_dir(&staging)
        .with_context(|| format!("create readiness pack staging {}", staging.display()))?;

    copy_regular_file(&root.join(BASELINE_MANIFEST), &staging.join(MANIFEST_NAME))?;
    let compatibility_destination = staging.join(COMPATIBILITY_DESTINATION);
    fs::create_dir_all(
        compatibility_destination
            .parent()
            .expect("compatibility destination has a parent"),
    )?;
    copy_regular_file(&root.join(COMPATIBILITY_SOURCE), &compatibility_destination)?;
    fs::rename(&staging, output).with_context(|| {
        format!(
            "publish readiness pack {} -> {}; staging was retained",
            staging.display(),
            output.display()
        )
    })?;
    Ok(())
}

fn stage_file(root: &Path, pack: &Path, source: &Path, path: &str) -> Result<StagedEvidence> {
    release_readiness::validate_manifest_path(root, &pack.join(MANIFEST_NAME))?;
    let canonical_pack = fs::canonicalize(pack)
        .with_context(|| format!("resolve readiness pack {}", pack.display()))?;
    anyhow::ensure!(canonical_pack.is_dir(), "readiness pack is not a directory");
    let relative = validate_relative_path(path)?;
    anyhow::ensure!(
        relative != Path::new(MANIFEST_NAME),
        "the readiness manifest cannot be replaced by readiness-pack stage"
    );
    let source_metadata = fs::symlink_metadata(source)
        .with_context(|| format!("inspect evidence source {}", source.display()))?;
    anyhow::ensure!(
        source_metadata.is_file() && !source_metadata.file_type().is_symlink(),
        "evidence source must be a regular non-symlink file: {}",
        source.display()
    );
    anyhow::ensure!(
        source_metadata.len() <= release_readiness::MAX_EVIDENCE_BYTES,
        "evidence source exceeds {} bytes: {}",
        release_readiness::MAX_EVIDENCE_BYTES,
        source.display()
    );
    let destination = prepare_destination(&canonical_pack, &relative)?;

    let destination_parent = destination
        .parent()
        .expect("validated staged evidence has a parent");
    let mut temporary = tempfile::Builder::new()
        .prefix(".rne-part-")
        .tempfile_in(destination_parent)
        .with_context(|| {
            format!(
                "create private staged evidence in {}",
                destination_parent.display()
            )
        })?;
    let staged = copy_and_hash(source, temporary.as_file_mut())?;
    temporary.as_file_mut().sync_all()?;
    temporary
        .persist_noclobber(&destination)
        .map_err(|error| error.error)
        .with_context(|| {
            format!(
                "publish staged evidence without replacing {}",
                destination.display()
            )
        })?;
    Ok(StagedEvidence {
        relative_path: path.to_string(),
        sha256: staged.0,
        size_bytes: staged.1,
    })
}

fn accept_installed_flagship(root: &Path, options: &AcceptInstalledFlagshipOptions) -> Result<()> {
    let manifest_path = options.pack.join(MANIFEST_NAME);
    release_readiness::validate_manifest_path(root, &manifest_path)?;
    let evidence = verify_installed_flagship_evidence(options)?;
    let unique_paths = [
        &evidence.release_archive.relative_path,
        &evidence.proof_bundle.relative_path,
        &evidence.submission_candidate.relative_path,
        &evidence.stdout_log.relative_path,
        &evidence.stderr_log.relative_path,
        &evidence.report.relative_path,
    ]
    .into_iter()
    .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        unique_paths.len() == 6,
        "installed flagship evidence roles must reference six distinct files"
    );

    let staged_path = |reference: &StagedEvidence| options.pack.join(&reference.relative_path);
    let report_path = staged_path(&evidence.report);
    let report_bytes = fs::read(&report_path)
        .with_context(|| format!("read staged flagship report {}", report_path.display()))?;
    release_artifacts::validate_staged_external_flagship_report(
        &report_bytes,
        release_artifacts::StagedExternalFlagshipReproduction {
            owner: &options.owner,
            repository: &options.repository,
            revision: &options.revision,
            measured_on: &options.measured_on,
            release_archive: &staged_path(&evidence.release_archive),
            proof_bundle: &staged_path(&evidence.proof_bundle),
            submission_candidate: &staged_path(&evidence.submission_candidate),
            stdout_log: &staged_path(&evidence.stdout_log),
            stderr_log: &staged_path(&evidence.stderr_log),
        },
    )?;

    let current = fs::read_to_string(&manifest_path)
        .with_context(|| format!("read readiness manifest {}", manifest_path.display()))?;
    let updated = manifest_with_installed_flagship_entry(&current, options, &evidence)?;
    atomically_replace_manifest(root, &options.pack, &current, &updated, || {
        anyhow::ensure!(
            verify_installed_flagship_evidence(options)? == evidence,
            "installed flagship evidence changed during acceptance"
        );
        Ok(())
    })
}

fn atomically_replace_manifest(
    root: &Path,
    pack: &Path,
    expected_current: &str,
    updated: &str,
    verify_evidence_unchanged: impl FnOnce() -> Result<()>,
) -> Result<()> {
    let manifest_path = pack.join(MANIFEST_NAME);
    let manifest_metadata = fs::symlink_metadata(&manifest_path)
        .with_context(|| format!("inspect readiness manifest {}", manifest_path.display()))?;
    anyhow::ensure!(
        manifest_metadata.is_file() && !manifest_metadata.file_type().is_symlink(),
        "readiness manifest must be a regular non-symlink file"
    );
    let mut temporary = tempfile::Builder::new()
        .prefix(".rne-readiness-manifest-")
        .tempfile_in(pack)
        .with_context(|| format!("create staged manifest in {}", pack.display()))?;
    temporary.write_all(updated.as_bytes())?;
    temporary
        .as_file()
        .set_permissions(manifest_metadata.permissions())?;
    temporary.as_file_mut().sync_all()?;
    release_readiness::validate_manifest_path(root, temporary.path())?;
    anyhow::ensure!(
        fs::read(&manifest_path)? == expected_current.as_bytes(),
        "readiness manifest changed during evidence acceptance"
    );
    verify_evidence_unchanged()?;
    temporary
        .persist(&manifest_path)
        .map_err(|error| error.error)
        .with_context(|| format!("atomically replace {}", manifest_path.display()))?;
    Ok(())
}

fn verify_installed_flagship_evidence(
    options: &AcceptInstalledFlagshipOptions,
) -> Result<AcceptedInstalledFlagshipEvidence> {
    Ok(AcceptedInstalledFlagshipEvidence {
        release_archive: verify_staged_file(&options.pack, &options.release_archive)?,
        proof_bundle: verify_staged_file(&options.pack, &options.proof_bundle)?,
        submission_candidate: verify_staged_file(&options.pack, &options.submission_candidate)?,
        stdout_log: verify_staged_file(&options.pack, &options.stdout_log)?,
        stderr_log: verify_staged_file(&options.pack, &options.stderr_log)?,
        report: verify_staged_file(&options.pack, &options.report)?,
    })
}

fn accept_external_simulator(root: &Path, options: &AcceptExternalSimulatorOptions) -> Result<()> {
    let manifest_path = options.pack.join(MANIFEST_NAME);
    release_readiness::validate_manifest_path(root, &manifest_path)?;
    let evidence = verify_external_simulator_evidence(options)?;
    let unique_paths = std::iter::once(&evidence.adapter.relative_path)
        .chain(std::iter::once(&evidence.task_spec.relative_path))
        .chain(std::iter::once(&evidence.runtime_manifest.relative_path))
        .chain(
            evidence
                .runtime_artifacts
                .iter()
                .map(|reference| &reference.relative_path),
        )
        .chain(std::iter::once(&evidence.conformance_report.relative_path))
        .chain(std::iter::once(&evidence.release_archive.relative_path))
        .chain(std::iter::once(
            &evidence.submission_candidate.relative_path,
        ))
        .chain(std::iter::once(&evidence.stdout_log.relative_path))
        .chain(std::iter::once(&evidence.stderr_log.relative_path))
        .chain(std::iter::once(&evidence.submission_report.relative_path))
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        unique_paths.len() == 12,
        "external simulator evidence roles must reference twelve distinct files"
    );

    let staged_path = |reference: &StagedEvidence| options.pack.join(&reference.relative_path);
    let runtime_artifacts = evidence
        .runtime_artifacts
        .iter()
        .map(staged_path)
        .collect::<Vec<_>>();
    let submission_report_path = staged_path(&evidence.submission_report);
    let report_bytes = fs::read(&submission_report_path).with_context(|| {
        format!(
            "read staged external simulator report {}",
            submission_report_path.display()
        )
    })?;
    external_simulator::validate_staged_submission_report(
        &report_bytes,
        external_simulator::StagedSubmission {
            owner: &options.owner,
            repository: &options.repository,
            revision: &options.revision,
            release_archive: &staged_path(&evidence.release_archive),
            adapter: &staged_path(&evidence.adapter),
            task_spec: &staged_path(&evidence.task_spec),
            runtime_manifest: &staged_path(&evidence.runtime_manifest),
            runtime_artifacts: &runtime_artifacts,
            conformance_report: &staged_path(&evidence.conformance_report),
            submission_candidate: &staged_path(&evidence.submission_candidate),
            stdout_log: &staged_path(&evidence.stdout_log),
            stderr_log: &staged_path(&evidence.stderr_log),
            adapter_arguments: &options.adapter_arguments,
        },
    )?;

    let current = fs::read_to_string(&manifest_path)
        .with_context(|| format!("read readiness manifest {}", manifest_path.display()))?;
    let updated = manifest_with_external_simulator_entry(&current, options, &evidence)?;
    atomically_replace_manifest(root, &options.pack, &current, &updated, || {
        anyhow::ensure!(
            verify_external_simulator_evidence(options)? == evidence,
            "external simulator evidence changed during acceptance"
        );
        Ok(())
    })
}

fn verify_external_simulator_evidence(
    options: &AcceptExternalSimulatorOptions,
) -> Result<AcceptedExternalSimulatorEvidence> {
    Ok(AcceptedExternalSimulatorEvidence {
        adapter: verify_staged_file(&options.pack, &options.adapter)?,
        task_spec: verify_staged_file(&options.pack, &options.task_spec)?,
        runtime_manifest: verify_staged_file(&options.pack, &options.runtime_manifest)?,
        runtime_artifacts: options
            .runtime_artifacts
            .iter()
            .map(|path| verify_staged_file(&options.pack, path))
            .collect::<Result<Vec<_>>>()?,
        conformance_report: verify_staged_file(&options.pack, &options.conformance_report)?,
        release_archive: verify_staged_file(&options.pack, &options.release_archive)?,
        submission_candidate: verify_staged_file(&options.pack, &options.submission_candidate)?,
        stdout_log: verify_staged_file(&options.pack, &options.stdout_log)?,
        stderr_log: verify_staged_file(&options.pack, &options.stderr_log)?,
        submission_report: verify_staged_file(&options.pack, &options.submission_report)?,
    })
}

fn manifest_with_external_simulator_entry(
    current: &str,
    options: &AcceptExternalSimulatorOptions,
    evidence: &AcceptedExternalSimulatorEvidence,
) -> Result<String> {
    anyhow::ensure!(
        !current.contains('\r') || !current.replace("\r\n", "").contains('\r'),
        "readiness manifest contains a non-canonical carriage return"
    );
    let mut updated = current.replace("\r\n", "\n");
    if !updated.ends_with('\n') {
        updated.push('\n');
    }
    updated.push('\n');
    updated.push_str("[[external_system]]\n");
    for (key, value) in [
        ("id", options.id.as_str()),
        ("owner", options.owner.as_str()),
        ("repository", options.repository.as_str()),
        ("revision", options.revision.as_str()),
    ] {
        updated.push_str(key);
        updated.push_str(" = ");
        updated.push_str(&toml_string(value));
        updated.push('\n');
    }
    updated.push_str("kind = \"simulator_adapter\"\n");
    for (key, reference) in [
        ("subject", &evidence.adapter),
        ("task_spec", &evidence.task_spec),
        ("runtime_manifest", &evidence.runtime_manifest),
    ] {
        updated.push_str(key);
        updated.push_str(" = ");
        updated.push_str(&inline_reference(reference));
        updated.push('\n');
    }
    let arguments = toml::Value::Array(
        options
            .adapter_arguments
            .iter()
            .cloned()
            .map(toml::Value::String)
            .collect(),
    );
    updated.push_str("adapter_arguments = ");
    updated.push_str(&arguments.to_string());
    updated.push('\n');
    updated.push_str("runtime_artifacts = [\n");
    for reference in &evidence.runtime_artifacts {
        updated.push_str("  ");
        updated.push_str(&inline_reference(reference));
        updated.push_str(",\n");
    }
    updated.push_str("]\n");
    for (key, reference) in [
        ("report", &evidence.conformance_report),
        ("release_archive", &evidence.release_archive),
        ("submission_candidate", &evidence.submission_candidate),
        ("stdout_log", &evidence.stdout_log),
        ("stderr_log", &evidence.stderr_log),
        ("submission_report", &evidence.submission_report),
    ] {
        updated.push_str(key);
        updated.push_str(" = ");
        updated.push_str(&inline_reference(reference));
        updated.push('\n');
    }
    Ok(updated)
}

fn verify_staged_file(pack: &Path, path: &str) -> Result<StagedEvidence> {
    let canonical_pack = fs::canonicalize(pack)
        .with_context(|| format!("resolve readiness pack {}", pack.display()))?;
    anyhow::ensure!(canonical_pack.is_dir(), "readiness pack is not a directory");
    let relative = validate_relative_path(path)?;
    anyhow::ensure!(
        relative != Path::new(MANIFEST_NAME),
        "readiness manifest cannot be accepted as evidence"
    );
    let absolute = pack.join(&relative);
    let metadata = fs::symlink_metadata(&absolute)
        .with_context(|| format!("inspect staged evidence {}", absolute.display()))?;
    anyhow::ensure!(
        metadata.is_file()
            && !metadata.file_type().is_symlink()
            && metadata.len() > 0
            && metadata.len() <= release_readiness::MAX_EVIDENCE_BYTES,
        "staged evidence must be a non-empty bounded regular non-symlink file: {}",
        absolute.display()
    );
    let canonical = fs::canonicalize(&absolute)
        .with_context(|| format!("resolve staged evidence {}", absolute.display()))?;
    anyhow::ensure!(
        canonical.starts_with(&canonical_pack),
        "staged evidence escapes the readiness pack: {}",
        absolute.display()
    );
    let (sha256, size_bytes) = hash_file(&absolute)?;
    anyhow::ensure!(
        size_bytes == metadata.len(),
        "staged evidence changed while it was hashed: {}",
        absolute.display()
    );
    Ok(StagedEvidence {
        relative_path: path.to_string(),
        sha256,
        size_bytes,
    })
}

fn hash_file(path: &Path) -> Result<(String, u64)> {
    let mut input = File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    let mut size_bytes = 0_u64;
    loop {
        let read = input.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        size_bytes = size_bytes
            .checked_add(u64::try_from(read)?)
            .context("staged evidence size overflow")?;
        anyhow::ensure!(
            size_bytes <= release_readiness::MAX_EVIDENCE_BYTES,
            "staged evidence grew beyond {} bytes while hashing",
            release_readiness::MAX_EVIDENCE_BYTES
        );
        hasher.update(&buffer[..read]);
    }
    Ok((format!("sha256:{:x}", hasher.finalize()), size_bytes))
}

fn manifest_with_installed_flagship_entry(
    current: &str,
    options: &AcceptInstalledFlagshipOptions,
    evidence: &AcceptedInstalledFlagshipEvidence,
) -> Result<String> {
    anyhow::ensure!(
        !current.contains('\r') || !current.replace("\r\n", "").contains('\r'),
        "readiness manifest contains a non-canonical carriage return"
    );
    let mut updated = current.replace("\r\n", "\n");
    const EMPTY: &str = "installed_flagship_reproduction = []\n";
    let empty_count = updated.matches(EMPTY).count();
    anyhow::ensure!(
        empty_count <= 1,
        "readiness manifest contains duplicate empty flagship declarations"
    );
    if empty_count == 1 {
        updated = updated.replacen(EMPTY, "", 1);
    }
    if !updated.ends_with('\n') {
        updated.push('\n');
    }
    updated.push('\n');
    updated.push_str("[[installed_flagship_reproduction]]\n");
    for (key, value) in [
        ("id", options.id.as_str()),
        ("owner", options.owner.as_str()),
        ("repository", options.repository.as_str()),
        ("revision", options.revision.as_str()),
        ("measured_on", options.measured_on.as_str()),
    ] {
        updated.push_str(key);
        updated.push_str(" = ");
        updated.push_str(&toml_string(value));
        updated.push('\n');
    }
    updated.push_str("author_assistance = false\n");
    for (key, reference) in [
        ("release_archive", &evidence.release_archive),
        ("proof_bundle", &evidence.proof_bundle),
        ("submission_candidate", &evidence.submission_candidate),
        ("stdout_log", &evidence.stdout_log),
        ("stderr_log", &evidence.stderr_log),
        ("report", &evidence.report),
    ] {
        updated.push_str(key);
        updated.push_str(" = ");
        updated.push_str(&inline_reference(reference));
        updated.push('\n');
    }
    Ok(updated)
}

fn validate_relative_path(path: &str) -> Result<PathBuf> {
    anyhow::ensure!(
        !path.is_empty() && !path.contains('\\') && !path.chars().any(char::is_control),
        "staged evidence path must be a non-empty forward-slash relative path"
    );
    anyhow::ensure!(
        path.split('/')
            .all(|component| !component.is_empty() && component != "." && component != ".."),
        "staged evidence path must use canonical non-empty components: {path}"
    );
    let relative = PathBuf::from(path);
    anyhow::ensure!(
        !relative.is_absolute()
            && relative
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "staged evidence path escapes its pack: {path}"
    );
    Ok(relative)
}

fn prepare_destination(pack: &Path, relative: &Path) -> Result<PathBuf> {
    let mut current = pack.to_path_buf();
    let parent = relative
        .parent()
        .expect("validated relative evidence path has a parent");
    for component in parent.components() {
        let Component::Normal(component) = component else {
            unreachable!("relative path was validated")
        };
        current.push(component);
        match fs::symlink_metadata(&current) {
            Ok(metadata) => anyhow::ensure!(
                metadata.is_dir() && !metadata.file_type().is_symlink(),
                "staged evidence parent must be a regular directory: {}",
                current.display()
            ),
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                fs::create_dir(&current).with_context(|| {
                    format!("create staged evidence directory {}", current.display())
                })?;
            }
            Err(error) => {
                return Err(error).with_context(|| format!("inspect {}", current.display()))
            }
        }
    }
    let destination = pack.join(relative);
    ensure_missing(&destination, "staged evidence")?;
    Ok(destination)
}

fn ensure_missing(path: &Path, label: &str) -> Result<()> {
    match fs::symlink_metadata(path) {
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error).with_context(|| format!("inspect {label} {}", path.display())),
        Ok(_) => bail!("refusing to overwrite existing {label}: {}", path.display()),
    }
}

fn copy_regular_file(source: &Path, destination: &Path) -> Result<()> {
    let metadata = fs::symlink_metadata(source)
        .with_context(|| format!("inspect baseline file {}", source.display()))?;
    anyhow::ensure!(
        metadata.is_file() && !metadata.file_type().is_symlink(),
        "baseline file must be a regular non-symlink file: {}",
        source.display()
    );
    fs::copy(source, destination).with_context(|| {
        format!(
            "copy readiness baseline {} -> {}",
            source.display(),
            destination.display()
        )
    })?;
    Ok(())
}

fn copy_and_hash(source: &Path, output: &mut File) -> Result<(String, u64)> {
    let mut input =
        File::open(source).with_context(|| format!("open evidence source {}", source.display()))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    let mut size_bytes = 0_u64;
    loop {
        let read = input.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        size_bytes = size_bytes
            .checked_add(u64::try_from(read)?)
            .context("staged evidence size overflow")?;
        anyhow::ensure!(
            size_bytes <= release_readiness::MAX_EVIDENCE_BYTES,
            "evidence source grew beyond {} bytes while staging",
            release_readiness::MAX_EVIDENCE_BYTES
        );
        output.write_all(&buffer[..read])?;
        hasher.update(&buffer[..read]);
    }
    output.flush()?;
    Ok((format!("sha256:{:x}", hasher.finalize()), size_bytes))
}

fn staging_sibling(path: &Path, marker: &str) -> Result<PathBuf> {
    let name = path
        .file_name()
        .context("readiness pack output must have a directory name")?;
    let mut staging_name = OsString::from(".");
    staging_name.push(name);
    staging_name.push(format!(".{marker}-{}", std::process::id()));
    Ok(path.with_file_name(staging_name))
}

fn inline_reference(staged: &StagedEvidence) -> String {
    format!(
        "{{ path = {}, sha256 = {} }}",
        toml_string(&staged.relative_path),
        toml_string(&staged.sha256)
    )
}

fn toml_string(value: &str) -> String {
    toml::Value::String(value.to_string()).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn accepted_options(pack: PathBuf) -> AcceptInstalledFlagshipOptions {
        AcceptInstalledFlagshipOptions {
            pack,
            id: "community-lab-a".to_string(),
            owner: "external-owner".to_string(),
            repository: "https://github.com/external-owner/rne-reproduction".to_string(),
            revision: "a".repeat(40),
            measured_on: "2026-08-27".to_string(),
            release_archive: "flagship/archive.zip".to_string(),
            proof_bundle: "flagship/proof.zip".to_string(),
            submission_candidate: "flagship/submission.json".to_string(),
            stdout_log: "flagship/stdout.txt".to_string(),
            stderr_log: "flagship/stderr.txt".to_string(),
            report: "flagship/report.json".to_string(),
        }
    }

    fn accepted_reference(path: &str, digest: u8) -> StagedEvidence {
        StagedEvidence {
            relative_path: path.to_string(),
            sha256: format!("sha256:{}", format!("{digest:x}").repeat(64)),
            size_bytes: 1,
        }
    }

    fn accepted_evidence() -> AcceptedInstalledFlagshipEvidence {
        AcceptedInstalledFlagshipEvidence {
            release_archive: accepted_reference("flagship/archive.zip", 1),
            proof_bundle: accepted_reference("flagship/proof.zip", 2),
            submission_candidate: accepted_reference("flagship/submission.json", 3),
            stdout_log: accepted_reference("flagship/stdout.txt", 4),
            stderr_log: accepted_reference("flagship/stderr.txt", 5),
            report: accepted_reference("flagship/report.json", 6),
        }
    }

    fn external_simulator_options(pack: PathBuf) -> AcceptExternalSimulatorOptions {
        AcceptExternalSimulatorOptions {
            pack,
            id: "external-gazebo".to_string(),
            owner: "external-owner".to_string(),
            repository: "https://github.com/external-owner/gazebo-adapter".to_string(),
            revision: "b".repeat(40),
            adapter: "simulator/adapter.py".to_string(),
            task_spec: "simulator/task.json".to_string(),
            runtime_manifest: "simulator/runtime.json".to_string(),
            runtime_artifacts: vec![
                "simulator/world.sdf".to_string(),
                "simulator/robot.sdf".to_string(),
                "simulator/adapter.json".to_string(),
            ],
            adapter_arguments: vec![
                "<adapter-subject>".to_string(),
                "--runtime-manifest".to_string(),
                "<runtime-manifest>".to_string(),
            ],
            conformance_report: "simulator/conformance.json".to_string(),
            release_archive: "simulator/release.tar.gz".to_string(),
            submission_candidate: "simulator/submission.json".to_string(),
            stdout_log: "simulator/stdout.txt".to_string(),
            stderr_log: "simulator/stderr.txt".to_string(),
            submission_report: "simulator/maintainer-report.json".to_string(),
        }
    }

    fn external_simulator_evidence() -> AcceptedExternalSimulatorEvidence {
        AcceptedExternalSimulatorEvidence {
            adapter: accepted_reference("simulator/adapter.py", 1),
            task_spec: accepted_reference("simulator/task.json", 2),
            runtime_manifest: accepted_reference("simulator/runtime.json", 3),
            runtime_artifacts: vec![
                accepted_reference("simulator/world.sdf", 4),
                accepted_reference("simulator/robot.sdf", 5),
                accepted_reference("simulator/adapter.json", 6),
            ],
            conformance_report: accepted_reference("simulator/conformance.json", 7),
            release_archive: accepted_reference("simulator/release.tar.gz", 8),
            submission_candidate: accepted_reference("simulator/submission.json", 9),
            stdout_log: accepted_reference("simulator/stdout.txt", 10),
            stderr_log: accepted_reference("simulator/stderr.txt", 11),
            submission_report: accepted_reference("simulator/maintainer-report.json", 12),
        }
    }

    #[test]
    fn initializes_an_external_pack_from_the_honest_baseline() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();

        assert_eq!(
            fs::read(pack.join(MANIFEST_NAME)).unwrap(),
            fs::read(root.join(BASELINE_MANIFEST)).unwrap()
        );
        assert_eq!(
            fs::read(pack.join(COMPATIBILITY_DESTINATION)).unwrap(),
            fs::read(root.join(COMPATIBILITY_SOURCE)).unwrap()
        );
        assert!(init_pack(&root, &pack).is_err());
    }

    #[test]
    fn stages_bytes_once_and_returns_a_canonical_reference() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("controller report.json");
        fs::write(&source, b"external evidence\n").unwrap();

        let staged = stage_file(&root, &pack, &source, "plugins/controller/report.json").unwrap();
        assert_eq!(
            staged,
            StagedEvidence {
                relative_path: "plugins/controller/report.json".to_string(),
                sha256: "sha256:e60b658f8e64bc576589f0b066c7b019a42b624650a328fcf598ef6cf02b4dbe"
                    .to_string(),
                size_bytes: 18,
            }
        );
        assert_eq!(
            fs::read(pack.join("plugins/controller/report.json")).unwrap(),
            b"external evidence\n"
        );
        assert_eq!(
            inline_reference(&staged),
            "{ path = \"plugins/controller/report.json\", sha256 = \"sha256:e60b658f8e64bc576589f0b066c7b019a42b624650a328fcf598ef6cf02b4dbe\" }"
        );
        assert!(stage_file(&root, &pack, &source, "plugins/controller/report.json").is_err());
    }

    #[test]
    fn accepted_flagship_entry_replaces_empty_placeholder_and_rejects_duplicate_id() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let manifest_path = pack.join(MANIFEST_NAME);
        let current = fs::read_to_string(&manifest_path).unwrap();
        let options = accepted_options(pack.clone());
        let evidence = accepted_evidence();

        let updated = manifest_with_installed_flagship_entry(&current, &options, &evidence)
            .expect("accepted entry");
        assert!(!updated.contains("installed_flagship_reproduction = []"));
        let value = updated.parse::<toml::Value>().expect("valid TOML");
        let entries = value["installed_flagship_reproduction"]
            .as_array()
            .expect("flagship entries");
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0]["id"].as_str(), Some("community-lab-a"));
        assert_eq!(entries[0]["author_assistance"].as_bool(), Some(false));
        assert_eq!(
            entries[0]["report"]["path"].as_str(),
            Some("flagship/report.json")
        );
        fs::write(&manifest_path, &updated).unwrap();
        release_readiness::validate_manifest_path(&root, &manifest_path).unwrap();

        let duplicate =
            manifest_with_installed_flagship_entry(&updated, &options, &evidence).unwrap();
        fs::write(&manifest_path, duplicate).unwrap();
        assert!(release_readiness::validate_manifest_path(&root, &manifest_path).is_err());
    }

    #[test]
    fn accepts_a_complete_flagship_chain_atomically() {
        fn raw_digest(path: &Path) -> String {
            hash_file(path)
                .unwrap()
                .0
                .strip_prefix("sha256:")
                .unwrap()
                .to_string()
        }

        fn member(path: &str, file: &Path) -> serde_json::Value {
            serde_json::json!({
                "path": path,
                "size_bytes": fs::metadata(file).unwrap().len(),
                "sha256": raw_digest(file),
            })
        }

        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let evidence_root = pack.join("flagship");
        fs::create_dir(&evidence_root).unwrap();
        let archive = evidence_root.join("archive.zip");
        let proof = evidence_root.join("proof.zip");
        let candidate = evidence_root.join("submission.json");
        let stdout = evidence_root.join("stdout.txt");
        let stderr = evidence_root.join("stderr.txt");
        let report = evidence_root.join("report.json");
        fs::write(&archive, b"archive").unwrap();
        fs::write(&proof, b"proof").unwrap();
        fs::write(&stdout, b"stdout").unwrap();
        fs::write(&stderr, b"stderr").unwrap();
        fs::write(
            &candidate,
            serde_json::to_vec_pretty(&serde_json::json!({
                "kind": "rne_external_flagship_submission_candidate",
                "schema_version": 2,
                "candidate_status": "not_accepted_pending_maintainer_verification",
                "author_assistance": false,
                "evidence_repository": {
                    "owner": "external-owner",
                    "url": "https://github.com/external-owner/rne-reproduction"
                },
                "measurement": {
                    "measured_on": "2026-08-27",
                    "machine_label": "community-lab-desktop-a",
                    "operating_system": "windows",
                    "architecture": "x86_64",
                    "release_target": "x86_64-pc-windows-msvc",
                    "elapsed_ms": 21_921,
                    "target_ms": 900_000
                },
                "release_archive": {
                    "url": "https://example.invalid/archive.zip",
                    "file_name": "archive.zip",
                    "size_bytes": fs::metadata(&archive).unwrap().len(),
                    "sha256": raw_digest(&archive)
                },
                "proof_bundle": {
                    "url": "https://example.invalid/proof.zip",
                    "file_name": "proof.zip",
                    "size_bytes": fs::metadata(&proof).unwrap().len(),
                    "sha256": raw_digest(&proof)
                },
                "required_proof_paths": [
                    "flagship-proof/installed-proof-report.json",
                    "flagship-proof/time-to-proof-report.json",
                    "flagship-proof/cross-backend-report.json",
                    "flagship-proof/recorded-shadow-proof.json",
                    "flagship-proof/failure-capsule/capsule.json"
                ],
                "reproduction": {
                    "commands": ["verify archive", "extract archive", "run proof"],
                    "exit_statuses": [0, 0, 0],
                    "stdout_log_path": "logs/stdout.txt",
                    "stderr_log_path": "logs/stderr.txt"
                }
            }))
            .unwrap(),
        )
        .unwrap();
        let retained = serde_json::json!({
            "path": "retained.json",
            "size_bytes": 1,
            "sha256": "0".repeat(64)
        });
        fs::write(
            &report,
            serde_json::to_vec_pretty(&serde_json::json!({
                "kind": "rne_external_flagship_reproduction_report",
                "schema_version": 2,
                "status": "passed",
                "owner": "external-owner",
                "repository": "https://github.com/external-owner/rne-reproduction",
                "revision": "a".repeat(40),
                "measured_on": "2026-08-27",
                "author_assistance": false,
                "release_version": "0.2.0",
                "release_revision": "b".repeat(40),
                "release_target": "x86_64-pc-windows-msvc",
                "machine_label": "community-lab-desktop-a",
                "operating_system": "windows",
                "architecture": "x86_64",
                "elapsed_ms": 21_921,
                "target_ms": 900_000,
                "task_id": "rne.flagship.mobile_lift_shared_aisle.v1",
                "physics_execution_paths": ["rapier_native", "mujoco_native"],
                "first_violation_step": 240,
                "first_violation_sim_time_ticks": 2_000_000_000_u64,
                "archive": member("archive.zip", &archive),
                "proof_bundle": member("proof.zip", &proof),
                "submission_candidate": member("submission.json", &candidate),
                "stdout_log": member("logs/stdout.txt", &stdout),
                "stderr_log": member("logs/stderr.txt", &stderr),
                "release_report": retained.clone(),
                "checksum_manifest": retained.clone(),
                "producer_executable": retained.clone(),
                "installed_proof_report": retained.clone(),
                "time_to_proof_report": retained.clone(),
                "cross_backend_report": retained.clone(),
                "failure_capsule_manifest": retained
            }))
            .unwrap(),
        )
        .unwrap();

        let options = accepted_options(pack.clone());
        accept_installed_flagship(&root, &options).expect("accept complete chain");
        let manifest_path = pack.join(MANIFEST_NAME);
        let accepted = fs::read_to_string(&manifest_path).unwrap();
        let value = accepted.parse::<toml::Value>().unwrap();
        assert_eq!(
            value["installed_flagship_reproduction"][0]["id"].as_str(),
            Some("community-lab-a")
        );
        release_readiness::validate_manifest_path(&root, &manifest_path).unwrap();

        assert!(accept_installed_flagship(&root, &options).is_err());
        assert_eq!(fs::read_to_string(&manifest_path).unwrap(), accepted);
    }

    #[test]
    fn external_simulator_entry_is_complete_ordered_and_duplicate_safe() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let manifest_path = pack.join(MANIFEST_NAME);
        let current = fs::read_to_string(&manifest_path).unwrap();
        let options = external_simulator_options(pack.clone());
        let evidence = external_simulator_evidence();

        let updated = manifest_with_external_simulator_entry(&current, &options, &evidence)
            .expect("external simulator entry");
        let value = updated.parse::<toml::Value>().expect("valid TOML");
        let entries = value["external_system"].as_array().unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0]["id"].as_str(), Some("external-gazebo"));
        assert_eq!(entries[0]["kind"].as_str(), Some("simulator_adapter"));
        assert_eq!(entries[0]["runtime_artifacts"].as_array().unwrap().len(), 3);
        assert_eq!(
            entries[0]["adapter_arguments"].as_array().unwrap()[2].as_str(),
            Some("<runtime-manifest>")
        );
        fs::write(&manifest_path, &updated).unwrap();
        release_readiness::validate_manifest_path(&root, &manifest_path).unwrap();

        let duplicate =
            manifest_with_external_simulator_entry(&updated, &options, &evidence).unwrap();
        fs::write(&manifest_path, duplicate).unwrap();
        assert!(release_readiness::validate_manifest_path(&root, &manifest_path).is_err());
    }

    #[test]
    fn rejects_unsafe_destinations_and_oversized_sources() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("evidence.bin");
        fs::write(&source, b"evidence").unwrap();

        for path in [
            "",
            "../escape",
            "nested\\escape",
            "/absolute",
            "nested//file",
            "nested/./file",
            "trailing/",
            "line\nbreak",
        ] {
            assert!(stage_file(&root, &pack, &source, path).is_err());
        }
        assert!(stage_file(&root, &pack, &source, MANIFEST_NAME).is_err());

        let oversized = temp.path().join("oversized.bin");
        File::create(&oversized)
            .unwrap()
            .set_len(release_readiness::MAX_EVIDENCE_BYTES + 1)
            .unwrap();
        assert!(stage_file(&root, &pack, &oversized, "large/file.bin").is_err());
        assert!(!pack.join("large/file.bin").exists());
    }

    #[cfg(unix)]
    #[test]
    fn rejects_symlinked_sources_and_destination_parents() {
        use std::os::unix::fs::symlink;

        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("evidence.bin");
        fs::write(&source, b"evidence").unwrap();
        let source_link = temp.path().join("evidence-link.bin");
        symlink(&source, &source_link).unwrap();
        assert!(stage_file(&root, &pack, &source_link, "safe/file.bin").is_err());

        let outside = temp.path().join("outside");
        fs::create_dir(&outside).unwrap();
        symlink(&outside, pack.join("linked")).unwrap();
        assert!(stage_file(&root, &pack, &source, "linked/file.bin").is_err());
        assert!(!outside.join("file.bin").exists());
    }

    #[test]
    fn option_parsers_require_each_input_exactly_once() {
        let root = Path::new("workspace");
        assert!(parse_init_options(&mut Vec::<String>::new().into_iter(), root).is_err());
        assert!(parse_stage_options(
            &mut ["--pack", "pack", "--source", "file"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
        assert!(parse_init_options(
            &mut ["--output", "one", "--output", "two"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
        assert!(parse_accept_installed_flagship_options(
            &mut ["--pack", "pack", "--id", "only-partial"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
        let mut complete = [
            "--pack",
            "pack",
            "--id",
            "run-a",
            "--owner",
            "external-owner",
            "--repository",
            "https://github.com/external-owner/reproduction",
            "--revision",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--measured-on",
            "2026-08-27",
            "--release-archive",
            "flagship/archive.zip",
            "--proof-bundle",
            "flagship/proof.zip",
            "--submission-candidate",
            "flagship/submission.json",
            "--stdout-log",
            "flagship/stdout.txt",
            "--stderr-log",
            "flagship/stderr.txt",
            "--report",
            "flagship/report.json",
        ]
        .map(str::to_string)
        .into_iter();
        let parsed = parse_accept_installed_flagship_options(&mut complete, root).unwrap();
        assert_eq!(parsed.pack, root.join("pack"));
        assert_eq!(parsed.id, "run-a");

        assert!(parse_accept_external_simulator_options(
            &mut ["--pack", "pack", "--runtime-artifact", "only-one"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
        let mut simulator = [
            "--pack",
            "pack",
            "--id",
            "gazebo-a",
            "--owner",
            "external-owner",
            "--repository",
            "https://github.com/external-owner/gazebo-adapter",
            "--revision",
            "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "--adapter",
            "sim/adapter.py",
            "--task-spec",
            "sim/task.json",
            "--runtime-manifest",
            "sim/runtime.json",
            "--runtime-artifact",
            "sim/world.sdf",
            "--runtime-artifact",
            "sim/robot.sdf",
            "--runtime-artifact",
            "sim/adapter.json",
            "--adapter-argument",
            "--runtime-manifest",
            "--conformance-report",
            "sim/conformance.json",
            "--release-archive",
            "sim/release.tar.gz",
            "--submission-candidate",
            "sim/submission.json",
            "--stdout-log",
            "sim/stdout.txt",
            "--stderr-log",
            "sim/stderr.txt",
            "--submission-report",
            "sim/maintainer.json",
        ]
        .map(str::to_string)
        .into_iter();
        let parsed = parse_accept_external_simulator_options(&mut simulator, root).unwrap();
        assert_eq!(parsed.runtime_artifacts.len(), 3);
        assert_eq!(parsed.adapter_arguments, ["--runtime-manifest"]);
    }
}
