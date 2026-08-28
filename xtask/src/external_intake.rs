//! Validates the public intake routes for independent RNE evidence.

use super::{workspace_root, RELEASE_VERSION};
use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::collections::BTreeSet;
use std::fs;
use std::path::{Component, Path, PathBuf};

const DEFAULT_REGISTRY: &str = "release/external-evidence-intake.toml";
const REGISTRY_SCHEMA_VERSION: u32 = 10;
const MAX_INTAKE_FILE_BYTES: u64 = 128 * 1024;
const RELEASE_URL_PREFIX: &str = "https://github.com/rsasaki0109/RoboSim/releases/tag/";

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct IntakeRegistry {
    schema_version: u32,
    qualifying_release_tag: String,
    qualifying_release_url: String,
    guide_path: String,
    installed_flagship_quickstart_path: String,
    installed_flagship_submission_template_path: String,
    external_project_submission_template_path: String,
    third_party_plugin_submission_template_path: String,
    external_simulator_submission_template_path: String,
    route: Vec<IntakeRoute>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct IntakeRoute {
    id: String,
    readiness_check: String,
    minimum_accepted: u32,
    issue_template: String,
    independent_owner_required: bool,
    author_assistance_allowed: bool,
    qualifying_kinds: Vec<String>,
    audited_nonqualifying_kinds: Vec<String>,
    required_metadata: Vec<String>,
    required_artifacts: Vec<String>,
    #[serde(default)]
    conditional_requirements: Vec<String>,
    form_fields: Vec<String>,
}

struct ExpectedRoute {
    id: &'static str,
    readiness_check: &'static str,
    minimum_accepted: u32,
    author_assistance_allowed: bool,
    qualifying_kinds: &'static [&'static str],
    audited_nonqualifying_kinds: &'static [&'static str],
    template: &'static str,
    metadata: &'static [&'static str],
    artifacts: &'static [&'static str],
    conditional: &'static [&'static str],
    form_fields: &'static [&'static str],
}

const EXPECTED_ROUTES: [ExpectedRoute; 4] = [
    ExpectedRoute {
        id: "installed_flagship_reproduction",
        readiness_check: "installed_flagship_reproduction",
        minimum_accepted: 1,
        author_assistance_allowed: false,
        qualifying_kinds: &[],
        audited_nonqualifying_kinds: &[],
        template: ".github/ISSUE_TEMPLATE/installed-flagship-reproduction.yml",
        metadata: &[
            "owner",
            "repository",
            "revision",
            "measured_on",
            "author_assistance",
        ],
        artifacts: &[
            "release_archive",
            "release_bundle",
            "proof_bundle",
            "submission_candidate",
            "stdout_log",
            "stderr_log",
            "installed_proof",
            "time_to_proof",
            "cross_backend_report",
            "failure_capsule",
        ],
        conditional: &[],
        form_fields: &[
            "independence",
            "repository",
            "revision",
            "rne_release",
            "platform",
            "machine",
            "measured_on",
            "release_archive",
            "proof_bundle",
            "submission_candidate",
            "reproduction_command",
            "verification",
        ],
    },
    ExpectedRoute {
        id: "external_project",
        readiness_check: "external_projects",
        minimum_accepted: 2,
        author_assistance_allowed: false,
        qualifying_kinds: &[],
        audited_nonqualifying_kinds: &[],
        template: ".github/ISSUE_TEMPLATE/external-project-evidence.yml",
        metadata: &[
            "owner",
            "repository",
            "revision",
            "first_used_on",
            "last_verified_on",
            "author_assistance",
        ],
        artifacts: &[
            "release_archive",
            "task_spec",
            "failure_capsule",
            "submission_candidate",
            "stdout_log",
            "stderr_log",
            "maintainer_report",
        ],
        conditional: &[],
        form_fields: &[
            "independence",
            "repository",
            "revision",
            "rne_release",
            "release_archive",
            "first_used_on",
            "last_verified_on",
            "task_spec",
            "failure_capsule",
            "submission_candidate",
            "stdout_log",
            "stderr_log",
            "reproduction_command",
            "verification",
        ],
    },
    ExpectedRoute {
        id: "third_party_plugin",
        readiness_check: "third_party_plugin",
        minimum_accepted: 1,
        author_assistance_allowed: true,
        qualifying_kinds: &[],
        audited_nonqualifying_kinds: &[],
        template: ".github/ISSUE_TEMPLATE/third-party-plugin-evidence.yml",
        metadata: &["owner", "repository", "revision"],
        artifacts: &[
            "release_archive",
            "library",
            "manifest",
            "report",
            "submission_candidate",
            "stdout_log",
            "stderr_log",
            "maintainer_report",
        ],
        conditional: &[],
        form_fields: &[
            "independence",
            "repository",
            "revision",
            "rne_release",
            "platform",
            "release_archive",
            "library",
            "manifest",
            "report",
            "submission_candidate",
            "stdout_log",
            "stderr_log",
            "conformance_command",
            "verification",
        ],
    },
    ExpectedRoute {
        id: "external_system",
        readiness_check: "external_system",
        minimum_accepted: 1,
        author_assistance_allowed: true,
        qualifying_kinds: &["physics_backend", "hardware_adapter", "simulator_adapter"],
        audited_nonqualifying_kinds: &["accelerator_adapter"],
        template: ".github/ISSUE_TEMPLATE/external-system-evidence.yml",
        metadata: &["owner", "repository", "revision", "kind"],
        artifacts: &["subject", "report"],
        conditional: &[
            "hardware_adapter.task_spec",
            "hardware_adapter.adapter_arguments",
            "hardware_adapter.safety_authorization",
            "simulator_adapter.task_spec",
            "simulator_adapter.adapter_arguments",
            "simulator_adapter.runtime_manifest",
            "simulator_adapter.runtime_artifacts",
            "simulator_adapter.release_archive",
            "simulator_adapter.submission_candidate",
            "simulator_adapter.stdout_log",
            "simulator_adapter.stderr_log",
            "simulator_adapter.maintainer_report",
            "accelerator_adapter.task_spec",
            "accelerator_adapter.adapter_arguments",
            "accelerator_adapter.accelerator_manifest",
            "accelerator_adapter.runtime_contract",
        ],
        form_fields: &[
            "independence",
            "kind",
            "repository",
            "revision",
            "rne_release",
            "platform",
            "subject",
            "task_spec",
            "adapter_arguments",
            "simulator_runtime_manifest",
            "simulator_runtime_artifacts",
            "simulator_submission",
            "accelerator_manifest",
            "runtime_contract",
            "report",
            "conformance_command",
            "safety",
            "verification",
        ],
    },
];

/// Validates the committed external-evidence intake contract.
pub(crate) fn validate_committed(root: &Path) -> Result<()> {
    validate_registry(root, &root.join(DEFAULT_REGISTRY))
}

/// Runs the standalone external-evidence intake validation command.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = workspace_root()?;
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
                println!("external-intake-check [--registry PATH]");
                return Ok(());
            }
            other => bail!("unknown external-intake-check argument: {other}"),
        }
    }
    validate_registry(&root, &registry)?;
    println!(
        "external evidence intake contract ok: routes={} registry={}",
        EXPECTED_ROUTES.len(),
        registry.display()
    );
    Ok(())
}

fn validate_registry(root: &Path, path: &Path) -> Result<()> {
    let bytes = fs::read(path)
        .with_context(|| format!("read external evidence intake registry {}", path.display()))?;
    let registry: IntakeRegistry = toml::from_str(
        std::str::from_utf8(&bytes).context("external evidence intake registry is not UTF-8")?,
    )
    .context("parse external evidence intake registry")?;
    anyhow::ensure!(
        registry.schema_version == REGISTRY_SCHEMA_VERSION,
        "external evidence intake registry schema must be {REGISTRY_SCHEMA_VERSION}"
    );
    let qualifying_release_tag = format!("v{RELEASE_VERSION}");
    let qualifying_release_url = format!("{RELEASE_URL_PREFIX}{qualifying_release_tag}");
    anyhow::ensure!(
        registry.qualifying_release_tag == qualifying_release_tag
            && registry.qualifying_release_url == qualifying_release_url,
        "external evidence intake campaign must name the current immutable release"
    );
    anyhow::ensure!(
        registry.route.len() == EXPECTED_ROUTES.len(),
        "external evidence intake registry must contain exactly {} routes",
        EXPECTED_ROUTES.len()
    );
    let guide = read_repository_file(root, &registry.guide_path)?;
    anyhow::ensure!(
        guide.starts_with("# External evidence intake\n"),
        "external evidence intake guide title drifted"
    );
    anyhow::ensure!(
        guide.contains(&registry.qualifying_release_tag)
            && guide.contains(&registry.qualifying_release_url)
            && guide.contains("do not open an evidence issue"),
        "external intake guide must expose the qualifying release and pre-publication boundary"
    );
    let flagship_quickstart =
        read_repository_file(root, &registry.installed_flagship_quickstart_path)?;
    anyhow::ensure!(
        flagship_quickstart.starts_with("# Installed flagship reproduction quickstart\n"),
        "installed flagship quickstart title drifted"
    );
    anyhow::ensure!(
        flagship_quickstart.contains(&registry.installed_flagship_submission_template_path)
            && flagship_quickstart.contains("candidate submission")
            && flagship_quickstart.contains("not accepted evidence")
            && flagship_quickstart.contains("self-referential digest")
            && flagship_quickstart.contains("has no Git"),
        "installed flagship quickstart must bind the candidate template and non-acceptance boundary"
    );
    validate_flagship_submission_template(&read_repository_file(
        root,
        &registry.installed_flagship_submission_template_path,
    )?)?;
    validate_project_submission_template(&read_repository_file(
        root,
        &registry.external_project_submission_template_path,
    )?)?;
    validate_plugin_submission_template(&read_repository_file(
        root,
        &registry.third_party_plugin_submission_template_path,
    )?)?;
    validate_simulator_submission_template(&read_repository_file(
        root,
        &registry.external_simulator_submission_template_path,
    )?)?;
    anyhow::ensure!(
        guide.contains(&registry.installed_flagship_quickstart_path)
            || guide.contains("EXTERNAL_FLAGSHIP_REPRODUCTION.md"),
        "external intake guide must link the installed flagship quickstart"
    );
    let readme = read_repository_file(root, "README.md")?;
    validate_readme_discovery(&readme, &registry)?;

    let mut ids = BTreeSet::new();
    for (route, expected) in registry.route.iter().zip(&EXPECTED_ROUTES) {
        validate_route(route, expected)?;
        anyhow::ensure!(
            ids.insert(route.id.as_str()),
            "duplicate external evidence intake route: {}",
            route.id
        );
        let form = read_repository_file(root, &route.issue_template)?;
        validate_issue_form(&form, route)?;
        anyhow::ensure!(
            guide.contains(&format!("## `{}`", route.id)),
            "external evidence intake guide is missing route {}",
            route.id
        );
        anyhow::ensure!(
            guide.contains(&route.issue_template),
            "external evidence intake guide does not link template {}",
            route.issue_template
        );
    }
    Ok(())
}

fn validate_project_submission_template(text: &str) -> Result<()> {
    let value: serde_json::Value =
        serde_json::from_str(text).context("parse external-project submission template")?;
    anyhow::ensure!(
        value.get("kind").and_then(serde_json::Value::as_str)
            == Some("rne_external_project_submission_candidate")
            && value
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(1)
            && value
                .get("candidate_status")
                .and_then(serde_json::Value::as_str)
                == Some("not_accepted_pending_maintainer_verification")
            && value
                .get("author_assistance")
                .and_then(serde_json::Value::as_bool)
                == Some(false),
        "external-project submission template identity or independence boundary drifted"
    );
    let repository = value
        .get("evidence_repository")
        .and_then(serde_json::Value::as_object)
        .context("external-project submission template omitted evidence_repository")?;
    anyhow::ensure!(
        repository.keys().map(String::as_str).eq(["owner", "url"])
            && repository.values().all(serde_json::Value::is_null),
        "external-project submission template must keep Git revision outside the candidate"
    );
    for object in ["release", "platform", "usage", "project", "reproduction"] {
        anyhow::ensure!(
            value.get(object).is_some_and(serde_json::Value::is_object),
            "external-project submission template omitted {object}"
        );
    }
    Ok(())
}

fn validate_plugin_submission_template(text: &str) -> Result<()> {
    let value: serde_json::Value =
        serde_json::from_str(text).context("parse controller-plugin submission template")?;
    anyhow::ensure!(
        value.get("kind").and_then(serde_json::Value::as_str)
            == Some("rne_external_controller_plugin_submission_candidate")
            && value
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(1)
            && value
                .get("candidate_status")
                .and_then(serde_json::Value::as_str)
                == Some("not_accepted_pending_maintainer_verification"),
        "controller-plugin submission template identity drifted"
    );
    let repository = value
        .get("evidence_repository")
        .and_then(serde_json::Value::as_object)
        .context("controller-plugin submission template omitted evidence_repository")?;
    anyhow::ensure!(
        repository.keys().map(String::as_str).eq(["owner", "url"])
            && repository.values().all(serde_json::Value::is_null),
        "controller-plugin submission template must keep Git revision outside the candidate"
    );
    for object in ["release", "platform", "artifacts", "reproduction"] {
        anyhow::ensure!(
            value.get(object).is_some_and(serde_json::Value::is_object),
            "controller-plugin submission template omitted {object}"
        );
    }
    Ok(())
}

fn validate_simulator_submission_template(text: &str) -> Result<()> {
    let value: serde_json::Value =
        serde_json::from_str(text).context("parse simulator-adapter submission template")?;
    anyhow::ensure!(
        value.get("kind").and_then(serde_json::Value::as_str)
            == Some("rne_external_simulator_adapter_submission_candidate")
            && value
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(1)
            && value
                .get("candidate_status")
                .and_then(serde_json::Value::as_str)
                == Some("not_accepted_pending_maintainer_verification"),
        "simulator-adapter submission template identity drifted"
    );
    let repository = value
        .get("evidence_repository")
        .and_then(serde_json::Value::as_object)
        .context("simulator-adapter submission template omitted evidence_repository")?;
    let runtime_artifacts = value
        .pointer("/artifacts/runtime_artifacts")
        .and_then(serde_json::Value::as_array)
        .context("simulator-adapter submission template omitted runtime artifacts")?;
    anyhow::ensure!(
        repository.keys().map(String::as_str).eq(["owner", "url"])
            && repository.values().all(serde_json::Value::is_null)
            && runtime_artifacts.len() == 3
            && value
                .get("adapter_arguments")
                .is_some_and(serde_json::Value::is_array),
        "simulator-adapter submission template must be acyclic and retain three ordered artifacts"
    );
    Ok(())
}

fn validate_flagship_submission_template(text: &str) -> Result<()> {
    let value: serde_json::Value =
        serde_json::from_str(text).context("parse installed flagship submission template")?;
    anyhow::ensure!(
        value.get("kind").and_then(serde_json::Value::as_str)
            == Some("rne_external_flagship_submission_candidate")
            && value
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(2)
            && value
                .get("candidate_status")
                .and_then(serde_json::Value::as_str)
                == Some("not_accepted_pending_maintainer_verification")
            && value
                .get("author_assistance")
                .and_then(serde_json::Value::as_bool)
                == Some(false),
        "installed flagship submission template identity or non-acceptance policy drifted"
    );
    let required_paths = value
        .get("required_proof_paths")
        .and_then(serde_json::Value::as_array)
        .context("installed flagship submission template omitted required_proof_paths")?;
    let actual = required_paths
        .iter()
        .map(|path| path.as_str().context("required proof path is not a string"))
        .collect::<Result<Vec<_>>>()?;
    anyhow::ensure!(
        actual
            == [
                "flagship-proof/installed-proof-report.json",
                "flagship-proof/time-to-proof-report.json",
                "flagship-proof/cross-backend-report.json",
                "flagship-proof/recorded-shadow-proof.json",
                "flagship-proof/failure-capsule/capsule.json",
            ],
        "installed flagship submission template proof paths drifted"
    );
    let repository = value
        .get("evidence_repository")
        .and_then(serde_json::Value::as_object)
        .context("installed flagship submission template omitted evidence_repository")?;
    anyhow::ensure!(
        repository.keys().map(String::as_str).eq(["owner", "url"])
            && repository.values().all(serde_json::Value::is_null),
        "installed flagship submission template must not embed its own Git revision"
    );
    for object in [
        "evidence_repository",
        "measurement",
        "release_archive",
        "proof_bundle",
        "reproduction",
    ] {
        anyhow::ensure!(
            value.get(object).is_some_and(serde_json::Value::is_object),
            "installed flagship submission template omitted {object}"
        );
    }
    Ok(())
}

fn validate_readme_discovery(readme: &str, registry: &IntakeRegistry) -> Result<()> {
    let heading = readme
        .find("## Independent validation wanted\n")
        .context("README is missing the independent-validation heading")?;
    let quickstart = readme
        .find("## Quickstart\n")
        .context("README is missing the quickstart heading")?;
    anyhow::ensure!(
        heading < quickstart,
        "README independent-validation routes must appear before Quickstart"
    );
    anyhow::ensure!(
        readme.contains("docs/EXTERNAL_EVIDENCE_INTAKE.md"),
        "README independent-validation section must link the intake guide"
    );
    anyhow::ensure!(
        readme.contains(&registry.qualifying_release_tag)
            && readme.contains(&registry.qualifying_release_url)
            && readme.contains("do not open an evidence issue"),
        "README must expose the qualifying release and pre-publication boundary"
    );
    for route in &registry.route {
        let template = route
            .issue_template
            .rsplit('/')
            .next()
            .context("external intake issue template is missing its file name")?;
        let issue_url =
            format!("https://github.com/rsasaki0109/RoboSim/issues/new?template={template}");
        anyhow::ensure!(
            readme.contains(&issue_url),
            "README does not expose the public issue URL for {}",
            route.id
        );
    }
    anyhow::ensure!(
        readme.contains("Opening an issue is only the start of review:")
            && readme.contains("does not imply acceptance"),
        "README must not present an issue submission as accepted evidence"
    );
    Ok(())
}

fn validate_route(route: &IntakeRoute, expected: &ExpectedRoute) -> Result<()> {
    anyhow::ensure!(
        route.id == expected.id,
        "external intake route order or id drifted"
    );
    anyhow::ensure!(
        route.readiness_check == expected.readiness_check,
        "external intake readiness check drifted for {}",
        route.id
    );
    anyhow::ensure!(
        route.minimum_accepted == expected.minimum_accepted,
        "external intake threshold drifted for {}",
        route.id
    );
    anyhow::ensure!(
        route.issue_template == expected.template,
        "external intake issue template drifted for {}",
        route.id
    );
    anyhow::ensure!(
        route.independent_owner_required,
        "external intake route {} must require independent ownership",
        route.id
    );
    anyhow::ensure!(
        route.author_assistance_allowed == expected.author_assistance_allowed,
        "external intake author-assistance policy drifted for {}",
        route.id
    );
    ensure_exact(
        &route.qualifying_kinds,
        expected.qualifying_kinds,
        "qualifying kinds",
        &route.id,
    )?;
    ensure_exact(
        &route.audited_nonqualifying_kinds,
        expected.audited_nonqualifying_kinds,
        "audited nonqualifying kinds",
        &route.id,
    )?;
    ensure_exact(
        &route.required_metadata,
        expected.metadata,
        "metadata",
        &route.id,
    )?;
    ensure_exact(
        &route.required_artifacts,
        expected.artifacts,
        "artifacts",
        &route.id,
    )?;
    ensure_exact(
        &route.conditional_requirements,
        expected.conditional,
        "conditional requirements",
        &route.id,
    )?;
    ensure_exact(
        &route.form_fields,
        expected.form_fields,
        "form fields",
        &route.id,
    )
}

fn ensure_exact(actual: &[String], expected: &[&str], field: &str, route: &str) -> Result<()> {
    anyhow::ensure!(
        actual
            .iter()
            .map(String::as_str)
            .eq(expected.iter().copied()),
        "external intake {field} drifted for {route}"
    );
    anyhow::ensure!(
        actual.iter().collect::<BTreeSet<_>>().len() == actual.len(),
        "external intake {field} contains duplicates for {route}"
    );
    Ok(())
}

fn read_repository_file(root: &Path, relative: &str) -> Result<String> {
    validate_relative_path(relative)?;
    let canonical_root =
        fs::canonicalize(root).with_context(|| format!("resolve repository {}", root.display()))?;
    let candidate = root.join(relative);
    let metadata = fs::symlink_metadata(&candidate)
        .with_context(|| format!("inspect external intake file {}", candidate.display()))?;
    anyhow::ensure!(
        metadata.is_file() && !metadata.file_type().is_symlink(),
        "external intake file must be a regular non-symlink file: {}",
        candidate.display()
    );
    anyhow::ensure!(
        metadata.len() <= MAX_INTAKE_FILE_BYTES,
        "external intake file exceeds {MAX_INTAKE_FILE_BYTES} bytes: {}",
        candidate.display()
    );
    let canonical = fs::canonicalize(&candidate)?;
    anyhow::ensure!(
        canonical.starts_with(&canonical_root),
        "external intake file escaped the repository: {}",
        candidate.display()
    );
    let text = fs::read_to_string(canonical)?;
    anyhow::ensure!(
        !text.contains('\r') && !text.contains('\t'),
        "external intake text must use LF and spaces: {relative}"
    );
    Ok(text)
}

fn validate_relative_path(path: &str) -> Result<()> {
    anyhow::ensure!(
        !path.is_empty() && !path.contains('\\') && !path.chars().any(char::is_control),
        "external intake path must be a non-empty forward-slash relative path"
    );
    anyhow::ensure!(
        path.split('/')
            .all(|component| !component.is_empty() && component != "." && component != ".."),
        "external intake path must use canonical components: {path}"
    );
    let parsed = Path::new(path);
    anyhow::ensure!(
        !parsed.is_absolute()
            && parsed
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "external intake path escaped the repository: {path}"
    );
    Ok(())
}

fn validate_issue_form(text: &str, route: &IntakeRoute) -> Result<()> {
    let qualifying_release_tag = format!("v{RELEASE_VERSION}");
    let qualifying_release_url = format!("{RELEASE_URL_PREFIX}{qualifying_release_tag}");
    anyhow::ensure!(
        text.contains(&qualifying_release_tag)
            && text.contains(&qualifying_release_url)
            && text.contains("Do not submit before these assets are published."),
        "external intake issue form must expose the qualifying release and publication boundary"
    );
    anyhow::ensure!(
        text.starts_with("name: "),
        "issue form must start with name"
    );
    for key in ["description: ", "title: ", "body:"] {
        anyhow::ensure!(
            text.lines().any(|line| line.starts_with(key)),
            "issue form missing {key}"
        );
    }
    let mut fields = Vec::new();
    let lines = text.lines().collect::<Vec<_>>();
    for (index, line) in lines.iter().enumerate() {
        let Some(id) = line.trim().strip_prefix("id: ") else {
            continue;
        };
        anyhow::ensure!(
            !id.is_empty()
                && id
                    .bytes()
                    .all(|byte| byte.is_ascii_lowercase() || byte == b'_'),
            "issue form contains invalid field id {id:?}"
        );
        let end = lines[index + 1..]
            .iter()
            .position(|next| next.trim().starts_with("id: "))
            .map_or(lines.len(), |offset| index + 1 + offset);
        anyhow::ensure!(
            lines[index + 1..end].windows(2).any(|pair| {
                pair[0].trim() == "validations:" && pair[1].trim() == "required: true"
            }),
            "issue form field {id} is not required"
        );
        fields.push(id);
    }
    anyhow::ensure!(
        fields
            .iter()
            .copied()
            .eq(route.form_fields.iter().map(String::as_str)),
        "issue form field order drifted for {}",
        route.id
    );
    anyhow::ensure!(
        fields.iter().collect::<BTreeSet<_>>().len() == fields.len(),
        "issue form contains duplicate field ids for {}",
        route.id
    );
    anyhow::ensure!(
        text.contains("A submitted issue is not acceptance evidence"),
        "issue form must say that submission is not acceptance"
    );
    if route.id == "external_system" {
        let physics = text
            .find("        - physics_backend\n")
            .context("external system form omitted physics_backend")?;
        let hardware = text
            .find("        - hardware_adapter\n")
            .context("external system form omitted hardware_adapter")?;
        let simulator = text
            .find("        - simulator_adapter\n")
            .context("external system form omitted simulator_adapter")?;
        let accelerator = text
            .find("        - accelerator_adapter\n")
            .context("external system form omitted accelerator_adapter")?;
        anyhow::ensure!(
            physics < hardware && hardware < simulator && simulator < accelerator,
            "external system kind options are not canonical"
        );
    }
    if route.id == "installed_flagship_reproduction" {
        anyhow::ensure!(
            text.contains("Keep the candidate JSON outside this archive")
                && !text.contains("completed candidate submission JSON"),
            "installed flagship form must not create a self-referential proof bundle"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_external_intake_contract_is_complete() {
        validate_committed(&workspace_root().unwrap()).unwrap();
    }

    #[test]
    fn external_system_eligibility_and_form_kinds_cannot_drift() {
        let root = workspace_root().unwrap();
        let registry: IntakeRegistry =
            toml::from_str(&fs::read_to_string(root.join(DEFAULT_REGISTRY)).unwrap()).unwrap();
        let route = registry
            .route
            .iter()
            .find(|route| route.id == "external_system")
            .unwrap();
        assert_eq!(
            route.qualifying_kinds,
            ["physics_backend", "hardware_adapter", "simulator_adapter"]
        );
        assert_eq!(route.audited_nonqualifying_kinds, ["accelerator_adapter"]);
        let form = fs::read_to_string(root.join(&route.issue_template)).unwrap();
        validate_issue_form(&form, route).unwrap();
        assert!(
            validate_issue_form(&form.replace("        - accelerator_adapter\n", ""), route)
                .is_err()
        );
        assert!(
            validate_issue_form(&form.replace("        - simulator_adapter\n", ""), route).is_err()
        );
    }

    #[test]
    fn readme_discovery_cannot_drop_a_route_or_acceptance_warning() {
        let root = workspace_root().unwrap();
        let registry: IntakeRegistry =
            toml::from_str(&fs::read_to_string(root.join(DEFAULT_REGISTRY)).unwrap()).unwrap();
        let readme = fs::read_to_string(root.join("README.md")).unwrap();
        validate_readme_discovery(&readme, &registry).unwrap();

        assert!(validate_readme_discovery(
            &readme.replace("external-project-evidence.yml", "missing.yml"),
            &registry
        )
        .is_err());
        assert!(validate_readme_discovery(
            &readme.replace("does not imply acceptance", "is accepted evidence"),
            &registry
        )
        .is_err());
        assert!(validate_readme_discovery(&readme.replace("v0.2.0", "v0.1.0"), &registry).is_err());
    }

    #[test]
    fn issue_form_fields_must_be_required_and_ordered() {
        let route = IntakeRoute {
            id: "test".to_string(),
            readiness_check: "test".to_string(),
            minimum_accepted: 1,
            issue_template: "test.yml".to_string(),
            independent_owner_required: true,
            author_assistance_allowed: false,
            qualifying_kinds: Vec::new(),
            audited_nonqualifying_kinds: Vec::new(),
            required_metadata: Vec::new(),
            required_artifacts: Vec::new(),
            conditional_requirements: Vec::new(),
            form_fields: vec!["one".to_string(), "two".to_string()],
        };
        let valid = "name: Test\ndescription: Test\ntitle: Test\nbody:\n  - type: input\n    id: one\n    validations:\n      required: true\n  - type: input\n    id: two\n    validations:\n      required: true\n# Current campaign: v0.2.0 — https://github.com/rsasaki0109/RoboSim/releases/tag/v0.2.0\n# Do not submit before these assets are published.\n# A submitted issue is not acceptance evidence\n";
        validate_issue_form(valid, &route).unwrap();
        assert!(
            validate_issue_form(&valid.replace("required: true", "required: false"), &route)
                .is_err()
        );
        assert!(validate_issue_form(&valid.replace("id: one", "id: two"), &route).is_err());
    }

    #[test]
    fn registry_and_paths_fail_closed() {
        let unknown = "schema_version = 3\nguide_path = \"guide.md\"\nunknown = true\n";
        assert!(toml::from_str::<IntakeRegistry>(unknown).is_err());
        for path in [
            "",
            "../guide.md",
            "docs\\guide.md",
            "docs//guide.md",
            "/guide.md",
        ] {
            assert!(validate_relative_path(path).is_err());
        }
    }
}
